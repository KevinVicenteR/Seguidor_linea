#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "FirmwareConfig.h"
#include "ScenarioRunner.h"
#include "TinyMlp.h"
#include "TrackLibrary.h"

using namespace sim;

namespace {

constexpr int kInputs = 8;
constexpr int kHidden = 8;
constexpr int kResidualLimit = 80;

struct Sample {
  int8_t input[kInputs];
  float target;
  bool validation;
};

struct RobotCase {
  std::size_t track;
  RobotParams robot;
  uint32_t seed;
  bool validation;
};

struct FloatNet {
  double w1[kHidden][kInputs];
  double b1[kHidden];
  double w2[kHidden];
  double b2;
  double forward(const int8_t* x, double* hidden) const {
    double y = b2;
    for (int j = 0; j < kHidden; j++) {
      double s = b1[j];
      for (int i = 0; i < kInputs; i++) s += w1[j][i] * (x[i] / 127.0);
      hidden[j] = s > 0 ? s : 0;
      y += w2[j] * hidden[j];
    }
    return y;
  }
};

struct QuantNet {
  std::vector<int8_t> w1;
  std::vector<int32_t> b1;
  std::vector<int16_t> w2;
  std::vector<int32_t> b2;
  uint8_t shift1 = 0;
  uint8_t shift2 = 0;
  atlas::TinyMlpWeights weights() const {
    return {kInputs, kHidden, 1, w1.data(), b1.data(), shift1, w2.data(), b2.data(), shift2};
  }
};

QuantNet zeroNet() {
  QuantNet q;
  q.w1.assign(kHidden * kInputs, 0);
  q.b1.assign(kHidden, 0);
  q.w2.assign(kHidden, 0);
  q.b2.assign(1, 0);
  return q;
}

QuantNet quantize(const FloatNet& n) {
  QuantNet q;
  double maxW1 = 1e-9, maxB1 = 1e-9, maxW2 = 1e-9;
  for (int j = 0; j < kHidden; j++) {
    for (int i = 0; i < kInputs; i++) maxW1 = std::max(maxW1, std::fabs(n.w1[j][i]));
    maxB1 = std::max(maxB1, std::fabs(n.b1[j]));
    maxW2 = std::max(maxW2, std::fabs(n.w2[j]));
  }
  const double a = 127.0 / maxW1;
  const double sumScale = a * 127.0 / 127.0;
  int s1 = 0;
  while (s1 < 20 && sumScale * 127.0 * kInputs / (1 << s1) > 8000.0) s1++;
  const double hiddenScale = a / (1 << s1) * 127.0;
  q.shift1 = static_cast<uint8_t>(s1);
  for (int j = 0; j < kHidden; j++) {
    for (int i = 0; i < kInputs; i++) q.w1.push_back(static_cast<int8_t>(std::lround(n.w1[j][i] * a)));
    q.b1.push_back(static_cast<int32_t>(std::lround(n.b1[j] * a * 127.0)));
  }
  int s2 = 0;
  while (s2 < 30 && maxW2 * (1 << (s2 + 1)) / hiddenScale * kResidualLimit < 16000.0) s2++;
  q.shift2 = static_cast<uint8_t>(s2);
  const double c = (1 << s2) / hiddenScale;
  for (int j = 0; j < kHidden; j++) q.w2.push_back(static_cast<int16_t>(std::lround(n.w2[j] * kResidualLimit * c)));
  q.b2.push_back(static_cast<int32_t>(std::lround(n.b2 * kResidualLimit * (1 << s2))));
  return q;
}

int16_t clampWheel(int v) { return static_cast<int16_t>(std::max(-134, std::min(255, v))); }

int deadband(int pwm) {
  if (pwm == 0) return 0;
  const int m = std::abs(pwm);
  const int c = 40 + m * (255 - 40) / 255;
  return pwm < 0 ? -c : c;
}

double rolloutCost(RobotModel robot, const Track& track, int base, int correction, int deviation) {
  DriveInputs in;
  in.leftPwm = deadband(clampWheel(base + correction));
  in.rightPwm = deadband(clampWheel(base - correction));
  in.driversEnabled = true;
  constexpr double kDt = 0.0005;
  constexpr int kSteps = 120;
  double cost = 0;
  for (int k = 1; k <= kSteps; k++) {
    robot.step(kDt, in);
    if (k % 4 == 0) {
      const double d = track.distanceToLine(robot.sensorBarCenter()) * 1000.0;
      cost += d * d;
    }
  }
  return cost + 0.02 * deviation * deviation;
}

int expertResidual(const RobotModel& robot, const Track& track, const atlas::LineFollower& follower) {
  const atlas::MotorCommand& cmd = follower.lastCommand();
  const int base = (cmd.left + cmd.right) / 2;
  const int pd = follower.pdCorrection();
  int best = 0;
  double bestCost = 1e300;
  for (int d = -kResidualLimit; d <= kResidualLimit; d += 8) {
    const double c = rolloutCost(robot, track, base, pd + d, d);
    if (c < bestCost) {
      bestCost = c;
      best = d;
    }
  }
  return best;
}

std::vector<RobotParams> robotModels() {
  RobotParams nominal, moderate, real;
  moderate.motorTimeConstant = 0.055;
  moderate.yawTimeConstant = 0.020;
  moderate.commandLatencyS = 0.003;
  real.motorTimeConstant = 0.070;
  real.yawTimeConstant = 0.040;
  real.commandLatencyS = 0.004;
  return {nominal, moderate, real};
}

bool required(const std::string& n) { return n != "Trébol de lazos" && n != "Triángulo de rizos"; }

std::vector<RobotCase> makeCases(const std::vector<TrackEntry>& lib, const std::vector<double>& grips, int seeds,
                                 uint32_t seedBase) {
  std::vector<RobotCase> cases;
  const auto models = robotModels();
  int index = 0;
  for (std::size_t t = 0; t < lib.size(); t++) {
    if (!required(lib[t].track.name())) continue;
    for (std::size_t m = 0; m < models.size(); m++)
      for (double mu : grips)
        for (int s = 0; s < seeds; s++) {
          RobotCase c;
          c.track = t;
          c.robot = models[m];
          c.robot.tireFriction = mu;
          c.seed = seedBase + static_cast<uint32_t>(t * 31 + m * 7 + s * 7919 + std::lround(mu * 100));
          c.validation = index++ % 5 == 4;
          cases.push_back(c);
        }
  }
  return cases;
}

struct Outcome {
  std::vector<ScenarioResult> results;
  std::vector<Sample> samples;
};

Outcome runCase(const std::vector<TrackEntry>& lib, const RobotCase& c, const atlas::TinyMlpWeights* student,
                bool collect) {
  atlas::RobotConfig safe = atlas::firmwareSafeConfig();
  atlas::RobotConfig fast = atlas::firmwareFastConfig();
  for (atlas::RobotConfig* cfg : {&safe, &fast}) {
    cfg->control.residualNet = student;
    cfg->control.residualLimit = kResidualLimit;
  }
  ScenarioRunner runner(c.robot, SensorModel(), safe, fast);
  Outcome out;
  uint32_t lastCycle = 0;
  if (collect) {
    const Track& track = lib[c.track].track;
    runner.setObserver([&](double, const atlas::LineSensorArray&, const atlas::LineFollower& f, const RobotModel& r) {
      if (!f.isRunning() || f.isStopping() || !f.historyReady() || !f.lastReading().isTrackable()) return;
      const uint32_t cycle = f.controlCycles();
      if (cycle == lastCycle || cycle % 6 != 0) return;
      lastCycle = cycle;
      Sample s;
      for (int i = 0; i < kInputs; i++) s.input[i] = f.residualInputs()[i];
      s.target = static_cast<float>(expertResidual(r, track, f)) / kResidualLimit;
      s.validation = c.validation;
      out.samples.push_back(s);
    });
  }
  ScenarioSpec spec;
  spec.trackIndex = c.track;
  spec.cruisePwm = safe.speed.basePwm;
  spec.seed = c.seed;
  spec.impellerPwm = 0;
  spec.fastImpellerPwm = 0;
  spec.learnTrack = true;
  spec.attempts = 3;
  out.results = runner.runAttempts(lib[c.track], spec);
  return out;
}

std::vector<Outcome> runAll(const std::vector<TrackEntry>& lib, const std::vector<RobotCase>& cases,
                            const atlas::TinyMlpWeights* student, bool collect) {
  std::vector<Outcome> outs(cases.size());
  std::atomic<std::size_t> next{0};
  std::vector<std::thread> pool;
  for (unsigned k = 0; k < std::max(2u, std::thread::hardware_concurrency()); k++)
    pool.emplace_back([&] {
      for (std::size_t i; (i = next++) < cases.size();) outs[i] = runCase(lib, cases[i], student, collect);
    });
  for (auto& t : pool) t.join();
  return outs;
}

struct Score {
  int failures = 0;
  int runs = 0;
  double mapTime = 0;
  double reconTime = 0;
  double error = 0;
  double zigzag = 0;
};

Score score(const std::vector<Outcome>& outs) {
  Score s;
  int nMap = 0, nRecon = 0, nOk = 0;
  for (const Outcome& o : outs)
    for (std::size_t a = 0; a < o.results.size(); a++) {
      const ScenarioResult& r = o.results[a];
      s.runs++;
      const bool ok = r.passed() && r.raceTime < 40.0;
      if (!ok) {
        s.failures++;
        continue;
      }
      if (a == 0) {
        s.reconTime += r.raceTime;
        nRecon++;
      } else {
        s.mapTime += r.raceTime;
        nMap++;
      }
      s.error += r.rmsOffsetMm;
      s.zigzag += r.zigzagPerMeter;
      nOk++;
    }
  if (nMap) s.mapTime /= nMap;
  if (nRecon) s.reconTime /= nRecon;
  if (nOk) {
    s.error /= nOk;
    s.zigzag /= nOk;
  }
  return s;
}

FloatNet train(const std::vector<Sample>& data, int epochs, double* validationRms) {
  std::mt19937 random(11);
  std::normal_distribution<double> init(0.0, 1.0);
  FloatNet net;
  for (int j = 0; j < kHidden; j++) {
    for (int i = 0; i < kInputs; i++) net.w1[j][i] = init(random) * std::sqrt(2.0 / kInputs);
    net.b1[j] = 0.05;
    net.w2[j] = init(random) * 0.05;
  }
  net.b2 = 0;
  std::vector<std::size_t> order, check;
  for (std::size_t i = 0; i < data.size(); i++) (data[i].validation ? check : order).push_back(i);
  constexpr int kParams = kHidden * kInputs + kHidden + kHidden + 1;
  std::vector<double> m(kParams, 0), v(kParams, 0), g(kParams);
  long step = 0;
  FloatNet best = net;
  double bestRms = 1e9;
  auto rmsOf = [&](const FloatNet& n) {
    double sum = 0;
    double h[kHidden];
    for (std::size_t i : check) {
      const double e = n.forward(data[i].input, h) - data[i].target;
      sum += e * e;
    }
    return std::sqrt(sum / std::max<std::size_t>(1, check.size()));
  };
  for (int epoch = 0; epoch < epochs; epoch++) {
    std::shuffle(order.begin(), order.end(), random);
    for (std::size_t first = 0; first < order.size(); first += 128) {
      std::fill(g.begin(), g.end(), 0.0);
      const std::size_t last = std::min(order.size(), first + 128);
      for (std::size_t b = first; b < last; b++) {
        const Sample& s = data[order[b]];
        double h[kHidden];
        const double e = net.forward(s.input, h) - s.target;
        for (int j = 0; j < kHidden; j++) {
          g[kHidden * kInputs + kHidden + j] += e * h[j];
          if (h[j] <= 0) continue;
          const double back = e * net.w2[j];
          for (int i = 0; i < kInputs; i++) g[j * kInputs + i] += back * (s.input[i] / 127.0);
          g[kHidden * kInputs + j] += back;
        }
        g[kParams - 1] += e;
      }
      const double n = static_cast<double>(last - first);
      step++;
      int p = 0;
      auto adam = [&](double& w) {
        const double grad = g[p] / n;
        m[p] = 0.9 * m[p] + 0.1 * grad;
        v[p] = 0.999 * v[p] + 0.001 * grad * grad;
        const double mh = m[p] / (1 - std::pow(0.9, step));
        const double vh = v[p] / (1 - std::pow(0.999, step));
        w -= 2e-3 * mh / (std::sqrt(vh) + 1e-8);
        p++;
      };
      for (int j = 0; j < kHidden; j++)
        for (int i = 0; i < kInputs; i++) adam(net.w1[j][i]);
      for (int j = 0; j < kHidden; j++) adam(net.b1[j]);
      for (int j = 0; j < kHidden; j++) adam(net.w2[j]);
      adam(net.b2);
    }
    const double r = rmsOf(net);
    if (r < bestRms) {
      bestRms = r;
      best = net;
    }
  }
  *validationRms = bestRms;
  return best;
}

template <typename T>
std::string join(const std::vector<T>& v) {
  std::string s;
  for (std::size_t i = 0; i < v.size(); i++) s += (i ? ", " : "") + std::to_string(static_cast<long>(v[i]));
  return s;
}

void writeHeader(const std::string& path, const QuantNet& q) {
  std::ofstream f(path);
  f << "#pragma once\n\n#include <stdint.h>\n\n#include \"Progmem.h\"\n#include \"TinyMlp.h\"\n\n"
    << "namespace atlas {\nnamespace neural {\n\n"
    << "constexpr int16_t kResidualLimit = " << kResidualLimit << ";\n\n"
    << "ATLAS_PROGMEM const int8_t kResidualWeights1[" << kHidden * kInputs << "] = {" << join(q.w1) << "};\n"
    << "ATLAS_PROGMEM const int32_t kResidualBias1[" << kHidden << "] = {" << join(q.b1) << "};\n"
    << "ATLAS_PROGMEM const int16_t kResidualWeights2[" << kHidden << "] = {" << join(q.w2) << "};\n"
    << "ATLAS_PROGMEM const int32_t kResidualBias2[1] = {" << join(q.b2) << "};\n\n"
    << "constexpr TinyMlpWeights kResidualNet = {" << kInputs << ", " << kHidden << ", 1, kResidualWeights1, "
    << "kResidualBias1, " << int(q.shift1) << ", kResidualWeights2, kResidualBias2, " << int(q.shift2) << "};\n\n"
    << "}\n}\n";
}

void printScore(const char* label, const Score& s) {
  std::printf("  %-28s fallas %3d/%-4d  reconoc. %6.2f s  con mapa %6.2f s  error %5.2f mm  zigzag %.2f/m\n", label,
              s.failures, s.runs, s.reconTime, s.mapTime, s.error, s.zigzag);
  std::fflush(stdout);
}

}

int main(int argc, char** argv) {
  const int iterations = argc > 1 ? std::atoi(argv[1]) : 4;
  const int samplesWanted = argc > 2 ? std::atoi(argv[2]) : 50000;
  const auto lib = buildTrackLibrary();
  const auto trainCases = makeCases(lib, {1.0, 0.8}, 2, 3000);
  const auto evalCases = makeCases(lib, {1.0, 0.8, 0.7}, 3, 9000);
  std::printf("DAgger: experto MPC (21 correcciones, horizonte 60 ms) -> red residual %d-%d-1 (±%d PWM)\n", kInputs,
              kHidden, kResidualLimit);
  std::printf("Casos de entrenamiento %zu, de evaluación %zu (semillas distintas)\n\n", trainCases.size(),
              evalCases.size());

  const QuantNet zero = zeroNet();
  const atlas::TinyMlpWeights zeroWeights = zero.weights();
  const Score baseline = score(runAll(lib, evalCases, &zeroWeights, false));
  printScore("PD solo (referencia)", baseline);

  std::vector<Sample> dataset;
  QuantNet student = zero;
  QuantNet bestNet = zero;
  Score bestScore = baseline;
  int bestIteration = -1;
  for (int it = 0; it < iterations; it++) {
    const atlas::TinyMlpWeights w = student.weights();
    const auto outs = runAll(lib, trainCases, &w, true);
    std::size_t added = 0;
    for (const Outcome& o : outs) {
      dataset.insert(dataset.end(), o.samples.begin(), o.samples.end());
      added += o.samples.size();
    }
    std::vector<Sample> subset = dataset;
    std::mt19937 shuffle(it + 1);
    std::shuffle(subset.begin(), subset.end(), shuffle);
    if (static_cast<int>(subset.size()) > samplesWanted) subset.resize(samplesWanted);
    double rms = 0;
    const FloatNet net = train(subset, 60, &rms);
    student = quantize(net);
    const atlas::TinyMlpWeights trained = student.weights();
    const Score s = score(runAll(lib, evalCases, &trained, false));
    char label[64];
    std::snprintf(label, sizeof label, "DAgger iteración %d", it + 1);
    std::printf("  [+%zu etiquetas del experto, total %zu, entrenadas %zu, RMS validación %.3f]\n", added,
                dataset.size(), subset.size(), rms * kResidualLimit);
    printScore(label, s);
    const bool better = s.failures < bestScore.failures ||
                        (s.failures == bestScore.failures && s.mapTime + s.reconTime < bestScore.mapTime + bestScore.reconTime - 0.05);
    if (better) {
      bestScore = s;
      bestNet = student;
      bestIteration = it + 1;
    }
  }
  if (bestIteration < 0) {
    std::printf("\nNinguna iteración superó al PD solo: no se escriben pesos.\n");
    return 0;
  }
  const std::string path =
      std::string(ATLAS_SOURCE_DIR) + "/firmware/bibliotecas/AtlasCore/src/red_neuronal/NeuralResidualWeights.h";
  writeHeader(path, bestNet);
  std::printf("\nMejor: iteración %d. Pesos escritos en %s\n", bestIteration, path.c_str());
  return 0;
}
