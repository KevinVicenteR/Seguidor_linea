#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <map>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "FirmwareConfig.h"
#include "NeuralHierarchy.h"
#include "ScenarioRunner.h"
#include "TrackLibrary.h"

using namespace sim;

namespace {

constexpr int kIn = atlas::kHierarchyInputs;
constexpr int kSit = atlas::kSituations;
constexpr int kGateHidden = 6;
constexpr int kExpertHidden = 4;
const char* const kSituationNames[kSit] = {"recta", "curva", "cruce/hueco", "recuperación"};

std::string jsonEscape(const std::string& s) {
  std::string out;
  for (char c : s) {
    if (c == '"' || c == '\\') out += '\\';
    out += c;
  }
  return out;
}

struct Log {
  std::vector<std::string> gateEpochs, generations, notes;
  std::string finalTable, baselineGains, trainedGains, situationUse;
  std::mutex lock;
  void note(const std::string& text) {
    std::printf("  * %s\n", text.c_str());
    std::fflush(stdout);
    notes.push_back("\"" + jsonEscape(text) + "\"");
  }
  void write(const std::string& path) const {
    std::ofstream out(path);
    auto list = [&](const std::vector<std::string>& v) {
      std::string s = "[";
      for (std::size_t i = 0; i < v.size(); i++) s += (i ? ",\n  " : "\n  ") + v[i];
      return s + "\n]";
    };
    out << "{\n\"gateEpochs\": " << list(gateEpochs) << ",\n\"generations\": " << list(generations)
        << ",\n\"notes\": " << list(notes) << ",\n\"baselineGains\": " << (baselineGains.empty() ? "null" : baselineGains)
        << ",\n\"trainedGains\": " << (trainedGains.empty() ? "null" : trainedGains)
        << ",\n\"situationUse\": " << (situationUse.empty() ? "null" : situationUse)
        << ",\n\"finalTable\": " << (finalTable.empty() ? "[]" : finalTable) << "\n}\n";
  }
};

struct Tracks {
  std::vector<TrackEntry> all;
  std::vector<bool> extreme;
  std::vector<bool> hard;
};

Tracks loadTracks() {
  Tracks t;
  for (TrackEntry& e : buildTrackLibrary()) {
    const std::string& n = e.track.name();
    if (n == "Trébol de lazos" || n == "Triángulo de rizos") continue;
    const bool hard = t.all.size() <= 1 || n == "Gran Premio" || n.find("Hexágono") != std::string::npos;
    t.all.push_back(std::move(e));
    t.extreme.push_back(false);
    t.hard.push_back(hard);
  }
  for (TrackEntry& e : buildExtremeTrackLibrary()) {
    t.all.push_back(std::move(e));
    t.extreme.push_back(true);
    t.hard.push_back(true);
  }
  return t;
}

struct Case {
  std::size_t track;
  int model;
  uint32_t seed;
};

const char* const kModelNames[3] = {"nominal", "moderado", "real"};

void physicalVariation(const Case& c, RobotParams* robot, SensorModel* sensors) {
  std::mt19937 random(c.seed * 2654435761u + 17);
  auto uniform = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(random); };
  if (c.model == 1) {
    robot->motorTimeConstant = 0.055;
    robot->yawTimeConstant = 0.02;
    robot->commandLatencyS = 0.003;
  } else if (c.model == 2) {
    robot->motorTimeConstant = 0.07;
    robot->yawTimeConstant = 0.04;
    robot->commandLatencyS = 0.004;
  }
  robot->motorTimeConstant *= uniform(0.85, 1.2);
  robot->yawTimeConstant += uniform(0.0, 0.01);
  robot->commandLatencyS += uniform(0.0, 0.0015);
  robot->rightMotorGain = uniform(0.96, 1.04);
  robot->tireFriction = uniform(0.85, 1.1);
  sensors->spotRadius = uniform(0.003, 0.0055);
  sensors->gainSpread = uniform(0.05, 0.18);
  sensors->contrast = uniform(0.8, 1.0);
  sensors->noiseSigma = uniform(2.0, 8.0);
  sensors->ambientOffset = uniform(0.0, 12.0);
}

struct Gains {
  double safeKp, safeKd, fastKp, fastKd;
};

struct Outcome {
  int failures = 0;
  double cost = 0;
  double fastTime = 0, zigzag10 = 0, steering = 0;
  int fastLaps = 0, laps = 0;
};

double visibleZigzag(const ScenarioResult& r) {
  int side = 0, crossings = 0;
  for (const TelemetrySample& s : r.telemetry) {
    const int now = s.lateralOffset > 0.010 ? 1 : (s.lateralOffset < -0.010 ? -1 : 0);
    if (now != 0 && side != 0 && now != side) crossings++;
    if (now != 0) side = now;
  }
  return r.distance > 0 ? crossings / r.distance : 0;
}

Outcome runCase(const Tracks& tracks, const Case& c, const Gains& g, const atlas::NeuralHierarchy* net) {
  atlas::RobotConfig safe = atlas::firmwareSafeConfig(true), fast = atlas::firmwareFastConfig(true);
  safe.control.kp = static_cast<float>(g.safeKp);
  safe.control.kd = static_cast<float>(g.safeKd);
  fast.control.kp = static_cast<float>(g.fastKp);
  fast.control.kd = static_cast<float>(g.fastKd);
  safe.control.hierarchy = fast.control.hierarchy = net;
  if (net == nullptr) safe.control.neuralGains = fast.control.neuralGains = false;
  RobotParams robot;
  SensorModel sensors;
  physicalVariation(c, &robot, &sensors);
  ScenarioSpec spec;
  spec.trackIndex = c.track;
  spec.cruisePwm = safe.speed.basePwm;
  spec.seed = c.seed;
  spec.learnTrack = true;
  spec.attempts = 3;
  spec.impellerPwm = safe.impeller.pwm;
  spec.fastImpellerPwm = fast.impeller.pwm;
  spec.sensorNoiseSigma = sensors.noiseSigma;
  const std::vector<ScenarioResult> attempts =
      ScenarioRunner(robot, sensors, safe, fast).runAttempts(tracks.all[c.track], spec);
  Outcome o;
  const double weight = c.model == 2 ? 1.5 : 1.0;
  for (std::size_t a = 0; a < attempts.size(); a++) {
    const ScenarioResult& r = attempts[a];
    if (!r.passed()) {
      o.failures++;
      o.cost += 300.0 * weight;
      continue;
    }
    const double zz = visibleZigzag(r);
    o.cost += weight * (r.raceTime + 3.0 * zz + 0.05 * r.steeringRmsPwm);
    o.zigzag10 += zz;
    o.steering += r.steeringRmsPwm;
    o.laps++;
    if (a > 0) {
      o.fastTime += r.raceTime;
      o.fastLaps++;
    }
  }
  return o;
}

std::vector<std::vector<Outcome>> runMany(const Tracks& tracks, const std::vector<Case>& cases,
                                          const std::vector<Gains>& gains,
                                          const std::vector<const atlas::NeuralHierarchy*>& nets) {
  std::vector<std::vector<Outcome>> out(nets.size(), std::vector<Outcome>(cases.size()));
  std::atomic<std::size_t> next{0};
  const std::size_t jobs = nets.size() * cases.size();
  auto worker = [&] {
    for (std::size_t j = next++; j < jobs; j = next++) {
      const std::size_t i = j / cases.size(), k = j % cases.size();
      out[i][k] = runCase(tracks, cases[k], gains[i], nets[i]);
    }
  };
  std::vector<std::thread> pool;
  for (unsigned t = 0; t < std::max(2u, std::thread::hardware_concurrency()); t++) pool.emplace_back(worker);
  for (std::thread& t : pool) t.join();
  return out;
}

double totalCost(const std::vector<Outcome>& v) {
  double s = 0;
  for (const Outcome& o : v) s += o.cost;
  return s;
}

struct SeedSource {
  uint32_t next;
  explicit SeedSource(uint32_t base) : next(base) {}
  uint32_t take() { return next++; }
};

std::vector<Case> sampleCases(const Tracks& tracks, int phase, int count, SeedSource& seeds, std::mt19937& random) {
  std::vector<std::size_t> pool;
  for (std::size_t t = 0; t < tracks.all.size(); t++) {
    if (tracks.extreme[t] && phase < 2) continue;
    pool.push_back(t);
    if (phase >= 1 && tracks.hard[t]) pool.push_back(t);
  }
  std::vector<Case> cases;
  for (int i = 0; i < count; i++) {
    cases.push_back({pool[std::uniform_int_distribution<std::size_t>(0, pool.size() - 1)(random)], i % 3, seeds.take()});
  }
  return cases;
}

std::vector<Case> fixedCases(const Tracks& tracks, bool withExtreme, int seedsPerTrackModel, SeedSource& seeds) {
  std::vector<Case> cases;
  for (std::size_t t = 0; t < tracks.all.size(); t++) {
    if (tracks.extreme[t] && !withExtreme) continue;
    for (int m = 0; m < 3; m++)
      for (int k = 0; k < seedsPerTrackModel; k++) cases.push_back({t, m, seeds.take()});
  }
  return cases;
}

struct Dense2 {
  int in, hid, out;
  std::vector<double> w1, b1, w2, b2;
  Dense2(int i, int h, int o) : in(i), hid(h), out(o), w1(i * h), b1(h), w2(h * o), b2(o) {}
  void forward(const int8_t* x, double* hidden, double* y) const {
    for (int j = 0; j < hid; j++) {
      double s = b1[j];
      for (int i = 0; i < in; i++) s += w1[j * in + i] * (x[i] / 127.0);
      hidden[j] = s > 0 ? s : 0;
    }
    for (int k = 0; k < out; k++) {
      double s = b2[k];
      for (int j = 0; j < hid; j++) s += w2[k * hid + j] * hidden[j];
      y[k] = s;
    }
  }
};

struct QuantNet {
  std::vector<int8_t> w1;
  std::vector<int32_t> b1;
  std::vector<int16_t> w2;
  std::vector<int32_t> b2;
  uint8_t shift1 = 7, shift2 = 8;
  int in = 0, hid = 0, out = 0;
  atlas::TinyMlpWeights view() const {
    return {static_cast<uint8_t>(in), static_cast<uint8_t>(hid), static_cast<uint8_t>(out), w1.data(), b1.data(),
            shift1, w2.data(),          b2.data(),           shift2};
  }
};

QuantNet quantize(const Dense2& n, double maxHidden) {
  QuantNet q;
  q.in = n.in;
  q.hid = n.hid;
  q.out = n.out;
  double maxW1 = 1e-9;
  for (double w : n.w1) maxW1 = std::max(maxW1, std::abs(w));
  const double sw1 = 127.0 / maxW1, acc = sw1 * 127.0;
  for (double w : n.w1) q.w1.push_back(static_cast<int8_t>(std::lround(w * sw1)));
  for (double b : n.b1) q.b1.push_back(static_cast<int32_t>(std::lround(b * acc)));
  q.shift1 = 0;
  while (maxHidden * acc / std::pow(2.0, q.shift1) > 12000.0) q.shift1++;
  const double hs = acc / std::pow(2.0, q.shift1);
  double maxW2 = 1e-9;
  for (double w : n.w2) maxW2 = std::max(maxW2, std::abs(w));
  q.shift2 = 0;
  for (int s = 0; s <= 20; s++) {
    const double f = std::pow(2.0, s) / hs;
    if (maxW2 * f > 32000.0 || maxW2 * f * 16000.0 * n.hid > 1.0e9) break;
    q.shift2 = static_cast<uint8_t>(s);
  }
  const double f = std::pow(2.0, q.shift2) / hs;
  for (double w : n.w2) q.w2.push_back(static_cast<int16_t>(std::lround(w * f)));
  for (double b : n.b2) q.b2.push_back(static_cast<int32_t>(std::lround(b * std::pow(2.0, q.shift2))));
  return q;
}

struct GateSample {
  int8_t x[kIn];
  int label;
  bool validation;
};

double trackCurvature(const Track& track, const Vec2& p) {
  const double s = track.nearestArcLength(p);
  const double h1 = track.poseAt(s - 0.03).heading, h2 = track.poseAt(s + 0.03).heading;
  double d = h2 - h1;
  while (d > M_PI) d -= 2 * M_PI;
  while (d < -M_PI) d += 2 * M_PI;
  return std::abs(d) / 0.06;
}

int countCrossings(const Track& track, const RobotModel& robot) {
  const Vec2 left = robot.sensorPosition(0, atlas::kSensorCount);
  const Vec2 right = robot.sensorPosition(atlas::kSensorCount - 1, atlas::kSensorCount);
  const Vec2 axis = (right - left) * (1.0 / (atlas::kSensorCount - 1));
  int count = 0;
  double previous = 1e9, current = 1e9;
  for (double i = -2.0; i <= atlas::kSensorCount + 1.0; i += 0.05) {
    const double next = track.distanceToLine(left + axis * i);
    if (current < previous && current <= next && current < 0.0015) count++;
    previous = current;
    current = next;
  }
  return count;
}

std::vector<GateSample> gateData(const Tracks& tracks) {
  std::vector<Case> runs;
  SeedSource seeds(5000000);
  for (std::size_t t = 0; t < tracks.all.size(); t++)
    for (int m = 0; m < 3; m++)
      for (int k = 0; k < 3; k++) runs.push_back({t, m, seeds.take()});
  std::vector<GateSample> data;
  std::mutex lock;
  std::atomic<std::size_t> next{0};
  auto worker = [&] {
    for (std::size_t r = next++; r < runs.size(); r = next++) {
      const Case& c = runs[r];
      atlas::RobotConfig safe = atlas::firmwareSafeConfig(true), fast = atlas::firmwareFastConfig(true);
      RobotParams robot;
      SensorModel sensors;
      physicalVariation(c, &robot, &sensors);
      ScenarioRunner runner(robot, sensors, safe, fast);
      std::vector<GateSample> local;
      int tick = 0;
      const bool validation = (c.track % 4) == 3;
      runner.setObserver([&](double, const atlas::LineSensorArray&, const atlas::LineFollower& f, const RobotModel& rb) {
        if (!f.isRunning() || f.isRecovering() || f.isBridging() || (tick++ % 200) != 0) return;
        GateSample s;
        std::copy(f.neuralInputs(), f.neuralInputs() + kIn, s.x);
        const Track& track = tracks.all[c.track].track;
        if (s.x[7] < 75) {
          s.label = 3;
        } else if (s.x[6] != 0 || countCrossings(track, rb) >= 2) {
          s.label = 2;
        } else if (trackCurvature(track, rb.sensorBarCenter()) > 1.0 / 0.6) {
          s.label = 1;
        } else {
          s.label = 0;
        }
        s.validation = validation;
        local.push_back(s);
      });
      ScenarioSpec spec;
      spec.trackIndex = c.track;
      spec.cruisePwm = safe.speed.basePwm;
      spec.seed = c.seed;
      spec.learnTrack = true;
      spec.attempts = 2;
      spec.impellerPwm = safe.impeller.pwm;
      spec.fastImpellerPwm = fast.impeller.pwm;
      spec.sensorNoiseSigma = sensors.noiseSigma;
      runner.runAttempts(tracks.all[c.track], spec);
      std::lock_guard<std::mutex> g(lock);
      data.insert(data.end(), local.begin(), local.end());
    }
  };
  std::vector<std::thread> pool;
  for (unsigned t = 0; t < std::max(2u, std::thread::hardware_concurrency()); t++) pool.emplace_back(worker);
  for (std::thread& t : pool) t.join();
  return data;
}

QuantNet trainGate(const std::vector<GateSample>& data, Log& log) {
  std::mt19937 random(3);
  Dense2 net(kIn, kGateHidden, kSit);
  std::normal_distribution<double> init(0, 1);
  for (double& w : net.w1) w = init(random) * std::sqrt(2.0 / kIn);
  for (double& b : net.b1) b = 0.1;
  for (double& w : net.w2) w = init(random) * 20.0;
  for (double& b : net.b2) b = 64.0;

  int count[kSit] = {};
  std::vector<std::size_t> train, val;
  for (std::size_t i = 0; i < data.size(); i++) {
    (data[i].validation ? val : train).push_back(i);
    if (!data[i].validation) count[data[i].label]++;
  }
  double classWeight[kSit];
  for (int k = 0; k < kSit; k++) classWeight[k] = train.size() / (kSit * std::max(1, count[k]) * 1.0);
  std::printf("Etapa 1: %zu muestras de entrenamiento, %zu de validación (pistas separadas)\n", train.size(), val.size());
  for (int k = 0; k < kSit; k++) std::printf("  %-13s %6d muestras  peso %.2f\n", kSituationNames[k], count[k], classWeight[k]);

  const int params = static_cast<int>(net.w1.size() + net.b1.size() + net.w2.size() + net.b2.size());
  std::vector<double> m(params, 0), v(params, 0), grad(params);
  auto param = [&](int k) -> double& {
    if (k < static_cast<int>(net.w1.size())) return net.w1[k];
    k -= static_cast<int>(net.w1.size());
    if (k < static_cast<int>(net.b1.size())) return net.b1[k];
    k -= static_cast<int>(net.b1.size());
    if (k < static_cast<int>(net.w2.size())) return net.w2[k];
    return net.b2[k - net.w2.size()];
  };
  auto evaluate = [&](const std::vector<std::size_t>& idx, double* loss, double* accuracy) {
    double l = 0, wsum = 0;
    int ok = 0;
    for (std::size_t i : idx) {
      double h[kGateHidden], y[kSit];
      net.forward(data[i].x, h, y);
      int best = 0;
      for (int k = 0; k < kSit; k++) {
        const double target = k == data[i].label ? 255.0 : 0.0;
        const double e = (std::max(0.0, y[k]) - target) / 255.0;
        l += classWeight[data[i].label] * e * e;
        if (std::max(0.0, y[k]) > std::max(0.0, y[best])) best = k;
      }
      wsum += classWeight[data[i].label];
      ok += best == data[i].label;
    }
    *loss = l / std::max(1e-9, wsum);
    *accuracy = idx.empty() ? 0 : 100.0 * ok / idx.size();
  };

  double rate = 0.01, bestVal = 1e9;
  const double l2 = 1e-4;
  Dense2 best = net;
  int sinceBest = 0, sinceDrop = 0, bestEpoch = 0;
  long step = 0;
  for (int epoch = 0; epoch < 400; epoch++) {
    std::shuffle(train.begin(), train.end(), random);
    for (std::size_t first = 0; first < train.size(); first += 128) {
      std::fill(grad.begin(), grad.end(), 0.0);
      const std::size_t last = std::min(train.size(), first + 128);
      for (std::size_t b = first; b < last; b++) {
        const GateSample& s = data[train[b]];
        double h[kGateHidden], y[kSit];
        net.forward(s.x, h, y);
        double dy[kSit];
        for (int k = 0; k < kSit; k++) {
          const double target = k == s.label ? 255.0 : 0.0;
          dy[k] = y[k] > 0 || target > 0 ? classWeight[s.label] * 2.0 * (std::max(0.0, y[k]) - target) / (255.0 * 255.0) : 0.0;
          if (y[k] <= 0 && target == 0) dy[k] = 0;
        }
        const int w2off = static_cast<int>(net.w1.size() + net.b1.size());
        for (int k = 0; k < kSit; k++) {
          for (int j = 0; j < kGateHidden; j++) grad[w2off + k * kGateHidden + j] += dy[k] * h[j];
          grad[w2off + net.w2.size() + k] += dy[k];
        }
        for (int j = 0; j < kGateHidden; j++) {
          if (h[j] <= 0) continue;
          double back = 0;
          for (int k = 0; k < kSit; k++) back += dy[k] * net.w2[k * kGateHidden + j];
          for (int i = 0; i < kIn; i++) grad[j * kIn + i] += back * (s.x[i] / 127.0);
          grad[net.w1.size() + j] += back;
        }
      }
      step++;
      const double scale = 1.0 / (last - first);
      for (int k = 0; k < params; k++) {
        const double g = grad[k] * scale + l2 * param(k);
        m[k] = 0.9 * m[k] + 0.1 * g;
        v[k] = 0.999 * v[k] + 0.001 * g * g;
        param(k) -= rate * (m[k] / (1 - std::pow(0.9, step))) / (std::sqrt(v[k] / (1 - std::pow(0.999, step))) + 1e-8);
      }
    }
    double trainLoss, trainAcc, valLoss, valAcc;
    evaluate(train, &trainLoss, &trainAcc);
    evaluate(val, &valLoss, &valAcc);
    std::string action = "-";
    if (valLoss < bestVal * 0.999) {
      bestVal = valLoss;
      best = net;
      bestEpoch = epoch;
      sinceBest = sinceDrop = 0;
      action = "nuevo mejor en validación (se guarda)";
    } else {
      sinceBest++;
      if (++sinceDrop >= 15 && rate > 1e-4) {
        rate *= 0.5;
        sinceDrop = 0;
        action = "meseta: paso de aprendizaje a la mitad";
      }
    }
    if (sinceBest >= 40) action = "parada temprana: 40 épocas sin mejorar";
    char row[512];
    std::snprintf(row, sizeof(row),
                  "{\"epoch\":%d,\"trainLoss\":%.5f,\"valLoss\":%.5f,\"trainAcc\":%.2f,\"valAcc\":%.2f,\"rate\":%.6f,\"action\":\"%s\"}",
                  epoch, trainLoss, valLoss, trainAcc, valAcc, rate, jsonEscape(action).c_str());
    log.gateEpochs.push_back(row);
    if (epoch % 10 == 0 || action != "-")
      std::printf("  época %3d  pérdida %.4f / %.4f  exactitud %.1f%% / %.1f%%  paso %.4f  %s\n", epoch, trainLoss, valLoss,
                  trainAcc, valAcc, rate, action.c_str());
    if (sinceBest >= 40) break;
  }
  std::printf("  mejor época %d\n", bestEpoch);

  double maxHidden = 1e-9;
  for (const GateSample& s : data) {
    double h[kGateHidden], y[kSit];
    best.forward(s.x, h, y);
    for (double x : h) maxHidden = std::max(maxHidden, x);
  }
  QuantNet q = quantize(best, maxHidden);
  const atlas::TinyMlpWeights view = q.view();
  int confusion[kSit][kSit] = {};
  for (std::size_t i : val) {
    int32_t y[kSit];
    atlas::TinyMlp(view).evaluate(data[i].x, y);
    int b = 0;
    for (int k = 0; k < kSit; k++)
      if (y[k] > y[b]) b = k;
    confusion[data[i].label][b]++;
  }
  std::printf("  matriz de confusión (validación, red en enteros; filas = real):\n");
  for (int r = 0; r < kSit; r++) {
    std::printf("    %-13s", kSituationNames[r]);
    for (int c = 0; c < kSit; c++) std::printf(" %6d", confusion[r][c]);
    std::printf("\n");
  }
  std::ostringstream note;
  note << "Etapa 1 terminada: mejor época " << bestEpoch << ", pérdida de validación " << bestVal;
  log.note(note.str());
  return q;
}

class SepCmaEs {
 public:
  SepCmaEs(std::vector<double> mean, double sigma, int lambda, uint32_t seed)
      : n_(static_cast<int>(mean.size())), lambda_(lambda), mean_(std::move(mean)), sigma_(sigma), c_(n_, 1.0),
        ps_(n_, 0.0), pc_(n_, 0.0), random_(seed) {
    mu_ = lambda_ / 2;
    double sum = 0, sum2 = 0;
    for (int i = 0; i < mu_; i++) {
      weights_.push_back(std::log(mu_ + 0.5) - std::log(i + 1.0));
      sum += weights_.back();
    }
    for (double& w : weights_) {
      w /= sum;
      sum2 += w * w;
    }
    mueff_ = 1.0 / sum2;
    cs_ = (mueff_ + 2) / (n_ + mueff_ + 5);
    ds_ = 1 + 2 * std::max(0.0, std::sqrt((mueff_ - 1) / (n_ + 1)) - 1) + cs_;
    cc_ = (4 + mueff_ / n_) / (n_ + 4 + 2 * mueff_ / n_);
    c1_ = 2.0 / ((n_ + 1.3) * (n_ + 1.3) + mueff_) * (n_ + 2) / 3.0;
    cmu_ = std::min(1 - c1_, 2 * (mueff_ - 2 + 1 / mueff_) / ((n_ + 2) * (n_ + 2) + mueff_) * (n_ + 2) / 3.0);
    chiN_ = std::sqrt(n_) * (1 - 1.0 / (4 * n_) + 1.0 / (21.0 * n_ * n_));
  }

  std::vector<std::vector<double>> ask() {
    z_.assign(lambda_, std::vector<double>(n_));
    std::vector<std::vector<double>> xs(lambda_, std::vector<double>(n_));
    std::normal_distribution<double> g(0, 1);
    for (int k = 0; k < lambda_; k++)
      for (int i = 0; i < n_; i++) {
        z_[k][i] = g(random_);
        xs[k][i] = mean_[i] + sigma_ * std::sqrt(c_[i]) * z_[k][i];
      }
    return xs;
  }

  void tell(const std::vector<double>& costs) {
    std::vector<int> order(lambda_);
    for (int i = 0; i < lambda_; i++) order[i] = i;
    std::sort(order.begin(), order.end(), [&](int a, int b) { return costs[a] < costs[b]; });
    std::vector<double> zmean(n_, 0.0);
    for (int i = 0; i < mu_; i++)
      for (int d = 0; d < n_; d++) zmean[d] += weights_[i] * z_[order[i]][d];
    double psNorm = 0;
    for (int d = 0; d < n_; d++) {
      mean_[d] += sigma_ * std::sqrt(c_[d]) * zmean[d];
      ps_[d] = (1 - cs_) * ps_[d] + std::sqrt(cs_ * (2 - cs_) * mueff_) * zmean[d];
      psNorm += ps_[d] * ps_[d];
    }
    psNorm = std::sqrt(psNorm);
    generation_++;
    const bool hsig = psNorm / std::sqrt(1 - std::pow(1 - cs_, 2.0 * generation_)) / chiN_ < 1.4 + 2.0 / (n_ + 1);
    for (int d = 0; d < n_; d++) {
      pc_[d] = (1 - cc_) * pc_[d] + (hsig ? std::sqrt(cc_ * (2 - cc_) * mueff_) * std::sqrt(c_[d]) * zmean[d] : 0.0);
      double rankMu = 0;
      for (int i = 0; i < mu_; i++) rankMu += weights_[i] * c_[d] * z_[order[i]][d] * z_[order[i]][d];
      c_[d] = (1 - c1_ - cmu_) * c_[d] + c1_ * pc_[d] * pc_[d] + cmu_ * rankMu;
    }
    sigma_ *= std::exp((cs_ / ds_) * (psNorm / chiN_ - 1));
    sigma_ = std::min(sigma_, 2.0);
  }

  void restartSigma(double sigma) {
    sigma_ = sigma;
    std::fill(ps_.begin(), ps_.end(), 0.0);
    std::fill(pc_.begin(), pc_.end(), 0.0);
    std::fill(c_.begin(), c_.end(), 1.0);
  }
  const std::vector<double>& mean() const { return mean_; }
  void setMean(const std::vector<double>& m) { mean_ = m; }
  double sigma() const { return sigma_; }

 private:
  int n_, lambda_, mu_;
  std::vector<double> mean_;
  double sigma_;
  std::vector<double> c_, ps_, pc_, weights_;
  double mueff_, cs_, ds_, cc_, c1_, cmu_, chiN_;
  int generation_ = 0;
  std::vector<std::vector<double>> z_;
  std::mt19937 random_;
};

constexpr int kExpertParams = kExpertHidden * kIn + kExpertHidden + 2 * kExpertHidden + 2;
constexpr int kStage2Params = kSit * kExpertParams + 4;

struct Hierarchy {
  QuantNet gate;
  QuantNet experts[kSit];
  atlas::NeuralHierarchy view() const {
    atlas::NeuralHierarchy h{};
    h.gate = gate.view();
    for (int k = 0; k < kSit; k++) h.experts[k] = experts[k].view();
    return h;
  }
};

QuantNet decodeExpert(const double* t) {
  QuantNet q;
  q.in = kIn;
  q.hid = kExpertHidden;
  q.out = 2;
  q.shift1 = 7;
  q.shift2 = 8;
  int k = 0;
  for (int i = 0; i < kExpertHidden * kIn; i++)
    q.w1.push_back(static_cast<int8_t>(std::max(-127L, std::min(127L, std::lround(t[k++] * 24)))));
  for (int i = 0; i < kExpertHidden; i++) q.b1.push_back(static_cast<int32_t>(std::lround(t[k++] * 3000)));
  for (int i = 0; i < 2 * kExpertHidden; i++)
    q.w2.push_back(static_cast<int16_t>(std::max(-32767L, std::min(32767L, std::lround(t[k++] * 24)))));
  for (int i = 0; i < 2; i++) q.b2.push_back((100 << 8) + static_cast<int32_t>(std::lround(t[k++] * 2560)));
  return q;
}

Gains decodeGains(const double* t, const Gains& base) {
  auto scale = [](double theta) { return std::max(0.6, std::min(1.4, 1.0 + 0.2 * theta)); };
  return {base.safeKp * scale(t[0]), base.safeKd * scale(t[1]), base.fastKp * scale(t[2]), base.fastKd * scale(t[3])};
}

bool meetsConstraints(const Tracks& tracks, const std::vector<Case>& cases, const std::vector<Outcome>& candidate,
                      const std::vector<Outcome>& reference, std::string* why) {
  std::map<std::pair<std::size_t, int>, Outcome> c, r;
  for (std::size_t i = 0; i < cases.size(); i++) {
    if (tracks.extreme[cases[i].track]) continue;
    auto key = std::make_pair(cases[i].track, cases[i].model);
    c[key].failures += candidate[i].failures;
    c[key].fastTime += candidate[i].fastTime;
    c[key].fastLaps += candidate[i].fastLaps;
    r[key].failures += reference[i].failures;
    r[key].fastTime += reference[i].fastTime;
    r[key].fastLaps += reference[i].fastLaps;
  }
  for (auto& kv : c) {
    const Outcome& ref = r[kv.first];
    const std::string name = tracks.all[kv.first.first].track.name() + "/" + kModelNames[kv.first.second];
    if (kv.second.failures > ref.failures) {
      *why = "más salidas en " + name;
      return false;
    }
    if (kv.second.fastLaps && ref.fastLaps &&
        kv.second.fastTime / kv.second.fastLaps > 1.03 * ref.fastTime / ref.fastLaps) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), " (%+.1f %%)", 100.0 * ((kv.second.fastTime / kv.second.fastLaps) / (ref.fastTime / ref.fastLaps) - 1));
      *why = "mapa más lento en " + name + buf;
      return false;
    }
  }
  return true;
}

std::string summarize(const std::vector<Outcome>& v) {
  int f = 0, laps = 0, fl = 0;
  double zz = 0, st = 0, ft = 0;
  for (const Outcome& o : v) {
    f += o.failures;
    laps += o.laps;
    zz += o.zigzag10;
    st += o.steering;
    ft += o.fastTime;
    fl += o.fastLaps;
  }
  char buf[200];
  std::snprintf(buf, sizeof(buf), "salidas %d  zigzag±10mm %.3f/m  dirección %.1f  mapa %.2fs", f, laps ? zz / laps : 0,
                laps ? st / laps : 0, fl ? ft / fl : 0);
  return buf;
}

template <typename T>
std::string join(const std::vector<T>& v, std::size_t from, std::size_t count) {
  std::string s;
  for (std::size_t i = 0; i < count; i++) s += (i ? ", " : "") + std::to_string(static_cast<long>(v[from + i]));
  return s;
}

void writeWeights(const std::string& path, const Hierarchy& h) {
  std::ofstream out(path);
  out << "#pragma once\n\n#include <stdint.h>\n\n#include \"NeuralHierarchy.h\"\n#include \"Progmem.h\"\n\n"
      << "namespace atlas {\nnamespace neural {\n\n"
      << "constexpr uint8_t kGateHidden = " << kGateHidden << ";\nconstexpr uint8_t kExpertHidden = " << kExpertHidden << ";\n\n"
      << "ATLAS_PROGMEM const int8_t kGateW1[kGateHidden * kHierarchyInputs] = {" << join(h.gate.w1, 0, h.gate.w1.size()) << "};\n"
      << "ATLAS_PROGMEM const int32_t kGateB1[kGateHidden] = {" << join(h.gate.b1, 0, h.gate.b1.size()) << "};\n"
      << "ATLAS_PROGMEM const int16_t kGateW2[kSituations * kGateHidden] = {" << join(h.gate.w2, 0, h.gate.w2.size()) << "};\n"
      << "ATLAS_PROGMEM const int32_t kGateB2[kSituations] = {" << join(h.gate.b2, 0, h.gate.b2.size()) << "};\n\n";
  auto block = [&](const char* type, const char* name, const char* dims, int which) {
    out << "ATLAS_PROGMEM const " << type << " " << name << "[kSituations][" << dims << "] = {\n";
    for (int k = 0; k < kSit; k++) {
      std::string row;
      if (which == 0) row = join(h.experts[k].w1, 0, h.experts[k].w1.size());
      if (which == 1) row = join(h.experts[k].b1, 0, h.experts[k].b1.size());
      if (which == 2) row = join(h.experts[k].w2, 0, h.experts[k].w2.size());
      if (which == 3) row = join(h.experts[k].b2, 0, h.experts[k].b2.size());
      out << "    {" << row << "}" << (k + 1 < kSit ? "," : "") << "\n";
    }
    out << "};\n";
  };
  block("int8_t", "kExpertW1", "kExpertHidden * kHierarchyInputs", 0);
  block("int32_t", "kExpertB1", "kExpertHidden", 1);
  block("int16_t", "kExpertW2", "2 * kExpertHidden", 2);
  block("int32_t", "kExpertB2", "2", 3);
  out << "\nconst NeuralHierarchy kHierarchy = {\n"
      << "    {kHierarchyInputs, kGateHidden, kSituations, kGateW1, kGateB1, " << int(h.gate.shift1) << ", kGateW2, kGateB2, "
      << int(h.gate.shift2) << "},\n    {";
  for (int k = 0; k < kSit; k++) {
    out << (k ? ",\n     " : "") << "{kHierarchyInputs, kExpertHidden, 2, kExpertW1[" << k << "], kExpertB1[" << k << "], 7, kExpertW2["
        << k << "], kExpertB2[" << k << "], 8}";
  }
  out << "}};\n\n}\n}\n";
}

std::string gainsJson(const Gains& g) {
  char buf[200];
  std::snprintf(buf, sizeof(buf), "{\"safeKp\":%.5f,\"safeKd\":%.4f,\"fastKp\":%.5f,\"fastKd\":%.4f}", g.safeKp, g.safeKd,
                g.fastKp, g.fastKd);
  return buf;
}

}

int main(int argc, char** argv) {
  const int generations = argc > 1 ? std::atoi(argv[1]) : 100;
  const std::string logPath = argc > 2 ? argv[2] : std::string(ATLAS_SOURCE_DIR) + "/entrenamiento/resultados/entrenamiento_jerarquica.json";
  const Tracks tracks = loadTracks();
  Log log;
  const atlas::RobotConfig safeNow = atlas::safeProfile(), fastNow = atlas::fastProfile();
  const Gains base = {safeNow.control.kp, safeNow.control.kd, fastNow.control.kp, fastNow.control.kd};
  log.baselineGains = gainsJson(base);
  std::printf("Pistas: %zu (%zu extremas). Kp/Kd base: seguro %.4f/%.3f, rápido %.4f/%.3f\n", tracks.all.size(),
              static_cast<std::size_t>(std::count(tracks.extreme.begin(), tracks.extreme.end(), true)), base.safeKp,
              base.safeKd, base.fastKp, base.fastKd);

  const std::vector<GateSample> data = gateData(tracks);
  Hierarchy best;
  best.gate = trainGate(data, log);
  log.write(logPath);

  std::mt19937 random(91);
  std::normal_distribution<double> g(0, 1);
  std::vector<double> mean(kStage2Params, 0.0);
  for (int k = 0; k < kSit; k++) {
    double* t = &mean[k * kExpertParams];
    for (int i = 0; i < kExpertHidden * kIn; i++) t[i] = g(random) * 0.7;
    for (int i = 0; i < kExpertHidden; i++) t[kExpertHidden * kIn + i] = 0.2;
  }
  SepCmaEs cma(mean, 0.3, 20, 7);
  SeedSource trainSeeds(10000000), checkSeeds(20000000), testSeeds(30000000);
  const std::vector<Case> checkCases = fixedCases(tracks, true, 1, checkSeeds);

  auto build = [&](const std::vector<double>& theta, Hierarchy* h, Gains* gains) {
    h->gate = best.gate;
    for (int k = 0; k < kSit; k++) h->experts[k] = decodeExpert(&theta[k * kExpertParams]);
    *gains = decodeGains(&theta[kSit * kExpertParams], base);
  };

  const std::vector<Outcome> reference = runMany(tracks, checkCases, {base}, {nullptr})[0];
  double bestCheck = totalCost(reference);
  std::vector<double> bestTheta(kStage2Params, 0.0);
  for (int k = 0; k < kSit; k++) bestTheta[k * kExpertParams + kExpertParams - 1] = 0;
  bool haveBest = false;
  std::printf("\nEtapa 2: referencia en validación: costo %.1f, %s\n", bestCheck, summarize(reference).c_str());
  int checksWithoutGain = 0, restarts = 0, phase = 0;
  long competitions = static_cast<long>(checkCases.size());
  for (int gen = 0; gen < generations; gen++) {
    const int newPhase = gen < generations / 5 ? 0 : (gen < 2 * generations / 5 ? 1 : 2);
    std::string action = "-";
    if (newPhase != phase) {
      phase = newPhase;
      action = phase == 1 ? "currículo: entran las pistas difíciles (doble peso)" : "currículo: entran las pistas extremas";
      log.note("gen " + std::to_string(gen) + ": " + action);
    }
    const std::vector<Case> cases = sampleCases(tracks, phase, 12, trainSeeds, random);
    const std::vector<std::vector<double>> xs = cma.ask();
    std::vector<Hierarchy> nets(xs.size());
    std::vector<atlas::NeuralHierarchy> views(xs.size());
    std::vector<Gains> gains(xs.size());
    std::vector<const atlas::NeuralHierarchy*> pointers;
    for (std::size_t i = 0; i < xs.size(); i++) {
      build(xs[i], &nets[i], &gains[i]);
      views[i] = nets[i].view();
      pointers.push_back(&views[i]);
    }
    const auto outcomes = runMany(tracks, cases, gains, pointers);
    competitions += static_cast<long>(xs.size() * cases.size());
    std::vector<double> costs;
    for (const auto& o : outcomes) costs.push_back(totalCost(o));
    cma.tell(costs);
    std::vector<double> sorted = costs;
    std::sort(sorted.begin(), sorted.end());

    std::string validation = "null";
    if (gen % 10 == 9) {
      Hierarchy h;
      Gains gm;
      build(cma.mean(), &h, &gm);
      const atlas::NeuralHierarchy view = h.view();
      const std::vector<Outcome> check = runMany(tracks, checkCases, {gm}, {&view})[0];
      competitions += static_cast<long>(checkCases.size());
      const double cost = totalCost(check);
      std::string why;
      const bool ok = meetsConstraints(tracks, checkCases, check, reference, &why);
      char buf[160];
      std::snprintf(buf, sizeof(buf), "%.2f", cost);
      validation = buf;
      if (ok && cost < bestCheck - 0.5) {
        bestCheck = cost;
        bestTheta = cma.mean();
        haveBest = true;
        checksWithoutGain = 0;
        action = "validación: nuevo mejor (" + summarize(check) + ")";
      } else {
        action = ok ? "validación: no mejora" : "validación: rechazado, " + why;
        if (++checksWithoutGain >= 3) {
          if (restarts < 2) {
            restarts++;
            checksWithoutGain = 0;
            cma.setMean(haveBest ? bestTheta : cma.mean());
            cma.restartSigma(0.2);
            action += "; estancado: reinicio de σ a 0.2 desde el mejor";
          } else {
            action += "; estancado tras 2 reinicios: se detiene";
          }
        }
      }
      log.note("gen " + std::to_string(gen) + ": " + action);
    }
    char row[700];
    std::snprintf(row, sizeof(row),
                  "{\"gen\":%d,\"phase\":%d,\"best\":%.2f,\"median\":%.2f,\"worst\":%.2f,\"sigma\":%.4f,\"validation\":%s,"
                  "\"competitions\":%ld,\"action\":\"%s\"}",
                  gen, phase, sorted.front(), sorted[sorted.size() / 2], sorted.back(), cma.sigma(), validation.c_str(),
                  competitions, jsonEscape(action).c_str());
    log.generations.push_back(row);
    std::printf("gen %3d fase %d  mejor %.1f  mediana %.1f  σ %.3f  competencias %ld  %s\n", gen, phase, sorted.front(),
                sorted[sorted.size() / 2], cma.sigma(), competitions, action == "-" ? "" : action.c_str());
    std::fflush(stdout);
    log.write(logPath);
    if (action.find("se detiene") != std::string::npos) break;
  }

  Hierarchy trained;
  Gains trainedGains;
  build(bestTheta, &trained, &trainedGains);
  if (haveBest) {
    std::vector<double> gateOut;
    for (int16_t w : trained.gate.w2) gateOut.push_back(w);
    for (int32_t b : trained.gate.b2) gateOut.push_back(b);
    const std::vector<double> scale = gateOut;
    SepCmaEs fine(std::vector<double>(gateOut.size(), 0.0), 0.1, 12, 13);
    double fineBest = bestCheck;
    std::vector<double> fineTheta(gateOut.size(), 0.0);
    auto applyFine = [&](const std::vector<double>& t, Hierarchy* h) {
      *h = trained;
      for (std::size_t i = 0; i < h->gate.w2.size(); i++)
        h->gate.w2[i] = static_cast<int16_t>(std::max(-32767.0, std::min(32767.0, scale[i] * (1 + 0.3 * t[i]))));
      for (std::size_t i = 0; i < h->gate.b2.size(); i++)
        h->gate.b2[i] = static_cast<int32_t>(scale[h->gate.w2.size() + i] * (1 + 0.3 * t[h->gate.w2.size() + i]));
    };
    for (int gen = 0; gen < 20; gen++) {
      const std::vector<Case> cases = sampleCases(tracks, 2, 12, trainSeeds, random);
      const auto xs = fine.ask();
      std::vector<Hierarchy> nets(xs.size());
      std::vector<atlas::NeuralHierarchy> views(xs.size());
      std::vector<const atlas::NeuralHierarchy*> ptrs;
      for (std::size_t i = 0; i < xs.size(); i++) {
        applyFine(xs[i], &nets[i]);
        views[i] = nets[i].view();
        ptrs.push_back(&views[i]);
      }
      const auto out = runMany(tracks, cases, std::vector<Gains>(xs.size(), trainedGains), ptrs);
      std::vector<double> costs;
      for (const auto& o : out) costs.push_back(totalCost(o));
      fine.tell(costs);
      if (gen % 5 == 4) {
        Hierarchy h;
        applyFine(fine.mean(), &h);
        const atlas::NeuralHierarchy view = h.view();
        const auto check = runMany(tracks, checkCases, {trainedGains}, {&view})[0];
        std::string why;
        if (meetsConstraints(tracks, checkCases, check, reference, &why) && totalCost(check) < fineBest - 0.5) {
          fineBest = totalCost(check);
          fineTheta = fine.mean();
          log.note("etapa 3 gen " + std::to_string(gen) + ": ajuste conjunto mejora la validación (" + summarize(check) + ")");
        } else {
          log.note("etapa 3 gen " + std::to_string(gen) + ": sin mejora" + (why.empty() ? "" : " (" + why + ")"));
        }
      }
    }
    applyFine(fineTheta, &trained);
  } else {
    log.note("Ninguna red superó a la referencia en validación: se conserva la red neutra.");
  }
  log.trainedGains = gainsJson(trainedGains);

  const std::vector<Case> testCases = fixedCases(tracks, true, 3, testSeeds);
  const atlas::NeuralHierarchy trainedView = trained.view();
  const auto test = runMany(tracks, testCases, {base, trainedGains}, {nullptr, &trainedView});
  std::string why;
  const bool ok = meetsConstraints(tracks, testCases, test[1], test[0], &why);
  std::printf("\nPrueba final (%zu competencias por configuración, semillas nuevas):\n  referencia: %s\n  entrenada:  %s\n  %s\n",
              testCases.size(), summarize(test[0]).c_str(), summarize(test[1]).c_str(),
              ok ? "cumple restricciones por pista y modelo" : ("NO cumple: " + why).c_str());
  std::string table = "[";
  for (std::size_t t = 0; t < tracks.all.size(); t++) {
    for (int m = 0; m < 3; m++) {
      Outcome r, c;
      for (std::size_t i = 0; i < testCases.size(); i++) {
        if (testCases[i].track != t || testCases[i].model != m) continue;
        for (Outcome* o : {&r, &c}) {
          const Outcome& src = o == &r ? test[0][i] : test[1][i];
          o->failures += src.failures;
          o->zigzag10 += src.zigzag10;
          o->steering += src.steering;
          o->fastTime += src.fastTime;
          o->fastLaps += src.fastLaps;
          o->laps += src.laps;
        }
      }
      char row[600];
      std::snprintf(row, sizeof(row),
                    "%s{\"track\":\"%s\",\"extreme\":%s,\"model\":\"%s\",\"refFail\":%d,\"newFail\":%d,\"refZigzag\":%.3f,"
                    "\"newZigzag\":%.3f,\"refSteer\":%.1f,\"newSteer\":%.1f,\"refTime\":%.3f,\"newTime\":%.3f}",
                    table.size() > 1 ? ",\n  " : "\n  ", jsonEscape(tracks.all[t].track.name()).c_str(),
                    tracks.extreme[t] ? "true" : "false", kModelNames[m], r.failures, c.failures,
                    r.laps ? r.zigzag10 / r.laps : 0, c.laps ? c.zigzag10 / c.laps : 0, r.laps ? r.steering / r.laps : 0,
                    c.laps ? c.steering / c.laps : 0, r.fastLaps ? r.fastTime / r.fastLaps : 0,
                    c.fastLaps ? c.fastTime / c.fastLaps : 0);
      table += row;
    }
  }
  log.finalTable = table + "\n]";
  log.note(ok ? "Prueba final: cumple restricciones por pista y modelo" : "Prueba final: NO cumple (" + why + ")");

  {
    double use[kSit] = {};
    long samples = 0;
    for (std::size_t t = 0; t < tracks.all.size(); t++) {
      atlas::RobotConfig safe = atlas::firmwareSafeConfig(true), fast = atlas::firmwareFastConfig(true);
      safe.control.hierarchy = fast.control.hierarchy = &trainedView;
      ScenarioRunner runner(RobotParams(), SensorModel(), safe, fast);
      runner.setObserver([&](double, const atlas::LineSensorArray&, const atlas::LineFollower& f, const RobotModel&) {
        if (!f.isRunning() || f.isRecovering()) return;
        double sum = 0;
        for (int k = 0; k < kSit; k++) sum += f.situationWeights()[k];
        if (sum <= 0) return;
        for (int k = 0; k < kSit; k++) use[k] += f.situationWeights()[k] / sum;
        samples++;
      });
      ScenarioSpec spec;
      spec.trackIndex = t;
      spec.cruisePwm = safe.speed.basePwm;
      spec.seed = 77 + static_cast<uint32_t>(t);
      spec.impellerPwm = safe.impeller.pwm;
      runner.run(tracks.all[t], spec);
    }
    std::string s = "{";
    for (int k = 0; k < kSit; k++) {
      char buf[80];
      std::snprintf(buf, sizeof(buf), "%s\"%s\":%.3f", k ? "," : "", kSituationNames[k], samples ? use[k] / samples : 0);
      s += buf;
    }
    log.situationUse = s + "}";
  }

  if (ok && haveBest) {
    const std::string path = std::string(ATLAS_SOURCE_DIR) + "/firmware/bibliotecas/AtlasCore/src/red_neuronal/NeuralHierarchyWeights.h";
    writeWeights(path, trained);
    std::printf("Pesos escritos en %s\nKp/Kd entrenados: seguro %.5f/%.4f  rápido %.5f/%.4f\n", path.c_str(),
                trainedGains.safeKp, trainedGains.safeKd, trainedGains.fastKp, trainedGains.fastKd);
  } else {
    std::printf("No se escriben pesos: la red no superó la prueba final.\n");
  }
  log.write(logPath);
  std::printf("Registro: %s\n", logPath.c_str());
  return 0;
}
