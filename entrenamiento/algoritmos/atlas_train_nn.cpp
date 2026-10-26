#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <functional>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "FirmwareConfig.h"
#include "LineSensorArray.h"
#include "NeuralDerivativeWeights.h"
#include "NeuralPositionWeights.h"
#include "Profiles.h"
#include "ScenarioRunner.h"
#include "TinyMlp.h"
#include "TrackLibrary.h"

using namespace sim;

namespace {

constexpr int kInputs = atlas::neural::kPositionInputs;
constexpr int kHidden = atlas::neural::kPositionHidden;

struct Sample {
  int8_t input[kInputs];
  float target;
  float weighted;
  bool validation;
};

struct Cluster {
  int first = -1;
  int last = -1;
};

Cluster clusterNear(const uint8_t* values, double position) {
  Cluster best;
  double bestDistance = 1e9;
  int i = 0;
  while (i < atlas::kSensorCount) {
    if (values[i] == 0) {
      i++;
      continue;
    }
    Cluster c;
    c.first = i;
    while (i + 1 < atlas::kSensorCount && values[i + 1] != 0) i++;
    c.last = i;
    const double center = (c.first + c.last) * 500.0;
    if (std::abs(center - position) < bestDistance) {
      bestDistance = std::abs(center - position);
      best = c;
    }
    i++;
  }
  return best;
}

std::vector<double> lineCrossings(const Track& track, const RobotModel& robot) {
  const Vec2 left = robot.sensorPosition(0, atlas::kSensorCount);
  const Vec2 right = robot.sensorPosition(atlas::kSensorCount - 1, atlas::kSensorCount);
  const Vec2 axis = (right - left) * (1.0 / (atlas::kSensorCount - 1));
  std::vector<double> crossings;
  const double step = 0.02;
  double previous = 1e9, current = 1e9;
  for (double index = -2.0; index <= atlas::kSensorCount + 1.0; index += step) {
    const double next = track.distanceToLine(left + axis * index);
    if (current < previous && current <= next && current < 0.0012) {
      crossings.push_back((index - step) * 1000.0);
    }
    previous = current;
    current = next;
  }
  return crossings;
}

struct DataRun {
  std::size_t track;
  RobotParams robot;
  SensorModel sensors;
  int pwm;
  uint32_t seed;
  bool validation;
};

std::vector<Sample> collect(const std::vector<TrackEntry>& tracks, const DataRun& run) {
  atlas::RobotConfig firmware = atlas::safeProfile();
  firmware.sensors.exposeNormalizedValues = true;
  ScenarioRunner runner(run.robot, run.sensors, firmware);

  std::vector<Sample> samples;
  int tick = 0;
  runner.setObserver([&](double, const atlas::LineSensorArray& sensors, const atlas::LineFollower& follower,
                         const RobotModel& robot) {
    if (!follower.isRunning() || (tick++ % 40) != 0) return;
    const atlas::LineReading& reading = follower.lastReading();
    if (!reading.onLine || reading.ambiguous) return;

    const uint8_t* values = sensors.values();
    const Cluster cluster = clusterNear(values, reading.position);
    if (cluster.first <= 0 || cluster.last >= atlas::kSensorCount - 1) return;
    if (cluster.last - cluster.first + 1 > 3) return;

    const std::vector<double> crossings = lineCrossings(tracks[run.track].track, robot);
    if (crossings.size() != 1) return;
    const double truth = crossings[0];
    if (truth < (cluster.first - 1) * 1000.0 || truth > (cluster.last + 1) * 1000.0) return;

    double weighted = 0, sum = 0;
    for (int i = cluster.first; i <= cluster.last; i++) {
      weighted += values[i] * i * 1000.0;
      sum += values[i];
    }

    Sample s;
    const int center = (cluster.first + cluster.last) / 2;
    const int8_t* normalized = sensors.normalizedValues();
    for (int k = 0; k < kInputs; k++) {
      const int sensor = center + k - kInputs / 2;
      const bool visible = sensor >= cluster.first - 1 && sensor <= cluster.last + 1 && sensor >= 0 &&
                           sensor < atlas::kSensorCount;
      s.input[k] = visible ? normalized[sensor] : 0;
    }
    s.target = static_cast<float>((truth - center * 1000.0) / 1000.0);
    s.weighted = static_cast<float>((weighted / sum - center * 1000.0) / 1000.0);
    s.validation = run.validation;
    samples.push_back(s);
  });

  ScenarioSpec spec;
  spec.trackIndex = run.track;
  spec.cruisePwm = static_cast<int16_t>(run.pwm);
  spec.seed = run.seed;
  spec.impellerPwm = 0;
  spec.sensorNoiseSigma = run.sensors.noiseSigma;
  spec.timeLimitS = 25.0;
  runner.run(tracks[run.track], spec);
  return samples;
}

std::vector<Sample> buildDataset(const std::vector<TrackEntry>& tracks) {
  RobotParams nominal, moderate, real;
  moderate.motorTimeConstant = 0.055;
  moderate.yawTimeConstant = 0.02;
  moderate.commandLatencyS = 0.003;
  real.motorTimeConstant = 0.07;
  real.yawTimeConstant = 0.04;
  real.commandLatencyS = 0.004;
  const RobotParams models[] = {nominal, moderate, real};
  const int speeds[] = {100, 140, 180};

  std::mt19937 random(2026);
  std::uniform_real_distribution<double> spot(0.0028, 0.0060), gain(0.04, 0.20), contrast(0.75, 1.0),
      noise(2.0, 10.0), ambient(0.0, 15.0);
  std::vector<DataRun> runs;
  int index = 0;
  for (std::size_t t = 0; t < tracks.size(); t++) {
    for (const RobotParams& model : models) {
      for (int pwm : speeds) {
        DataRun run;
        run.track = t;
        run.robot = model;
        run.sensors.spotRadius = spot(random);
        run.sensors.gainSpread = gain(random);
        run.sensors.contrast = contrast(random);
        run.sensors.noiseSigma = noise(random);
        run.sensors.ambientOffset = ambient(random);
        run.pwm = pwm;
        run.seed = 700 + static_cast<uint32_t>(index) * 13;
        run.validation = index % 5 == 4;
        runs.push_back(run);
        index++;
      }
    }
  }

  std::vector<Sample> dataset;
  std::mutex lock;
  std::atomic<std::size_t> next{0};
  auto worker = [&] {
    for (std::size_t i = next++; i < runs.size(); i = next++) {
      std::vector<Sample> part = collect(tracks, runs[i]);
      std::lock_guard<std::mutex> guard(lock);
      dataset.insert(dataset.end(), part.begin(), part.end());
    }
  };
  std::vector<std::thread> pool;
  for (unsigned k = 0; k < std::max(2u, std::thread::hardware_concurrency()); k++) pool.emplace_back(worker);
  for (std::thread& t : pool) t.join();
  std::printf("Datos: %zu corridas, %zu muestras\n", runs.size(), dataset.size());
  return dataset;
}

struct FloatNet {
  double w1[kHidden][kInputs];
  double b1[kHidden];
  double w2[kHidden];
  double b2;

  double forward(const int8_t* input, double* hidden) const {
    double out = b2;
    for (int j = 0; j < kHidden; j++) {
      double sum = b1[j];
      for (int i = 0; i < kInputs; i++) sum += w1[j][i] * (input[i] / 127.0);
      hidden[j] = sum > 0 ? sum : 0;
      out += w2[j] * hidden[j];
    }
    return out;
  }
};

constexpr int kParams = kHidden * kInputs + kHidden + kHidden + 1;

double* param(FloatNet& net, int k) {
  if (k < kHidden * kInputs) return &net.w1[k / kInputs][k % kInputs];
  k -= kHidden * kInputs;
  if (k < kHidden) return &net.b1[k];
  k -= kHidden;
  if (k < kHidden) return &net.w2[k];
  return &net.b2;
}

double validationRms(const FloatNet& net, const std::vector<Sample>& data, const std::vector<std::size_t>& indices) {
  double sum = 0;
  double hidden[kHidden];
  for (std::size_t i : indices) {
    const double error = net.forward(data[i].input, hidden) - data[i].target;
    sum += error * error;
  }
  return std::sqrt(sum / std::max<std::size_t>(1, indices.size()));
}

FloatNet train(const std::vector<Sample>& data, std::size_t samples, int maxEpochs) {
  std::mt19937 random(7);
  FloatNet net;
  std::normal_distribution<double> init(0.0, 1.0);
  for (int j = 0; j < kHidden; j++) {
    for (int i = 0; i < kInputs; i++) net.w1[j][i] = init(random) * std::sqrt(2.0 / kInputs);
    net.b1[j] = 0.1;
    net.w2[j] = init(random) * std::sqrt(1.0 / kHidden);
  }
  net.b2 = 0;

  std::vector<std::size_t> order, check;
  for (std::size_t i = 0; i < data.size(); i++) (data[i].validation ? check : order).push_back(i);
  std::shuffle(order.begin(), order.end(), random);
  std::shuffle(check.begin(), check.end(), random);
  if (order.size() > samples) order.resize(samples);
  if (check.size() > 100000) check.resize(100000);
  std::printf("Entrenando con %zu lecturas; parada temprana con %zu lecturas de validación\n", order.size(),
              check.size());

  std::vector<double> m(kParams, 0), v(kParams, 0), grad(kParams);
  const double beta1 = 0.9, beta2 = 0.999;
  long step = 0;
  const std::size_t batch = 128;
  double rate = 3e-3;
  FloatNet best = net;
  double bestRms = 1e9;
  int bestEpoch = 0, sinceBest = 0, sinceDrop = 0;
  for (int epoch = 0; epoch < maxEpochs; epoch++) {
    std::shuffle(order.begin(), order.end(), random);
    double loss = 0;
    for (std::size_t first = 0; first < order.size(); first += batch) {
      std::fill(grad.begin(), grad.end(), 0.0);
      const std::size_t last = std::min(order.size(), first + batch);
      for (std::size_t b = first; b < last; b++) {
        const Sample& s = data[order[b]];
        double hidden[kHidden];
        const double error = net.forward(s.input, hidden) - s.target;
        loss += error * error;
        for (int j = 0; j < kHidden; j++) {
          grad[kHidden * kInputs + kHidden + j] += error * hidden[j];
          if (hidden[j] <= 0) continue;
          const double back = error * net.w2[j];
          for (int i = 0; i < kInputs; i++) grad[j * kInputs + i] += back * (s.input[i] / 127.0);
          grad[kHidden * kInputs + j] += back;
        }
        grad[kParams - 1] += error;
      }
      step++;
      const double scale = 1.0 / static_cast<double>(last - first);
      for (int k = 0; k < kParams; k++) {
        const double g = grad[k] * scale;
        m[k] = beta1 * m[k] + (1 - beta1) * g;
        v[k] = beta2 * v[k] + (1 - beta2) * g * g;
        const double mHat = m[k] / (1 - std::pow(beta1, step));
        const double vHat = v[k] / (1 - std::pow(beta2, step));
        *param(net, k) -= rate * mHat / (std::sqrt(vHat) + 1e-8);
      }
    }
    const double valRms = validationRms(net, data, check);
    if (valRms < bestRms * 0.9995) {
      bestRms = valRms;
      best = net;
      bestEpoch = epoch;
      sinceBest = sinceDrop = 0;
    } else {
      sinceBest++;
      if (++sinceDrop >= 40 && rate > 1e-5) {
        rate *= 0.5;
        sinceDrop = 0;
      }
    }
    if (epoch % 50 == 0) {
      std::printf("  época %4d  RMS entrenamiento %.4f  validación %.4f sensores  (paso %.1e)\n", epoch,
                  std::sqrt(loss / order.size()), valRms, rate);
      std::fflush(stdout);
    }
    if (sinceBest >= 200) break;
  }
  std::printf("Mejor época %d: RMS de validación %.4f sensores (%.3f mm)\n", bestEpoch, bestRms, bestRms * 9.0);
  return best;
}

struct QuantNet {
  int8_t w1[kHidden * kInputs];
  int32_t b1[kHidden];
  uint8_t shift1;
  int16_t w2[kHidden];
  int32_t b2[1];
  uint8_t shift2;
  atlas::TinyMlpWeights view() const {
    return {static_cast<uint8_t>(kInputs), static_cast<uint8_t>(kHidden), 1, w1, b1, shift1, w2, b2, shift2};
  }
};

QuantNet quantize(const FloatNet& net, const std::vector<Sample>& data) {
  QuantNet q;
  double maxW1 = 1e-9;
  for (int j = 0; j < kHidden; j++)
    for (int i = 0; i < kInputs; i++) maxW1 = std::max(maxW1, std::abs(net.w1[j][i]));
  const double scaleW1 = 127.0 / maxW1;
  for (int j = 0; j < kHidden; j++) {
    for (int i = 0; i < kInputs; i++) q.w1[j * kInputs + i] = static_cast<int8_t>(std::lround(net.w1[j][i] * scaleW1));
    q.b1[j] = static_cast<int32_t>(std::lround(net.b1[j] * scaleW1 * 127.0));
  }
  double maxHidden = 1e-9;
  for (const Sample& s : data) {
    double hidden[kHidden];
    net.forward(s.input, hidden);
    for (double h : hidden) maxHidden = std::max(maxHidden, h);
  }
  const double accScale = scaleW1 * 127.0;
  q.shift1 = 0;
  while (maxHidden * accScale / std::pow(2.0, q.shift1) > 12000.0) q.shift1++;
  const double hiddenScale = accScale / std::pow(2.0, q.shift1);

  double maxW2 = 1e-9;
  for (double w : net.w2) maxW2 = std::max(maxW2, std::abs(w));
  q.shift2 = 0;
  for (int s = 0; s <= 20; s++) {
    const double factor = 1000.0 * std::pow(2.0, s) / hiddenScale;
    const double accMax = (maxW2 * factor) * 16000.0 * kHidden + std::abs(net.b2) * 1000.0 * std::pow(2.0, s);
    if (maxW2 * factor > 32000.0 || accMax > 1.0e9) break;
    q.shift2 = static_cast<uint8_t>(s);
  }
  const double factor = 1000.0 * std::pow(2.0, q.shift2) / hiddenScale;
  for (int j = 0; j < kHidden; j++) q.w2[j] = static_cast<int16_t>(std::lround(net.w2[j] * factor));
  q.b2[0] = static_cast<int32_t>(std::lround(net.b2 * 1000.0 * std::pow(2.0, q.shift2)));
  return q;
}

struct Errors {
  double weighted = 0, network = 0, quantized = 0, maxQuantized = 0;
  std::size_t count = 0;
};

Errors evaluate(const FloatNet& net, const QuantNet& q, const std::vector<Sample>& data, bool validation) {
  Errors e;
  const atlas::TinyMlpWeights weights = q.view();
  const atlas::TinyMlp mlp(weights);
  for (const Sample& s : data) {
    if (s.validation != validation) continue;
    double hidden[kHidden];
    const double floatOut = net.forward(s.input, hidden);
    int32_t quantOut = 0;
    mlp.evaluate(s.input, &quantOut);
    const double errorW = (s.weighted - s.target) * 1000.0;
    const double errorN = (floatOut - s.target) * 1000.0;
    const double errorQ = quantOut - s.target * 1000.0;
    e.weighted += errorW * errorW;
    e.network += errorN * errorN;
    e.quantized += errorQ * errorQ;
    e.maxQuantized = std::max(e.maxQuantized, std::abs(errorQ));
    e.count++;
  }
  const double n = static_cast<double>(std::max<std::size_t>(1, e.count));
  e.weighted = std::sqrt(e.weighted / n);
  e.network = std::sqrt(e.network / n);
  e.quantized = std::sqrt(e.quantized / n);
  return e;
}

template <typename T>
std::string join(const T* values, int count, int perLine) {
  std::string text;
  for (int i = 0; i < count; i++) {
    if (i % perLine == 0) text += "\n    ";
    text += std::to_string(static_cast<long>(values[i]));
    if (i + 1 < count) text += ", ";
  }
  return text;
}

void writeHeader(const std::string& path, const QuantNet& q, const Errors& e, std::size_t samples) {
  char date[32];
  const std::time_t now = std::time(nullptr);
  std::strftime(date, sizeof(date), "%Y-%m-%d", std::localtime(&now));
  std::ofstream out(path);
  out << "/*\n"
      << "  NeuralPositionWeights.h - GENERADO por entrenamiento/algoritmos/atlas_train_nn.cpp\n"
      << "  (atlas_train_nn posicion). No editar a mano.\n\n"
      << "  Red " << kInputs << " -> " << kHidden << " ReLU -> 1: posición fina de la línea.\n"
      << "  Entrenada el " << date << " con " << samples << " lecturas simuladas (parada temprana).\n"
      << "  Validación (posición, RMS en milésimas de sensor; 1000 = 9 mm):\n"
      << "    promedio ponderado " << std::lround(e.weighted) << ", red " << std::lround(e.quantized)
      << " (máx " << std::lround(e.maxQuantized) << ")\n"
      << "*/\n"
      << "#pragma once\n\n#include <stdint.h>\n\n#include \"Progmem.h\"\n#include \"TinyMlp.h\"\n\n"
      << "namespace atlas {\nnamespace neural {\n\n"
      << "constexpr uint8_t kPositionInputs = " << kInputs << ";\n"
      << "constexpr uint8_t kPositionHidden = " << kHidden << ";\n\n"
      << "ATLAS_PROGMEM const int8_t kPositionWeights1[kPositionHidden * kPositionInputs] = {"
      << join(q.w1, kHidden * kInputs, kInputs) << "};\n"
      << "ATLAS_PROGMEM const int32_t kPositionBias1[kPositionHidden] = {" << join(q.b1, kHidden, 5) << "};\n"
      << "ATLAS_PROGMEM const int16_t kPositionWeights2[kPositionHidden] = {" << join(q.w2, kHidden, 10)
      << "};\n"
      << "ATLAS_PROGMEM const int32_t kPositionBias2[1] = {" << join(q.b2, 1, 1) << "};\n\n"
      << "const TinyMlpWeights kPositionNet = {kPositionInputs, kPositionHidden, 1,\n"
      << "                                     kPositionWeights1, kPositionBias1, " << int(q.shift1) << ",\n"
      << "                                     kPositionWeights2, kPositionBias2, " << int(q.shift2) << "};\n\n"
      << "}  // namespace neural\n}  // namespace atlas\n";
}

int trainPosition(std::size_t samples) {
  const std::vector<TrackEntry> tracks = buildTrackLibrary();
  const std::vector<Sample> data = buildDataset(tracks);
  std::size_t validation = 0;
  for (const Sample& s : data) validation += s.validation;
  std::printf("Disponibles: entrenamiento %zu, validación %zu\n", data.size() - validation, validation);

  const FloatNet net = train(data, samples, 5000);
  const QuantNet q = quantize(net, data);
  const Errors train = evaluate(net, q, data, false);
  const Errors val = evaluate(net, q, data, true);
  std::printf("\nRMS de posición (milésimas de sensor, 1000 = 9 mm)\n");
  std::printf("                 ponderado   red(double)   red(enteros)   máx(enteros)\n");
  std::printf("  entrenamiento  %7.0f     %7.0f       %7.0f        %7.0f\n", train.weighted, train.network,
              train.quantized, train.maxQuantized);
  std::printf("  validación     %7.0f     %7.0f       %7.0f        %7.0f\n", val.weighted, val.network,
              val.quantized, val.maxQuantized);
  std::printf("  mejora en validación: %.0f %%\n", 100.0 * (1.0 - val.quantized / val.weighted));
  std::printf("  shift1 %d  shift2 %d\n", q.shift1, q.shift2);

  const std::string path = std::string(ATLAS_SOURCE_DIR) + "/firmware/bibliotecas/AtlasCore/src/red_neuronal/NeuralPositionWeights.h";
  writeHeader(path, q, val, std::min(samples, data.size() - validation));
  std::printf("Pesos escritos en %s\n", path.c_str());
  return 0;
}

namespace derivative {

static_assert(atlas::neural::kDerivativeInputs == kInputs, "misma forma que la red de posición");
static_assert(atlas::neural::kDerivativeHidden == kHidden, "misma forma que la red de posición");

bool trueCrossing(const Track& track, const RobotModel& robot, double* out) {
  const Vec2 left = robot.sensorPosition(0, atlas::kSensorCount);
  const Vec2 right = robot.sensorPosition(atlas::kSensorCount - 1, atlas::kSensorCount);
  const Vec2 axis = (right - left) * (1.0 / (atlas::kSensorCount - 1));
  const double step = 0.05;
  int found = 0;
  double a = 1e9, b = 1e9;
  for (double index = -2.0; index <= atlas::kSensorCount + 1.0; index += step) {
    const double c = track.distanceToLine(left + axis * index);
    if (b < a && b <= c && b < 0.0012) {
      if (++found > 1) return false;
      const double x = index - step;
      const double slope = std::max(a - b, c - b);
      *out = (x + (slope > 1e-12 ? step * (a - c) / (2.0 * slope) : 0.0)) * 1000.0;
    }
    a = b;
    b = c;
  }
  return found == 1;
}

struct Record {
  uint32_t cycle;
  int8_t input[kInputs];
  float classic;
  double truth;
  bool usable;
};

std::vector<Sample> collect(const std::vector<TrackEntry>& tracks, const DataRun& run, int lead,
                            const atlas::RobotConfig& firmware) {
  ScenarioRunner runner(run.robot, run.sensors, firmware);
  std::vector<Sample> samples;
  std::vector<Record> recent;
  uint32_t lastCycle = 0;
  uint32_t cleanSince = 0;
  runner.setObserver([&](double, const atlas::LineSensorArray&, const atlas::LineFollower& follower,
                         const RobotModel& robot) {
    const uint32_t cycle = follower.controlCycles();
    if (!follower.isRunning() || cycle == lastCycle) return;
    lastCycle = cycle;
    const atlas::LineReading& reading = follower.lastReading();
    Record r;
    r.cycle = cycle;
    if (!reading.onLine || reading.ambiguous || !trueCrossing(tracks[run.track].track, robot, &r.truth)) {
      recent.clear();
      cleanSince = 0;
      return;
    }
    cleanSince++;
    std::memcpy(r.input, follower.derivativeInputs(), kInputs);
    r.classic = static_cast<float>(follower.classicDerivative());
    r.usable = cleanSince >= 20;
    if (!recent.empty() && recent.back().cycle + 1 != cycle) recent.clear();
    recent.push_back(r);
    if (recent.size() > static_cast<std::size_t>(lead + 2)) recent.erase(recent.begin());
    if (recent.size() < static_cast<std::size_t>(lead + 2)) return;
    const Record& sample = recent[0];
    if (!sample.usable || (sample.cycle % 3) != 0) return;
    Sample s;
    std::memcpy(s.input, sample.input, kInputs);
    s.target = static_cast<float>((recent[lead + 1].truth - recent[lead].truth) / 1000.0);
    s.weighted = sample.classic / 1000.0f;
    s.validation = run.validation;
    samples.push_back(s);
  });

  ScenarioSpec spec;
  spec.trackIndex = run.track;
  spec.cruisePwm = static_cast<int16_t>(run.pwm);
  spec.seed = run.seed;
  spec.impellerPwm = 0;
  spec.sensorNoiseSigma = run.sensors.noiseSigma;
  spec.timeLimitS = 25.0;
  runner.run(tracks[run.track], spec);
  return samples;
}

std::vector<Sample> buildDataset(const std::vector<TrackEntry>& tracks, int lead) {
  RobotParams nominal, moderate, real;
  moderate.motorTimeConstant = 0.055;
  moderate.yawTimeConstant = 0.02;
  moderate.commandLatencyS = 0.003;
  real.motorTimeConstant = 0.07;
  real.yawTimeConstant = 0.04;
  real.commandLatencyS = 0.004;
  const RobotParams models[] = {nominal, moderate, real};
  const int speeds[] = {100, 140, 180};
  atlas::RobotConfig firmware = atlas::safeProfile();

  std::mt19937 random(4049);
  std::uniform_real_distribution<double> spot(0.0028, 0.0060), gain(0.04, 0.20), contrast(0.75, 1.0),
      noise(2.0, 10.0), ambient(0.0, 15.0);
  std::vector<DataRun> runs;
  int index = 0;
  for (std::size_t t = 0; t < tracks.size(); t++) {
    for (const RobotParams& model : models) {
      for (int pwm : speeds) {
        DataRun run;
        run.track = t;
        run.robot = model;
        run.sensors.spotRadius = spot(random);
        run.sensors.gainSpread = gain(random);
        run.sensors.contrast = contrast(random);
        run.sensors.noiseSigma = noise(random);
        run.sensors.ambientOffset = ambient(random);
        run.pwm = pwm;
        run.seed = 9100 + static_cast<uint32_t>(index) * 17;
        run.validation = t % 5 == 4;
        runs.push_back(run);
        index++;
      }
    }
  }

  std::vector<Sample> dataset;
  std::mutex lock;
  std::atomic<std::size_t> next{0};
  auto worker = [&] {
    for (std::size_t i = next++; i < runs.size(); i = next++) {
      std::vector<Sample> part = collect(tracks, runs[i], lead, firmware);
      std::lock_guard<std::mutex> guard(lock);
      dataset.insert(dataset.end(), part.begin(), part.end());
    }
  };
  std::vector<std::thread> pool;
  for (unsigned k = 0; k < std::max(2u, std::thread::hardware_concurrency()); k++) pool.emplace_back(worker);
  for (std::thread& t : pool) t.join();
  std::printf("Datos: %zu corridas, %zu muestras\n", runs.size(), dataset.size());
  return dataset;
}

void writeHeader(const std::string& path, const QuantNet& q, const Errors& e, std::size_t samples, int lead) {
  char date[32];
  const std::time_t now = std::time(nullptr);
  std::strftime(date, sizeof(date), "%Y-%m-%d", std::localtime(&now));
  std::ofstream out(path);
  out << "/*\n"
      << "  NeuralDerivativeWeights.h - GENERADO por entrenamiento/algoritmos/atlas_train_nn.cpp\n"
      << "  (atlas_train_nn derivada). No editar a mano.\n\n"
      << "  Red " << kInputs << " -> " << kHidden << " ReLU -> 1: cambio del error por ciclo de control\n"
      << "  (adelanto " << lead << " ciclos), desde las pendientes del historial del error.\n"
      << "  Entrenada el " << date << " con " << samples << " ciclos simulados (parada temprana).\n"
      << "  Validación (RMS en unidades de error por ciclo):\n"
      << "    D filtrado clásico " << e.weighted << ", red " << e.quantized << " (máx " << e.maxQuantized
      << ")\n"
      << "*/\n"
      << "#pragma once\n\n#include <stdint.h>\n\n#include \"Progmem.h\"\n#include \"TinyMlp.h\"\n\n"
      << "namespace atlas {\nnamespace neural {\n\n"
      << "constexpr bool kDerivativeTrained = true;\n"
      << "constexpr uint8_t kDerivativeInputs = " << kInputs << ";\n"
      << "constexpr uint8_t kDerivativeHidden = " << kHidden << ";\n\n"
      << "ATLAS_PROGMEM const int8_t kDerivativeWeights1[kDerivativeHidden * kDerivativeInputs] = {"
      << join(q.w1, kHidden * kInputs, kInputs) << "};\n"
      << "ATLAS_PROGMEM const int32_t kDerivativeBias1[kDerivativeHidden] = {" << join(q.b1, kHidden, 5) << "};\n"
      << "ATLAS_PROGMEM const int16_t kDerivativeWeights2[kDerivativeHidden] = {" << join(q.w2, kHidden, 10)
      << "};\n"
      << "ATLAS_PROGMEM const int32_t kDerivativeBias2[1] = {" << join(q.b2, 1, 1) << "};\n\n"
      << "const TinyMlpWeights kDerivativeNet = {kDerivativeInputs, kDerivativeHidden, 1,\n"
      << "                                       kDerivativeWeights1, kDerivativeBias1, " << int(q.shift1)
      << ",\n"
      << "                                       kDerivativeWeights2, kDerivativeBias2, " << int(q.shift2)
      << "};\n\n"
      << "}  // namespace neural\n}  // namespace atlas\n";
}

int train(std::size_t samples, int lead, bool write) {
  const std::vector<TrackEntry> tracks = buildTrackLibrary();
  const std::vector<Sample> data = buildDataset(tracks, lead);
  std::size_t validation = 0;
  for (const Sample& s : data) validation += s.validation;
  std::printf("Disponibles: entrenamiento %zu, validación %zu (pistas separadas)\n", data.size() - validation,
              validation);
  double spread = 0;
  for (const Sample& s : data) spread += s.target * s.target;
  std::printf("RMS de la derivada verdadera: %.1f unidades/ciclo\n", 1000.0 * std::sqrt(spread / data.size()));
  double cov = 0, varClassic = 0;
  for (const Sample& s : data) {
    cov += s.weighted * s.target;
    varClassic += s.weighted * s.weighted;
  }
  std::printf("D clásico / verdadera: %.2f  (correlación %.2f)\n", cov / spread, cov / std::sqrt(spread * varClassic));

  const FloatNet net = ::train(data, samples, 3000);
  const QuantNet q = quantize(net, data);
  const Errors tr = evaluate(net, q, data, false);
  const Errors val = evaluate(net, q, data, true);
  std::printf("\nRMS de la derivada (unidades de error por ciclo; 1000 = 1 sensor)\n");
  std::printf("                 D clásico   red(double)   red(enteros)   máx(enteros)\n");
  std::printf("  entrenamiento  %7.2f     %7.2f       %7.2f        %7.0f\n", tr.weighted, tr.network,
              tr.quantized, tr.maxQuantized);
  std::printf("  validación     %7.2f     %7.2f       %7.2f        %7.0f\n", val.weighted, val.network,
              val.quantized, val.maxQuantized);
  std::printf("  mejora en validación: %.0f %%\n", 100.0 * (1.0 - val.quantized / val.weighted));
  std::printf("  shift1 %d  shift2 %d\n", q.shift1, q.shift2);
  if (!write) return 0;
  const std::string path = std::string(ATLAS_SOURCE_DIR) + "/firmware/bibliotecas/AtlasCore/src/red_neuronal/NeuralDerivativeWeights.h";
  writeHeader(path, q, val, std::min(samples, data.size() - validation), lead);
  std::printf("Pesos escritos en %s\n", path.c_str());
  return 0;
}

}

namespace gains {

struct Case {
  std::size_t track;
  int model;
  uint32_t seed;
};

RobotParams robotModel(int model) {
  RobotParams p;
  if (model == 1) {
    p.motorTimeConstant = 0.055;
    p.yawTimeConstant = 0.02;
    p.commandLatencyS = 0.003;
  } else if (model == 2) {
    p.motorTimeConstant = 0.07;
    p.yawTimeConstant = 0.04;
    p.commandLatencyS = 0.004;
  }
  return p;
}

std::vector<Case> makeCases(const std::vector<TrackEntry>& tracks, uint32_t seedBase, int seedsPerTrack,
                            bool allTracks) {
  std::vector<Case> cases;
  for (std::size_t t = 0; t < tracks.size(); t++) {
    const std::string& name = tracks[t].track.name();
    if (name == "Trébol de lazos" || name == "Triángulo de rizos") continue;
    const bool training = t <= 1 || name == "Gran Premio" || name.find("Hexágono") != std::string::npos;
    if (!allTracks && !training) continue;
    for (int m = 0; m < 3; m++) {
      for (int k = 0; k < seedsPerTrack; k++) cases.push_back({t, m, seedBase + static_cast<uint32_t>(t * 101 + m * 17 + k * 7)});
    }
  }
  return cases;
}

struct Gains {
  double safeKp, safeKd, fastKp, fastKd;
};

struct GainResult {
  int failures = 0;
  double weight = 0, zigzag = 0, steering = 0, p95 = 0;
  double fastTime = 0;
  int fastLaps = 0;
  double meanZigzag() const { return weight ? zigzag / weight : 0; }
  double meanSteering() const { return weight ? steering / weight : 0; }
  double meanP95() const { return weight ? p95 / weight : 0; }
  double meanFastTime() const { return fastLaps ? fastTime / fastLaps : 0; }
  void add(const GainResult& o) {
    failures += o.failures;
    weight += o.weight;
    zigzag += o.zigzag;
    steering += o.steering;
    p95 += o.p95;
    fastTime += o.fastTime;
    fastLaps += o.fastLaps;
  }
};

GainResult runGains(const std::vector<TrackEntry>& tracks, const Case& c, const Gains& g) {
  atlas::RobotConfig safe = atlas::firmwareSafeConfig(true), fast = atlas::firmwareFastConfig(true);
  safe.control.neuralGains = fast.control.neuralGains = false;
  safe.control.kp = static_cast<float>(g.safeKp);
  safe.control.kd = static_cast<float>(g.safeKd);
  fast.control.kp = static_cast<float>(g.fastKp);
  fast.control.kd = static_cast<float>(g.fastKd);
  ScenarioSpec spec;
  spec.trackIndex = c.track;
  spec.cruisePwm = safe.speed.basePwm;
  spec.seed = c.seed;
  spec.learnTrack = true;
  spec.attempts = 3;
  spec.impellerPwm = safe.impeller.pwm;
  spec.fastImpellerPwm = fast.impeller.pwm;
  const std::vector<ScenarioResult> attempts =
      ScenarioRunner(robotModel(c.model), SensorModel(), safe, fast).runAttempts(tracks[c.track], spec);
  GainResult r;
  const double w = c.model == 2 ? 1.5 : 1.0;
  for (std::size_t a = 0; a < attempts.size(); a++) {
    const ScenarioResult& res = attempts[a];
    if (!res.passed()) {
      r.failures++;
      continue;
    }
    r.weight += w;
    r.zigzag += w * res.zigzagPerMeter;
    r.steering += w * res.steeringRmsPwm;
    r.p95 += w * res.p95OffsetMm;
    if (a > 0) {
      r.fastTime += res.raceTime;
      r.fastLaps++;
    }
  }
  return r;
}

std::vector<GainResult> runAllGains(const std::vector<TrackEntry>& tracks, const std::vector<Case>& cases,
                                    const std::vector<Gains>& configs,
                                    std::vector<std::vector<GainResult>>* perTrack = nullptr) {
  std::vector<std::vector<GainResult>> parts(configs.size(), std::vector<GainResult>(cases.size()));
  std::atomic<std::size_t> next{0}, done{0};
  const std::size_t jobs = configs.size() * cases.size();
  auto worker = [&] {
    for (std::size_t j = next++; j < jobs; j = next++) {
      parts[j / cases.size()][j % cases.size()] = runGains(tracks, cases[j % cases.size()], configs[j / cases.size()]);
      const std::size_t d = ++done;
      if (d % 500 == 0) {
        std::fprintf(stderr, "\r  %zu/%zu competencias", d, jobs);
      }
    }
  };
  std::vector<std::thread> pool;
  for (unsigned k = 0; k < std::max(2u, std::thread::hardware_concurrency()); k++) pool.emplace_back(worker);
  for (std::thread& t : pool) t.join();
  std::fprintf(stderr, "\n");
  std::vector<GainResult> totals(configs.size());
  if (perTrack) perTrack->assign(configs.size(), std::vector<GainResult>(tracks.size()));
  for (std::size_t i = 0; i < configs.size(); i++) {
    for (std::size_t k = 0; k < cases.size(); k++) {
      totals[i].add(parts[i][k]);
      if (perTrack) (*perTrack)[i][cases[k].track].add(parts[i][k]);
    }
  }
  return totals;
}

bool acceptable(const GainResult& r, const GainResult& current) {
  return r.failures == 0 && r.meanFastTime() <= current.meanFastTime() * 1.03;
}

bool smoother(const GainResult& a, const GainResult& b) {
  if (std::abs(a.meanZigzag() - b.meanZigzag()) > 0.005) return a.meanZigzag() < b.meanZigzag();
  return a.meanSteering() < b.meanSteering();
}

void printGains(const char* label, const Gains& g, const GainResult& r, const GainResult& current) {
  std::printf("  %-10s seguro kp %.4f kd %.2f | rápido kp %.4f kd %.2f | salidas %3d  zigzag %.3f/m  dirección %.1f  "
              "p95 %.1fmm  mapa %.2fs (%+.1f %%)\n",
              label, g.safeKp, g.safeKd, g.fastKp, g.fastKd, r.failures, r.meanZigzag(), r.meanSteering(), r.meanP95(),
              r.meanFastTime(), 100.0 * (r.meanFastTime() / current.meanFastTime() - 1.0));
}

int sweepGains() {
  const std::vector<TrackEntry> tracks = buildTrackLibrary();
  const atlas::RobotConfig safeNow = atlas::safeProfile(), fastNow = atlas::fastProfile();
  const Gains current = {safeNow.control.kp, safeNow.control.kd, fastNow.control.kp, fastNow.control.kd};

  const int count = 150;
  std::mt19937 random(31);
  std::vector<std::vector<double>> strata(4, std::vector<double>(count));
  for (auto& dim : strata) {
    for (int i = 0; i < count; i++) dim[i] = (i + std::uniform_real_distribution<double>(0, 1)(random)) / count;
    std::shuffle(dim.begin(), dim.end(), random);
  }
  std::vector<Gains> configs = {current};
  for (int i = 0; i < count; i++) {
    configs.push_back({0.0035 + strata[0][i] * (current.safeKp - 0.0035), 0.55 + strata[1][i] * (current.safeKd - 0.55),
                       0.0035 + strata[2][i] * (current.fastKp - 0.0035), 0.55 + strata[3][i] * (current.fastKd - 0.55)});
  }
  const std::vector<Case> stage1 = makeCases(tracks, 300000, 4, true);
  std::printf("Etapa 1: %zu combinaciones × %zu competencias = %zu\n", configs.size(), stage1.size(),
              configs.size() * stage1.size());
  std::fflush(stdout);
  const std::vector<GainResult> r1 = runAllGains(tracks, stage1, configs);
  long total = static_cast<long>(configs.size() * stage1.size());
  std::vector<int> passing;
  for (std::size_t i = 1; i < configs.size(); i++)
    if (acceptable(r1[i], r1[0])) passing.push_back(static_cast<int>(i));
  std::sort(passing.begin(), passing.end(), [&](int a, int b) { return smoother(r1[a], r1[b]); });
  printGains("actual", configs[0], r1[0], r1[0]);
  std::printf("  %zu de %d combinaciones cumplen (0 salidas, mapa <= +3 %%). Las 10 más suaves:\n", passing.size(), count);
  if (passing.size() > 10) passing.resize(10);
  for (int i : passing) printGains("", configs[i], r1[i], r1[0]);
  std::fflush(stdout);
  if (passing.empty()) {
    std::printf("Ninguna combinación más baja cumple: se conservan kp/kd actuales.\n");
    return 0;
  }

  std::vector<Gains> finalists = {current};
  for (int i : passing) finalists.push_back(configs[i]);
  const std::vector<Case> stage2 = makeCases(tracks, 400000, 18, true);
  std::printf("\nEtapa 2: %zu combinaciones × %zu competencias = %zu\n", finalists.size(), stage2.size(),
              finalists.size() * stage2.size());
  std::fflush(stdout);
  const std::vector<GainResult> r2 = runAllGains(tracks, stage2, finalists);
  total += static_cast<long>(finalists.size() * stage2.size());
  std::vector<int> best2;
  for (std::size_t i = 1; i < finalists.size(); i++)
    if (acceptable(r2[i], r2[0])) best2.push_back(static_cast<int>(i));
  std::sort(best2.begin(), best2.end(), [&](int a, int b) { return smoother(r2[a], r2[b]); });
  printGains("actual", finalists[0], r2[0], r2[0]);
  for (int i : best2) printGains("", finalists[i], r2[i], r2[0]);
  std::fflush(stdout);
  if (best2.empty()) {
    std::printf("Ninguna combinación pasó la etapa 2: se conservan kp/kd actuales.\n");
    return 0;
  }
  if (best2.size() > 3) best2.resize(3);

  std::vector<Gains> last = {current};
  for (int i : best2) last.push_back(finalists[i]);
  const std::vector<Case> stage3 = makeCases(tracks, 500000, 45, true);
  std::printf("\nEtapa 3 (validación): %zu combinaciones × %zu competencias = %zu\n", last.size(), stage3.size(),
              last.size() * stage3.size());
  std::fflush(stdout);
  std::vector<std::vector<GainResult>> perTrack;
  const std::vector<GainResult> r3 = runAllGains(tracks, stage3, last, &perTrack);
  total += static_cast<long>(last.size() * stage3.size());
  int winner = 0;
  for (std::size_t i = 1; i < last.size(); i++) {
    if (acceptable(r3[i], r3[0]) && (winner == 0 || smoother(r3[i], r3[winner]))) winner = static_cast<int>(i);
  }
  printGains("actual", last[0], r3[0], r3[0]);
  for (std::size_t i = 1; i < last.size(); i++) printGains(static_cast<int>(i) == winner ? "GANADORA" : "", last[i], r3[i], r3[0]);
  std::printf("\nTus pistas (validación):\n");
  for (std::size_t t = 0; t < 2; t++) {
    std::printf(" %s\n", tracks[t].track.name().c_str());
    printGains("actual", last[0], perTrack[0][t], perTrack[0][t]);
    if (winner) printGains("ganadora", last[winner], perTrack[winner][t], perTrack[0][t]);
  }
  std::printf("\nCompetencias simuladas en total: %ld\n", total);
  if (winner == 0) std::printf("Ninguna superó la validación: se conservan kp/kd actuales.\n");
  return 0;
}

double visibleZigzag(const ScenarioResult& r) {
  int side = 0, crossings = 0;
  for (const TelemetrySample& t : r.telemetry) {
    const int now = t.lateralOffset > 0.010 ? 1 : (t.lateralOffset < -0.010 ? -1 : 0);
    if (now != 0 && side != 0 && now != side) crossings++;
    if (now != 0) side = now;
  }
  return r.distance > 0 ? crossings / r.distance : 0;
}

struct Outcome {
  int failures = 0, laps = 0, fastLaps = 0;
  double zigzag = 0, steering = 0, fastTime = 0;
  void add(const Outcome& o) {
    failures += o.failures;
    laps += o.laps;
    fastLaps += o.fastLaps;
    zigzag += o.zigzag;
    steering += o.steering;
    fastTime += o.fastTime;
  }
  double meanZigzag() const { return laps ? zigzag / laps : 0; }
  double meanSteering() const { return laps ? steering / laps : 0; }
  double meanFastTime() const { return fastLaps ? fastTime / fastLaps : 0; }
};

struct Variant {
  std::string label;
  std::function<void(atlas::RobotConfig&)> tweak;
};

Outcome runCase(const std::vector<TrackEntry>& tracks, const Case& c, const Variant& v) {
  atlas::RobotConfig safe = atlas::firmwareSafeConfig(true), fast = atlas::firmwareFastConfig(true);
  safe.control.neuralGains = fast.control.neuralGains = false;
  safe.control.neuralDerivative = fast.control.neuralDerivative = false;
  if (v.tweak) {
    v.tweak(safe);
    v.tweak(fast);
  }
  ScenarioSpec spec;
  spec.trackIndex = c.track;
  spec.cruisePwm = safe.speed.basePwm;
  spec.seed = c.seed;
  spec.learnTrack = true;
  spec.attempts = 3;
  spec.impellerPwm = safe.impeller.pwm;
  spec.fastImpellerPwm = fast.impeller.pwm;
  const std::vector<ScenarioResult> attempts =
      ScenarioRunner(robotModel(c.model), SensorModel(), safe, fast).runAttempts(tracks[c.track], spec);
  Outcome o;
  for (std::size_t a = 0; a < attempts.size(); a++) {
    const ScenarioResult& r = attempts[a];
    if (!r.passed()) {
      o.failures++;
      continue;
    }
    o.laps++;
    o.zigzag += visibleZigzag(r);
    o.steering += r.steeringRmsPwm;
    if (a > 0) {
      o.fastTime += r.raceTime;
      o.fastLaps++;
    }
  }
  return o;
}

void printOutcome(const char* label, const Outcome& o, const Outcome& reference) {
  std::printf("  %-26s salidas %3d  zigzag±10mm %.3f/m  dirección %5.1f  mapa %6.2fs (%+.1f %%)\n", label, o.failures,
              o.meanZigzag(), o.meanSteering(), o.meanFastTime(),
              reference.meanFastTime() > 0 ? 100.0 * (o.meanFastTime() / reference.meanFastTime() - 1.0) : 0.0);
}

using Table = std::vector<std::vector<std::vector<Outcome>>>;

Table runVariants(const std::vector<TrackEntry>& tracks, const std::vector<Case>& cases,
                  const std::vector<Variant>& variants) {
  std::vector<Outcome> parts(variants.size() * cases.size());
  std::atomic<std::size_t> next{0};
  auto worker = [&] {
    for (std::size_t j = next++; j < parts.size(); j = next++)
      parts[j] = runCase(tracks, cases[j % cases.size()], variants[j / cases.size()]);
  };
  std::vector<std::thread> pool;
  for (unsigned k = 0; k < std::max(2u, std::thread::hardware_concurrency()); k++) pool.emplace_back(worker);
  for (std::thread& t : pool) t.join();
  Table table(variants.size(), std::vector<std::vector<Outcome>>(tracks.size(), std::vector<Outcome>(3)));
  for (std::size_t j = 0; j < parts.size(); j++) {
    const Case& c = cases[j % cases.size()];
    table[j / cases.size()][c.track][c.model].add(parts[j]);
  }
  return table;
}

Outcome total(const std::vector<std::vector<Outcome>>& byTrack) {
  Outcome o;
  for (const auto& models : byTrack)
    for (const Outcome& m : models) o.add(m);
  return o;
}

bool meetsLimits(const std::vector<TrackEntry>& tracks, const Table& table, std::size_t v, bool print) {
  bool ok = true;
  const char* models[] = {"nominal", "moderado", "real"};
  for (std::size_t t = 0; t < tracks.size(); t++) {
    for (int m = 0; m < 3; m++) {
      const Outcome& a = table[0][t][m];
      const Outcome& b = table[v][t][m];
      if (a.laps + a.failures == 0) continue;
      const double slower = a.meanFastTime() > 0 ? b.meanFastTime() / a.meanFastTime() - 1.0 : 0.0;
      const bool pass = b.failures <= a.failures && slower <= 0.03;
      ok = ok && pass;
      if (print)
        std::printf("  %-28s %-9s salidas %2d->%2d  zigzag %.3f->%.3f  mapa %6.2f->%6.2fs (%+.1f %%)%s\n",
                    tracks[t].track.name().c_str(), models[m], a.failures, b.failures, a.meanZigzag(), b.meanZigzag(),
                    a.meanFastTime(), b.meanFastTime(), 100.0 * slower, pass ? "" : "  <- NO CUMPLE");
    }
  }
  return ok;
}

int compare(const std::vector<TrackEntry>& tracks, const std::vector<Case>& cases, const std::vector<Variant>& variants,
            std::size_t detail) {
  const Table table = runVariants(tracks, cases, variants);
  const Outcome reference = total(table[0]);
  std::printf("Lazo cerrado: %zu competencias por variante (3 intentos cada una)\n", cases.size());
  for (std::size_t v = 0; v < variants.size(); v++) {
    const bool ok = v == 0 || meetsLimits(tracks, table, v, false);
    printOutcome(variants[v].label.c_str(), total(table[v]), reference);
    if (v > 0) std::printf("%s", ok ? "" : "      (no cumple en alguna pista/modelo)\n");
  }
  if (detail == 0 || detail >= variants.size()) return 0;
  std::printf("\nPor pista y modelo (%s -> %s):\n", variants[0].label.c_str(), variants[detail].label.c_str());
  const bool ok = meetsLimits(tracks, table, detail, true);
  std::printf("\n%s\n", ok ? "Cumple por pista y modelo (sin salidas extra, mapa <= +3 %)."
                           : "No cumple en todas las pistas y modelos.");
  return ok ? 0 : 2;
}

int testDerivative(int seedsPerTrack, bool predictor) {
  if (!atlas::neural::kDerivativeTrained) {
    std::printf("NeuralDerivativeWeights.h no tiene pesos entrenados.\n");
    return 1;
  }
  const std::vector<TrackEntry> tracks = buildTrackLibrary();
  auto blend = [predictor](uint8_t percent) {
    return Variant{"D red " + std::to_string(percent) + " %", [percent, predictor](atlas::RobotConfig& c) {
                     c.control.neuralDerivative = true;
                     c.control.derivativeBlend = percent;
                     c.control.latencyCompensation = predictor;
                   }};
  };
  const std::vector<Variant> variants = {
      {"D clásico", [predictor](atlas::RobotConfig& c) { c.control.latencyCompensation = predictor; }}, blend(15), blend(25), blend(40), blend(60), blend(100)};
  return compare(tracks, makeCases(tracks, 777000, seedsPerTrack, true), variants, 0);
}

int testPredictor(int seedsPerTrack) {
  const std::vector<TrackEntry> tracks = buildTrackLibrary();
  auto predictor = [](uint8_t shift, int8_t gain) {
    char label[48];
    std::snprintf(label, sizeof(label), "predictor 2^%d gan %+d", shift, gain);
    return Variant{label, [shift, gain](atlas::RobotConfig& c) {
                     c.control.latencyCompensation = true;
                     c.control.predictorShift = shift;
                     c.control.predictorGain = gain;
                   }};
  };
  std::vector<Variant> grid = {{"sin predictor", nullptr}};
  for (uint8_t shift : {5, 6, 7, 8, 9})
    for (int8_t gain : {-2, -3, -4, -5, -6}) grid.push_back(predictor(shift, gain));
  const std::vector<Case> search = makeCases(tracks, 555000, seedsPerTrack, true);
  std::printf("Búsqueda: %zu variantes\n", grid.size());
  std::fflush(stdout);
  const Table table = runVariants(tracks, search, grid);
  const Outcome reference = total(table[0]);
  printOutcome(grid[0].label.c_str(), reference, reference);
  std::vector<std::size_t> passing;
  for (std::size_t v = 1; v < grid.size(); v++) {
    const Outcome o = total(table[v]);
    const bool ok = meetsLimits(tracks, table, v, false);
    printOutcome(grid[v].label.c_str(), o, reference);
    if (ok && o.meanZigzag() < reference.meanZigzag()) passing.push_back(v);
  }
  std::fflush(stdout);
  std::sort(passing.begin(), passing.end(),
            [&](std::size_t a, std::size_t b) { return total(table[a]).meanZigzag() < total(table[b]).meanZigzag(); });
  if (passing.empty()) {
    std::printf("\nNinguna variante baja el zigzag cumpliendo los límites.\n");
    return 2;
  }
  if (passing.size() > 3) passing.resize(3);
  std::vector<Variant> finalists = {grid[0]};
  for (std::size_t v : passing) finalists.push_back(grid[v]);
  std::printf("\nConfirmación con semillas nuevas:\n");
  return compare(tracks, makeCases(tracks, 888000, seedsPerTrack * 2, true), finalists, 1);
}

}

}

int main(int argc, char** argv) {
  const std::string mode = argc > 1 ? argv[1] : "";
  if (mode == "posicion") return trainPosition(argc > 2 ? std::strtoul(argv[2], nullptr, 10) : 25000);
  if (mode == "ganancias") return gains::sweepGains();
  if (mode == "predictor") return gains::testPredictor(argc > 2 ? std::atoi(argv[2]) : 2);
  if (mode == "derivada-prueba") return gains::testDerivative(argc > 2 ? std::atoi(argv[2]) : 3,
                                                               !(argc > 3 && std::string(argv[3]) == "sin-predictor"));
  if (mode == "derivada") {
    const std::size_t samples = argc > 2 ? std::strtoul(argv[2], nullptr, 10) : 400000;
    const int lead = argc > 3 ? std::atoi(argv[3]) : 0;
    const bool write = argc > 4 && std::string(argv[4]) == "escribir";
    return derivative::train(samples, lead, write);
  }
  std::fprintf(stderr, "Uso: atlas_train_nn posicion [lecturas] | ganancias | derivada [ciclos] [adelanto] [escribir]\n");
  return 1;
}
