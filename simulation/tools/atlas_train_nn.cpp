/*
  atlas_train_nn.cpp - Entrena las redes neuronales del firmware en el simulador.

  posicion: red supervisada que estima la posición fina de la línea con los 7
    sensores alrededor del grupo seguido (lecturas sin umbral). Los datos salen
    de correr el firmware real en todas las pistas, con 3 modelos de robot,
    varias velocidades y sensores variados (tamaño del punto de lectura,
    ganancia, contraste, ruido, luz). La etiqueta es el cruce verdadero de la
    línea con el eje de la barra, que el simulador conoce exactamente.
    Entrena en double (Adam), cuantiza a enteros como los usa TinyMlp, compara
    con el promedio ponderado y escribe lib/AtlasCore/src/NeuralPositionWeights.h.

  control: red que ajusta en marcha el % de kp y kd (70-130 %) según el error,
    su cambio, la velocidad y la curva del mapa. Se entrena por evolución
    (estrategia evolutiva con muestreo antitético y Adam): cada candidato corre
    los 3 intentos en las pistas de competencia, práctica, Gran Premio y
    Hexágono con 3 modelos de robot. Aptitud = tiempo + error p95 + zigzag,
    con fuerte castigo por salirse. Escribe NeuralGainWeights.h.

  Uso: atlas_train_nn posicion [épocas]
       atlas_train_nn control [generaciones]
*/
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "LineSensorArray.h"
#include "NeuralGainWeights.h"
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
  float target;     // posición verdadera - centro de la ventana, en sensores
  float weighted;   // promedio ponderado del firmware - centro, en sensores
  bool validation;  // corridas separadas para medir sin trampa
};

//  ---------------------------------------------------------------------------
//  Datos
//  ---------------------------------------------------------------------------

struct Cluster {
  int first = -1;
  int last = -1;
};

// Mismo criterio que el firmware: grupo contiguo de sensores sobre umbral
// más cercano a la posición que reportó el seguidor.
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

// Cruces de la línea (su eje central) con el eje de la barra, en unidades de
// posición del firmware (sensor i = i·1000). Vacío si no hay cruce limpio.
std::vector<double> lineCrossings(const Track& track, const RobotModel& robot) {
  const Vec2 left = robot.sensorPosition(0, atlas::kSensorCount);
  const Vec2 right = robot.sensorPosition(atlas::kSensorCount - 1, atlas::kSensorCount);
  const Vec2 axis = (right - left) * (1.0 / (atlas::kSensorCount - 1));  // un sensor
  std::vector<double> crossings;
  const double step = 0.02;  // sensores (0.18 mm)
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
    if (crossings.size() != 1) return;  // cruces, picos o fin de línea: sin etiqueta limpia
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
  spec.impellerPwm = 150;
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

//  ---------------------------------------------------------------------------
//  Red en double (entrenamiento)
//  ---------------------------------------------------------------------------

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

FloatNet train(const std::vector<Sample>& data, int epochs) {
  std::mt19937 random(7);
  FloatNet net;
  std::normal_distribution<double> init(0.0, 1.0);
  for (int j = 0; j < kHidden; j++) {
    for (int i = 0; i < kInputs; i++) net.w1[j][i] = init(random) * std::sqrt(2.0 / kInputs);
    net.b1[j] = 0.1;
    net.w2[j] = init(random) * std::sqrt(1.0 / kHidden);
  }
  net.b2 = 0;

  std::vector<std::size_t> order;
  for (std::size_t i = 0; i < data.size(); i++)
    if (!data[i].validation) order.push_back(i);

  std::vector<double> m(kParams, 0), v(kParams, 0), grad(kParams);
  const double beta1 = 0.9, beta2 = 0.999;
  long step = 0;
  const std::size_t batch = 256;
  for (int epoch = 0; epoch < epochs; epoch++) {
    std::shuffle(order.begin(), order.end(), random);
    const double rate = 3e-3 * std::pow(0.1, static_cast<double>(epoch) / epochs);
    double loss = 0;
    for (std::size_t start = 0; start < order.size(); start += batch) {
      std::fill(grad.begin(), grad.end(), 0.0);
      const std::size_t end = std::min(order.size(), start + batch);
      for (std::size_t b = start; b < end; b++) {
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
      const double scale = 1.0 / static_cast<double>(end - start);
      for (int k = 0; k < kParams; k++) {
        const double g = grad[k] * scale;
        m[k] = beta1 * m[k] + (1 - beta1) * g;
        v[k] = beta2 * v[k] + (1 - beta2) * g * g;
        const double mHat = m[k] / (1 - std::pow(beta1, step));
        const double vHat = v[k] / (1 - std::pow(beta2, step));
        *param(net, k) -= rate * mHat / (std::sqrt(vHat) + 1e-8);
      }
    }
    if (epoch % 10 == 0 || epoch == epochs - 1) {
      std::printf("  época %3d  RMS entrenamiento %.4f sensores\n", epoch, std::sqrt(loss / order.size()));
    }
  }
  return net;
}

//  ---------------------------------------------------------------------------
//  Cuantización (mismo formato que TinyMlp)
//  ---------------------------------------------------------------------------

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
  // shift1: la capa oculta debe caber en int16 con margen.
  double maxHidden = 1e-9;
  for (const Sample& s : data) {
    double hidden[kHidden];
    net.forward(s.input, hidden);
    for (double h : hidden) maxHidden = std::max(maxHidden, h);
  }
  const double accScale = scaleW1 * 127.0;  // acumulador = h · accScale
  q.shift1 = 0;
  while (maxHidden * accScale / std::pow(2.0, q.shift1) > 12000.0) q.shift1++;
  const double hiddenScale = accScale / std::pow(2.0, q.shift1);

  // shift2: salida = posición en unidades del firmware (1000 por sensor).
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

//  ---------------------------------------------------------------------------
//  Evaluación y exportación
//  ---------------------------------------------------------------------------

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
      << "  NeuralPositionWeights.h - GENERADO por simulation/tools/atlas_train_nn.cpp\n"
      << "  (atlas_train_nn posicion). No editar a mano.\n\n"
      << "  Red " << kInputs << " -> " << kHidden << " ReLU -> 1: posición fina de la línea.\n"
      << "  Entrenada el " << date << " con " << samples << " lecturas simuladas.\n"
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

int trainPosition(int epochs) {
  const std::vector<TrackEntry> tracks = buildTrackLibrary();
  const std::vector<Sample> data = buildDataset(tracks);
  std::size_t validation = 0;
  for (const Sample& s : data) validation += s.validation;
  std::printf("Entrenamiento %zu, validación %zu\n", data.size() - validation, validation);

  const FloatNet net = train(data, epochs);
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

  const std::string path = std::string(ATLAS_SOURCE_DIR) + "/lib/AtlasCore/src/NeuralPositionWeights.h";
  writeHeader(path, q, val, data.size());
  std::printf("Pesos escritos en %s\n", path.c_str());
  return 0;
}

//  ---------------------------------------------------------------------------
//  Red de control (neuroevolución)
//  ---------------------------------------------------------------------------

namespace gains {

constexpr int kIn = atlas::neural::kGainInputs;
constexpr int kHid = atlas::neural::kGainHidden;
constexpr int kOut = atlas::neural::kGainOutputs;
constexpr int kParams = kHid * kIn + kHid + kOut * kHid + kOut;

// Parámetros reales del optimizador -> enteros que usa TinyMlp.
struct Net {
  int8_t w1[kHid * kIn];
  int32_t b1[kHid];
  int16_t w2[kOut * kHid];
  int32_t b2[kOut];
  atlas::TinyMlpWeights view() const {
    return {static_cast<uint8_t>(kIn), static_cast<uint8_t>(kHid), static_cast<uint8_t>(kOut), w1, b1,
            atlas::neural::kGainShift1, w2, b2, atlas::neural::kGainShift2};
  }
};

Net decode(const std::vector<double>& theta) {
  Net n;
  int k = 0;
  for (int i = 0; i < kHid * kIn; i++) n.w1[i] = static_cast<int8_t>(std::max(-127L, std::min(127L, std::lround(theta[k++] * 24))));
  for (int i = 0; i < kHid; i++) n.b1[i] = static_cast<int32_t>(std::lround(theta[k++] * 3000));
  for (int i = 0; i < kOut * kHid; i++) n.w2[i] = static_cast<int16_t>(std::max(-32767L, std::min(32767L, std::lround(theta[k++] * 24))));
  for (int i = 0; i < kOut; i++) n.b2[i] = (100L << atlas::neural::kGainShift2) + static_cast<int32_t>(std::lround(theta[k++] * 2560));
  return n;
}

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

struct Score {
  double cost = 0;
  int failures = 0;
  double time = 0, p95 = 0, zigzag = 0;
  int laps = 0;
};

// Los 3 intentos de una competencia con la configuración de src/main.cpp.
Score evaluateCase(const std::vector<TrackEntry>& tracks, const Case& c, const atlas::TinyMlpWeights* weights) {
  atlas::RobotConfig safe = atlas::safeProfile(), fast = atlas::fastProfile();
  atlas::useNeuralNetworks(safe, true);
  atlas::useNeuralNetworks(fast, true);
  if (weights == nullptr) {  // red de control neutra (100 %): solo la red de posición
    safe.control.neuralGains = false;
    fast.control.neuralGains = false;
  }
  safe.control.gainNetwork = weights;
  fast.control.gainNetwork = weights;
  ScenarioSpec spec;
  spec.trackIndex = c.track;
  spec.cruisePwm = 170;
  spec.seed = c.seed;
  spec.learnTrack = true;
  spec.attempts = 3;
  spec.impellerPwm = 190;
  spec.fastImpellerPwm = 235;
  const std::vector<ScenarioResult> attempts =
      ScenarioRunner(robotModel(c.model), SensorModel(), safe, fast).runAttempts(tracks[c.track], spec);
  Score s;
  const double weight = c.model == 2 ? 1.5 : 1.0;  // el robot "real" pesa más
  for (const ScenarioResult& r : attempts) {
    if (!r.passed()) {
      s.failures++;
      s.cost += 300.0 * weight;
      continue;
    }
    s.cost += weight * (r.raceTime + 0.04 * r.p95OffsetMm + 0.3 * r.zigzagPerMeter);
    s.time += r.raceTime;
    s.p95 += r.p95OffsetMm;
    s.zigzag += r.zigzagPerMeter;
    s.laps++;
  }
  return s;
}

// Evalúa varios candidatos sobre los mismos casos, en paralelo.
std::vector<Score> evaluateAll(const std::vector<TrackEntry>& tracks, const std::vector<Case>& cases,
                               const std::vector<const atlas::TinyMlpWeights*>& candidates) {
  std::vector<Score> totals(candidates.size());
  std::vector<std::vector<Score>> parts(candidates.size(), std::vector<Score>(cases.size()));
  std::atomic<std::size_t> next{0};
  const std::size_t jobs = candidates.size() * cases.size();
  auto worker = [&] {
    for (std::size_t j = next++; j < jobs; j = next++) {
      parts[j / cases.size()][j % cases.size()] = evaluateCase(tracks, cases[j % cases.size()], candidates[j / cases.size()]);
    }
  };
  std::vector<std::thread> pool;
  for (unsigned k = 0; k < std::max(2u, std::thread::hardware_concurrency()); k++) pool.emplace_back(worker);
  for (std::thread& t : pool) t.join();
  for (std::size_t i = 0; i < candidates.size(); i++) {
    for (const Score& p : parts[i]) {
      totals[i].cost += p.cost;
      totals[i].failures += p.failures;
      totals[i].time += p.time;
      totals[i].p95 += p.p95;
      totals[i].zigzag += p.zigzag;
      totals[i].laps += p.laps;
    }
  }
  return totals;
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

void printScore(const char* label, const Score& s) {
  const double laps = std::max(1, s.laps);
  std::printf("  %-22s costo %8.1f  salidas %2d  tiempo medio %.2fs  p95 %.1fmm  zigzag %.2f/m\n", label, s.cost,
              s.failures, s.time / laps, s.p95 / laps, s.zigzag / laps);
}

template <typename T>
std::string list(const T* values, int count, int perLine) {
  return join(values, count, perLine);
}

void writeGainHeader(const std::string& path, const Net& n, const Score& neutral, const Score& trained, int cases) {
  char date[32];
  const std::time_t now = std::time(nullptr);
  std::strftime(date, sizeof(date), "%Y-%m-%d", std::localtime(&now));
  const double nl = std::max(1, neutral.laps), tl = std::max(1, trained.laps);
  std::ofstream out(path);
  out << "/*\n"
      << "  NeuralGainWeights.h - GENERADO por simulation/tools/atlas_train_nn.cpp\n"
      << "  (atlas_train_nn control). No editar a mano.\n\n"
      << "  Red " << kIn << " -> " << kHid << " ReLU -> " << kOut << ": % de kp y % de kd en marcha.\n"
      << "  Entrenada el " << date << " por evolución en el simulador.\n"
      << "  Validación (" << cases << " competencias de 3 intentos, semillas nuevas):\n"
      << "    sin red de control: salidas " << neutral.failures << ", tiempo medio " << std::lround(neutral.time / nl * 100) / 100.0
      << " s, p95 " << std::lround(neutral.p95 / nl) << " mm, zigzag " << std::lround(neutral.zigzag / nl * 100) / 100.0 << "/m\n"
      << "    con red de control: salidas " << trained.failures << ", tiempo medio " << std::lround(trained.time / tl * 100) / 100.0
      << " s, p95 " << std::lround(trained.p95 / tl) << " mm, zigzag " << std::lround(trained.zigzag / tl * 100) / 100.0 << "/m\n"
      << "*/\n"
      << "#pragma once\n\n#include <stdint.h>\n\n#include \"Progmem.h\"\n#include \"TinyMlp.h\"\n\n"
      << "namespace atlas {\nnamespace neural {\n\n"
      << "constexpr uint8_t kGainInputs = " << kIn << ";\n"
      << "constexpr uint8_t kGainHidden = " << kHid << ";\n"
      << "constexpr uint8_t kGainOutputs = " << kOut << ";  // % de kp, % de kd\n"
      << "constexpr uint8_t kGainShift1 = " << int(atlas::neural::kGainShift1) << ";\n"
      << "constexpr uint8_t kGainShift2 = " << int(atlas::neural::kGainShift2) << ";\n\n"
      << "ATLAS_PROGMEM const int8_t kGainWeights1[kGainHidden * kGainInputs] = {" << list(n.w1, kHid * kIn, kIn) << "};\n"
      << "ATLAS_PROGMEM const int32_t kGainBias1[kGainHidden] = {" << list(n.b1, kHid, 4) << "};\n"
      << "ATLAS_PROGMEM const int16_t kGainWeights2[kGainOutputs * kGainHidden] = {" << list(n.w2, kOut * kHid, kHid) << "};\n"
      << "ATLAS_PROGMEM const int32_t kGainBias2[kGainOutputs] = {" << list(n.b2, kOut, kOut) << "};\n\n"
      << "const TinyMlpWeights kGainNet = {kGainInputs, kGainHidden, kGainOutputs,\n"
      << "                                 kGainWeights1, kGainBias1, kGainShift1,\n"
      << "                                 kGainWeights2, kGainBias2, kGainShift2};\n\n"
      << "}  // namespace neural\n}  // namespace atlas\n";
}

int train(int generations) {
  const std::vector<TrackEntry> tracks = buildTrackLibrary();
  std::mt19937 random(11);
  std::normal_distribution<double> gauss(0.0, 1.0);

  std::vector<double> theta(kParams, 0.0);
  for (int i = 0; i < kHid * kIn; i++) theta[i] = gauss(random) * 0.7;  // capa oculta activa
  for (int i = 0; i < kHid; i++) theta[kHid * kIn + i] = 0.2;           // salida en cero: empieza neutra

  const int pairs = 8;
  const double sigma = 0.35, rate = 0.06;
  std::vector<double> m(kParams, 0), v(kParams, 0);
  for (int g = 0; g < generations; g++) {
    const std::vector<Case> cases = makeCases(tracks, 20000 + g * 7919, 1, false);
    std::vector<std::vector<double>> noise(pairs, std::vector<double>(kParams));
    std::vector<Net> nets;
    for (int p = 0; p < pairs; p++) {
      for (double& n : noise[p]) n = gauss(random);
      for (int sign : {1, -1}) {
        std::vector<double> candidate = theta;
        for (int k = 0; k < kParams; k++) candidate[k] += sign * sigma * noise[p][k];
        nets.push_back(decode(candidate));
      }
    }
    nets.push_back(decode(theta));
    std::vector<atlas::TinyMlpWeights> views;
    for (const Net& n : nets) views.push_back(n.view());
    std::vector<const atlas::TinyMlpWeights*> candidates;
    for (const atlas::TinyMlpWeights& w : views) candidates.push_back(&w);
    candidates.push_back(nullptr);  // referencia: sin red de control
    const std::vector<Score> scores = evaluateAll(tracks, cases, candidates);

    // Rangos centrados (robusto al ruido) y gradiente antitético.
    std::vector<int> order(2 * pairs);
    for (int i = 0; i < 2 * pairs; i++) order[i] = i;
    std::sort(order.begin(), order.end(), [&](int a, int b) { return scores[a].cost < scores[b].cost; });
    std::vector<double> utility(2 * pairs);
    for (int r = 0; r < 2 * pairs; r++) utility[order[r]] = 0.5 - static_cast<double>(r) / (2 * pairs - 1);
    std::vector<double> grad(kParams, 0.0);
    for (int p = 0; p < pairs; p++) {
      const double diff = utility[2 * p] - utility[2 * p + 1];
      for (int k = 0; k < kParams; k++) grad[k] += diff * noise[p][k] / (pairs * sigma);
    }
    for (int k = 0; k < kParams; k++) {
      m[k] = 0.9 * m[k] + 0.1 * grad[k];
      v[k] = 0.999 * v[k] + 0.001 * grad[k] * grad[k];
      const double mHat = m[k] / (1 - std::pow(0.9, g + 1)), vHat = v[k] / (1 - std::pow(0.999, g + 1));
      theta[k] += rate * mHat / (std::sqrt(vHat) + 1e-8);
    }
    const Score& center = scores[2 * pairs];
    const Score& reference = scores[2 * pairs + 1];
    std::printf("gen %2d  centro: costo %.1f (salidas %d)  sin red de control: %.1f (salidas %d)  mejor: %.1f\n", g,
                center.cost, center.failures, reference.cost, reference.failures, scores[order[0]].cost);
    std::fflush(stdout);
  }

  // Validación con semillas nuevas en todas las pistas.
  const Net best = decode(theta);
  const atlas::TinyMlpWeights bestView = best.view();
  const std::vector<Case> validation = makeCases(tracks, 90000, 3, true);
  const std::vector<Score> result = evaluateAll(tracks, validation, {nullptr, &bestView});
  std::printf("\nValidación: %zu competencias de 3 intentos (todas las pistas, 3 modelos, semillas nuevas)\n",
              validation.size());
  printScore("sin red de control", result[0]);
  printScore("con red de control", result[1]);

  const std::string path = std::string(ATLAS_SOURCE_DIR) + "/lib/AtlasCore/src/NeuralGainWeights.h";
  writeGainHeader(path, best, result[0], result[1], static_cast<int>(validation.size()));
  std::printf("Pesos escritos en %s\n", path.c_str());
  return 0;
}

}  // namespace gains

}  // namespace

int main(int argc, char** argv) {
  const std::string mode = argc > 1 ? argv[1] : "";
  if (mode == "posicion") return trainPosition(argc > 2 ? std::atoi(argv[2]) : 60);
  if (mode == "control") return gains::train(argc > 2 ? std::atoi(argv[2]) : 40);
  std::fprintf(stderr, "Uso: atlas_train_nn posicion [épocas] | control [generaciones]\n");
  return 1;
}
