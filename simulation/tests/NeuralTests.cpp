#include "Fakes.h"
#include "LineFollower.h"
#include "LineSensorArray.h"
#include "NeuralHierarchy.h"
#include "PdController.h"
#include "TestFramework.h"
#include "TinyMlp.h"

using namespace atlas;

namespace {

void calibrate(LineSensorArray& sensors, test::FakeSensorBus& bus) {
  sensors.resetCalibration();
  bus.fill(30);
  sensors.collectCalibrationSamples();
  bus.fill(220);
  sensors.collectCalibrationSamples();
  sensors.finishCalibration();
}

LineReading readMany(LineSensorArray& sensors, int reads) {
  LineReading reading;
  for (int i = 0; i < reads; i++) reading = sensors.readLine();
  return reading;
}

}  // namespace

TEST("TinyMlp: capa oculta con ReLU y desplazamientos") {
  static const int8_t w1[] = {1, 0, 0, -1};
  static const int32_t b1[] = {0, 0};
  static const int16_t w2[] = {2, 3};
  static const int32_t b2[] = {10};
  const TinyMlpWeights weights = {2, 2, 1, w1, b1, 0, w2, b2, 1};
  const int8_t input[] = {5, 7};
  int32_t output = 0;
  TinyMlp(weights).evaluate(input, &output);
  EXPECT_EQ(output, 10);  // oculta = (5, ReLU(-7) = 0); (10 + 2·5) >> 1
}

TEST("Red de posición: línea centrada entre dos sensores da ~7500") {
  test::FakeSensorBus bus;
  SensorConfig config;
  config.neuralPosition = true;
  LineSensorArray sensors(bus, config);
  calibrate(sensors, bus);
  bus.lineAt(7, 8);
  EXPECT_NEAR(readMany(sensors, 3).position, 7500, 150);
}

TEST("Red de posición: ve el sensor apenas cubierto que el umbral descarta") {
  test::FakeSensorBus bus;
  SensorConfig plain;
  SensorConfig neural;
  neural.neuralPosition = true;
  LineSensorArray classic(bus, plain);
  LineSensorArray network(bus, neural);
  calibrate(classic, bus);
  calibrate(network, bus);
  bus.lineAt(7, 7);
  bus.values[8] = 150;  // cubierto ~35 %: bajo el umbral del 50 %
  const uint16_t weighted = readMany(classic, 3).position;
  const uint16_t refined = readMany(network, 3).position;
  EXPECT_EQ(weighted, 7000);
  EXPECT_TRUE(refined > 7000 && refined < 7500);
}

TEST("Red de posición: en el extremo de la barra usa el promedio ponderado") {
  test::FakeSensorBus bus;
  SensorConfig config;
  config.neuralPosition = true;
  LineSensorArray sensors(bus, config);
  calibrate(sensors, bus);
  bus.lineAt(0, 1);
  EXPECT_EQ(readMany(sensors, 3).position, 500);
}

TEST("Línea en diagonal (4 sensores) sin otras líneas cerca: no es un cruce") {
  test::FakeSensorBus bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  calibrate(sensors, bus);
  bus.lineAt(6, 8);
  readMany(sensors, 60);
  bus.lineAt(6, 9);
  const LineReading reading = sensors.readLine();
  EXPECT_FALSE(reading.ambiguous);
  EXPECT_EQ(reading.position, 7500);
}

TEST("Tras un cruce, una sola línea limpia lejos de la posición retenida se acepta") {
  test::FakeSensorBus bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  calibrate(sensors, bus);
  bus.lineAt(7, 8);
  readMany(sensors, 60);
  bus.lineAt(11, 12);  // salto de 4000: se retiene la posición
  EXPECT_TRUE(sensors.readLine().ambiguous);
  const LineReading settled = readMany(sensors, 25);
  EXPECT_FALSE(settled.ambiguous);
  EXPECT_EQ(settled.position, 11500);
}

TEST("PD con kp y kd separados al mismo % que la versión de un solo %") {
  PdController single(0.02f, 1.5f, 1000, 1);
  PdController split(0.02f, 1.5f, 1000, 1);
  const int32_t errors[] = {0, 800, 1500, -300, -2400, 50};
  for (int32_t e : errors) EXPECT_EQ(split.update(e, 80, 80), single.update(e, 80));
}

namespace {

// Red jerárquica con salida fija: principal con pesos iguales (o solo la
// situación `only`), expertos que dan `percent` (o 100 los demás).
struct ConstantHierarchy {
  int8_t w1[kHierarchyInputs] = {};
  int32_t b1[1] = {};
  int16_t w2[kSituations] = {};
  int32_t gateBias[kSituations];
  int32_t expertBias[kSituations][2];
  NeuralHierarchy net;
  ConstantHierarchy(int32_t percent, int only = -1) : net{} {
    for (int k = 0; k < kSituations; k++) {
      gateBias[k] = (only < 0 || only == k) ? (64 << 8) : 0;
      const int32_t p = (only < 0 || only == k) ? percent : 100;
      expertBias[k][0] = expertBias[k][1] = p << 8;
      net.experts[k] = {kHierarchyInputs, 1, 2, w1, b1, 0, w2, expertBias[k], 8};
    }
    net.gate = {kHierarchyInputs, 1, kSituations, w1, b1, 0, w2, gateBias, 8};
  }
};

int16_t steeringWith(const NeuralHierarchy* network, bool neural) {
  test::FakeSensorBus bus;
  test::FakeDrivetrain drivetrain;
  RobotConfig config;
  config.corner.enabled = false;
  config.speed.startKickMs = 0;
  config.speed.startKickPwm = 0;
  config.control.minMotorPwm = 0;
  config.control.curveSlowdownPercent = 0;
  config.adaptive.autoDamping = false;
  config.control.neuralGains = neural;
  config.control.hierarchy = network;
  LineSensorArray sensors(bus, config.sensors);
  LineFollower follower(sensors, drivetrain, config);
  calibrate(sensors, bus);
  follower.start(150, 0);
  bus.lineAt(7, 8);
  for (uint32_t t = 0; t < 200; t++) follower.update(t);
  bus.lineAt(10, 11);
  follower.update(200);
  return static_cast<int16_t>((drivetrain.leftMotor.pwm - drivetrain.rightMotor.pwm) / 2);
}

}  // namespace

TEST("Red jerárquica: la corrección nunca pasa del 130 % ni baja del 70 %") {
  const int16_t nominal = steeringWith(nullptr, false);
  ConstantHierarchy huge(1000), tiny(-1000), neutral(100);
  EXPECT_NEAR(steeringWith(&neutral.net, true), nominal, 1);
  EXPECT_NEAR(steeringWith(&huge.net, true), nominal * 13 / 10, 2);
  EXPECT_NEAR(steeringWith(&tiny.net, true), nominal * 7 / 10, 2);
}

TEST("Red jerárquica: la principal elige qué subneurona manda") {
  const int8_t input[kHierarchyInputs] = {};
  for (int k = 0; k < kSituations; k++) {
    ConstantHierarchy only(80, k);
    uint8_t kp = 0, kd = 0, weights[kSituations];
    evaluateHierarchy(only.net, input, &kp, &kd, weights);
    EXPECT_EQ(kp, 80);
    EXPECT_EQ(kd, 80);
    for (int j = 0; j < kSituations; j++) EXPECT_EQ(weights[j] > 0, j == k);
  }
  ConstantHierarchy mixed(80);  // las 4 pesan igual y dan 80: mezcla = 80
  uint8_t kp = 0, kd = 0;
  evaluateHierarchy(mixed.net, input, &kp, &kd);
  EXPECT_EQ(kp, 80);
}

// La misma factoría se usa en AVR y en --firmware, también para el control clásico.
#include "FirmwareConfig.h"
TEST("Ajustes de despliegue conservan velocidades y activan las redes entrenadas") {
  for (bool neural : {false, true}) {
    const auto safe = firmwareSafeConfig(neural);
    const auto fast = firmwareFastConfig(neural);
    EXPECT_EQ(safe.speed.basePwm, 170);
    EXPECT_EQ(safe.impeller.pwm, 190);
    EXPECT_EQ(fast.speed.basePwm, 170);
    EXPECT_EQ(fast.impeller.pwm, 235);
    EXPECT_EQ(safe.sensors.neuralPosition, neural);
    EXPECT_FALSE(safe.control.neuralGains);  // la red jerárquica quedó neutra: apagada
    EXPECT_EQ(safe.control.neuralDerivative, neural && neural::kDerivativeTrained && kDeployedDerivativeBlend > 0);
    EXPECT_EQ(fast.sensors.neuralPosition, neural);
    EXPECT_FALSE(fast.control.neuralGains);
  }
}
