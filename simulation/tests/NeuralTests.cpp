#include "Fakes.h"
#include "LineSensorArray.h"
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
