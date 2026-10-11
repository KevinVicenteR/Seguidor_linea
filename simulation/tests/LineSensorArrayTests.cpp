#include "Fakes.h"
#include "LineSensorArray.h"
#include "TestFramework.h"

using namespace atlas;

namespace {

// Calibra pasando la línea por todos los sensores.
void calibrate(LineSensorArray& sensors, test::FakeSensorBus& bus) {
  sensors.resetCalibration();
  bus.fill(30);
  sensors.collectCalibrationSamples();
  bus.fill(220);
  sensors.collectCalibrationSamples();
  sensors.finishCalibration();
}

}  // namespace

TEST("Calibración: umbral al 50% entre mínimo y máximo") {
  test::FakeSensorBus bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  calibrate(sensors, bus);
  for (uint8_t i = 0; i < kSensorCount; i++) {
    EXPECT_EQ(sensors.minValues()[i], 30);
    EXPECT_EQ(sensors.maxValues()[i], 220);
    EXPECT_EQ(sensors.thresholds()[i], 125);
  }
}

TEST("Calibración ignora un pico aislado de ruido") {
  test::FakeSensorBus bus;
  SensorConfig config;
  config.collectorSamples = 2;
  config.averageSamples = 1;
  LineSensorArray sensors(bus, config);
  sensors.resetCalibration();
  bus.fill(30);
  sensors.collectCalibrationSamples();
  // Un solo pico no basta: el máximo solo sube si todas las muestras lo superan.
  struct SpikeBus : public ILineSensorBus {
    int reads = 0;
    uint8_t read(uint8_t) override { return reads++ < kSensorCount ? 250 : 30; }
  } spike;
  LineSensorArray spiky(spike, config);
  spiky.resetCalibration();
  spiky.collectCalibrationSamples();
  EXPECT_EQ(spiky.maxValues()[0], 30);
}

TEST("Sensor sin calibrar nunca reporta línea") {
  test::FakeSensorBus bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  sensors.resetCalibration();
  sensors.finishCalibration();
  bus.fill(255);
  EXPECT_FALSE(sensors.readLine().onLine);
}

TEST("Posición centrada con la línea bajo los sensores 7 y 8") {
  test::FakeSensorBus bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  calibrate(sensors, bus);
  bus.lineAt(7, 8);
  const LineReading reading = sensors.readLine();
  EXPECT_TRUE(reading.onLine);
  EXPECT_EQ(reading.position, kLinePositionCenter);
  EXPECT_TRUE(reading.isTrackable());
}

TEST("Línea en el extremo izquierdo activa el borde y no es seguible") {
  test::FakeSensorBus bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  calibrate(sensors, bus);
  bus.lineAt(0, 0);
  const LineReading reading = sensors.readLine();
  EXPECT_TRUE(reading.leftEdgeDetected);
  EXPECT_FALSE(reading.rightEdgeDetected);
  EXPECT_FALSE(reading.isTrackable());  // posición 0: igual que el firmware original
}

TEST("Línea negra: las lecturas se invierten") {
  test::FakeSensorBus bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  sensors.setLineColor(LineColor::Black);
  calibrate(sensors, bus);
  bus.lineAt(0, 15, 220, 220);
  bus.values[11] = 30;  // línea negra = lectura baja
  bus.values[12] = 30;
  const LineReading reading = sensors.readLine();
  EXPECT_EQ(reading.position, 11500);
}

TEST("Sin línea bajo la barra") {
  test::FakeSensorBus bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  calibrate(sensors, bus);
  bus.fill(30);
  EXPECT_FALSE(sensors.readLine().onLine);
}

TEST("Cruce: elige el grupo de sensores más cercano a la posición anterior") {
  test::FakeSensorBus bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  calibrate(sensors, bus);
  bus.lineAt(6, 7);
  sensors.readLine();
  bus.lineAt(6, 7);
  bus.values[13] = bus.values[14] = 220;  // segunda línea a la derecha
  EXPECT_EQ(sensors.readLine().position, 6500);
}

TEST("Cruce: un grupo demasiado ancho mantiene la última posición") {
  test::FakeSensorBus bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  calibrate(sensors, bus);
  bus.lineAt(7, 8);
  sensors.readLine();
  bus.lineAt(7, 12);  // la línea transversal se une a la principal
  EXPECT_EQ(sensors.readLine().position, kLinePositionCenter);
}

TEST("Algoritmo original (sin seguimiento de grupo) promedia todo") {
  test::FakeSensorBus bus;
  SensorConfig config;
  config.trackNearestCluster = false;
  LineSensorArray sensors(bus, config);
  calibrate(sensors, bus);
  bus.lineAt(2, 3);
  bus.values[12] = bus.values[13] = 220;
  EXPECT_EQ(sensors.readLine().position, 7500);
}
