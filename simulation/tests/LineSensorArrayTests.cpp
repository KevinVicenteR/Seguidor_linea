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

// Las lecturas se invierten (línea negra): ADC 30 -> 225 y ADC 220 -> 35.
TEST("Calibración: umbral al 50% entre mínimo y máximo") {
  test::FakeSensorBus bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  calibrate(sensors, bus);
  for (uint8_t i = 0; i < kSensorCount; i++) {
    EXPECT_EQ(sensors.minValues()[i], 35);
    EXPECT_EQ(sensors.maxValues()[i], 225);
    EXPECT_EQ(sensors.thresholds()[i], 130);
  }
}

TEST("Calibración ignora un pico aislado de ruido") {
  test::FakeSensorBus bus;
  SensorConfig config;
  config.collectorSamples = 2;
  config.averageSamples = 1;
  // Un solo pico no basta: el máximo solo sube si todas las muestras lo superan.
  struct SpikeBus : public ILineSensorBus {
    int reads = 0;
    uint8_t read(uint8_t) override { return reads++ < kSensorCount ? 0 : 220; }
  } spike;
  LineSensorArray spiky(spike, config);
  spiky.resetCalibration();
  spiky.collectCalibrationSamples();
  EXPECT_EQ(spiky.maxValues()[0], 35);
}

TEST("Sensor sin calibrar nunca reporta línea") {
  test::FakeSensorBus bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  sensors.resetCalibration();
  sensors.finishCalibration();
  bus.fill(0);
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

TEST("Solo línea negra: la configuración por defecto invierte las lecturas") {
  test::FakeSensorBus bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  EXPECT_TRUE(sensors.lineColor() == LineColor::Black);
  calibrate(sensors, bus);
  bus.lineAt(11, 12);  // ADC bajo en 11-12 = línea negra
  EXPECT_EQ(sensors.readLine().position, 11500);
}

TEST("Sin línea bajo la barra") {
  test::FakeSensorBus bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  calibrate(sensors, bus);
  bus.fill(220);  // todo fondo blanco
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
  bus.values[13] = bus.values[14] = 30;  // segunda línea a la derecha
  EXPECT_EQ(sensors.readLine().position, 6500);
}

TEST("Cruce: un grupo demasiado ancho mantiene la última posición") {
  test::FakeSensorBus bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  calibrate(sensors, bus);
  for (int i = 0; i < 60; i++) {  // sigue la línea un rato (fuera de la ventana de recuperación)
    bus.lineAt(7, 8);
    sensors.readLine();
  }
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
  bus.values[12] = bus.values[13] = 30;
  EXPECT_EQ(sensors.readLine().position, 7500);
}

TEST("Cruce: la rama que se une al grupo no desvía la posición") {
  test::FakeSensorBus bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  calibrate(sensors, bus);
  for (int i = 0; i < 40; i++) {  // aprende la forma normal de la línea (2 sensores)
    bus.lineAt(7, 8);
    sensors.readLine();
  }
  bus.lineAt(7, 8);
  bus.values[11] = bus.values[12] = 30;  // se acerca otra línea por la derecha
  EXPECT_EQ(sensors.readLine().position, kLinePositionCenter);
  bus.lineAt(7, 9);  // se une: 3 sensores, la posición cruda saltaría a 8000
  const LineReading merged = sensors.readLine();
  EXPECT_TRUE(merged.ambiguous);
  EXPECT_EQ(merged.position, kLinePositionCenter);
  bus.lineAt(7, 8);
  bus.values[4] = bus.values[5] = 30;  // la rama sale por la izquierda
  EXPECT_EQ(sensors.readLine().position, kLinePositionCenter);
}

TEST("Sensor dañado: se acepta con 15 sensores y toma el promedio de sus vecinos") {
  struct DeadSensorBus : public ILineSensorBus {
    test::FakeSensorBus inner;
    uint8_t read(uint8_t channel) override { return channel == 8 ? 220 : inner.read(channel); }
  } bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  sensors.resetCalibration();
  bus.inner.fill(30);
  sensors.collectCalibrationSamples();
  bus.inner.fill(220);
  sensors.collectCalibrationSamples();
  sensors.finishCalibration();
  EXPECT_EQ(sensors.calibratedSensorCount(), 15);
  EXPECT_TRUE(sensors.isCalibrationValid());

  bus.inner.lineAt(7, 9);  // la línea pasa justo sobre el sensor dañado
  EXPECT_EQ(sensors.readLine().position, 8000);
}

TEST("Al recuperar la línea no se filtran saltos (la barra la cruza rápido)") {
  test::FakeSensorBus bus;
  SensorConfig config;
  LineSensorArray sensors(bus, config);
  calibrate(sensors, bus);
  for (int i = 0; i < 60; i++) {
    bus.lineAt(7, 8);
    sensors.readLine();
  }
  bus.fill(220);  // se perdió la línea
  sensors.readLine();
  bus.lineAt(13, 14);  // la vuelve a ver en un extremo...
  EXPECT_EQ(sensors.readLine().position, 13500);
  bus.lineAt(9, 10);  // ...y la barra la cruza rápido: salto de 4 sensores
  const LineReading reading = sensors.readLine();
  EXPECT_FALSE(reading.ambiguous);
  EXPECT_EQ(reading.position, 9500);
}
