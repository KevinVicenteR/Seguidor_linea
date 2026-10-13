#include "Config.h"
#include "Hal.h"
#include "SpeedPlanner.h"
#include "TestFramework.h"
#include "TrackMap.h"
#include "TrackMapStore.h"

using namespace atlas;

namespace {

TrackMap straightWithCurveAt(uint16_t curveBin, uint16_t length) {
  TrackMap map;
  map.startRecording();
  for (uint16_t i = 0; i < length; i++) map.recordCurvature(i, i == curveBin ? 255 : 0, false);
  map.finishRecording(length - 1, 1, 0);
  return map;
}

class ArrayStorage : public IPersistentStorage {
 public:
  ArrayStorage() {
    for (uint8_t& b : bytes) b = 0xFF;
  }
  uint8_t read(uint16_t address) override { return bytes[address]; }
  void write(uint16_t address, uint8_t value) override { bytes[address] = value; }
  uint8_t bytes[1024];
};

}  // namespace

TEST("SpeedPlanner: rectas al máximo y curva cerrada al mínimo de giro") {
  PlannerConfig config;
  SpeedPlanner planner(config);
  EXPECT_EQ(planner.limitFor(0), config.maxPwm);
  EXPECT_EQ(planner.limitFor(255), config.fullTurnPwm);
  EXPECT_TRUE(planner.limitFor(60) < config.maxPwm);
  EXPECT_TRUE(planner.limitFor(60) > config.fullTurnPwm);
}

TEST("SpeedPlanner: frena antes de la curva, no al llegar") {
  PlannerConfig config;
  SpeedPlanner planner(config);
  const TrackMap map = straightWithCurveAt(40, 80);
  EXPECT_EQ(planner.speedAt(map, 5), config.maxPwm);  // la curva está lejos
  EXPECT_EQ(planner.speedAt(map, 30), config.fullTurnPwm + config.brakePwmPerBin * 10);
  EXPECT_EQ(planner.speedAt(map, 40), config.fullTurnPwm);
  EXPECT_EQ(planner.speedAt(map, 45), config.maxPwm);  // ya pasó la curva
}

TEST("SpeedPlanner: fuera del mapa usa la velocidad mínima") {
  PlannerConfig config;
  SpeedPlanner planner(config);
  const TrackMap map = straightWithCurveAt(5, 20);
  EXPECT_EQ(planner.speedAt(map, 200), config.offMapPwm);
}

TEST("TrackMap: guarda la curvatura promedio del tramo y los huecos sin repetir") {
  TrackMap map;
  map.startRecording();
  map.recordCurvature(3, 40, false);
  map.recordCurvature(3, 120, false);
  map.recordCurvature(3, 20, false);
  map.recordCurvature(4, 0, true);  // línea perdida en el tramo 4
  map.recordGap(10);
  map.recordGap(11);  // mismo hueco
  map.recordGap(50);
  map.finishRecording(60, 30, 0);
  EXPECT_TRUE(map.isValid());
  EXPECT_EQ(map.curvature(3), 60);
  EXPECT_EQ(map.curvature(4), 255);
  EXPECT_EQ(map.gapCount(), 2);
  EXPECT_EQ(map.nearestGap(47, 10), 50);
  EXPECT_EQ(map.nearestGap(30, 5), -1);
}

TEST("TrackMap: un reconocimiento demasiado corto no es válido") {
  TrackMap map;
  map.startRecording();
  map.recordCurvature(5, 10, false);
  map.finishRecording(5, 30, 0);
  EXPECT_FALSE(map.isValid());
}

TEST("TrackMapStore: guarda y recupera el mapa; detecta datos dañados") {
  ArrayStorage eeprom;
  TrackMap map = straightWithCurveAt(12, 50);
  map.recordGap(20);
  TrackMapStore(eeprom).save(map);

  TrackMap loaded;
  EXPECT_TRUE(TrackMapStore(eeprom).load(loaded));
  EXPECT_EQ(loaded.length(), 50);
  EXPECT_EQ(loaded.curvature(12), 255);
  EXPECT_EQ(loaded.gapCount(), 1);
  EXPECT_EQ(loaded.gap(0), 20);

  eeprom.bytes[64 + 30] ^= 0x55;  // un byte corrupto
  TrackMap corrupted;
  EXPECT_FALSE(TrackMapStore(eeprom).load(corrupted));
}

TEST("SpeedPlanner: un reconocimiento difícil reduce la velocidad planeada") {
  PlannerConfig config;
  SpeedPlanner planner(config);
  TrackMap easy;
  easy.startRecording();
  easy.finishRecording(80, 1, 20);  // 2 % sin línea
  TrackMap hard;
  hard.startRecording();
  hard.finishRecording(80, 1, 60);  // 6 % sin línea
  EXPECT_EQ(planner.confidencePercent(easy), 100);
  EXPECT_EQ(planner.confidencePercent(hard), 100 - (60 - config.easyLostPerMille) * config.slowdownPerLostPerMille);
  EXPECT_TRUE(planner.speedAt(hard, 5) < planner.speedAt(easy, 5));
}
