/*
  ScenarioRunner.h - Ensambla firmware real (AtlasCore) + hardware simulado
  + física + operador, y ejecuta un escenario completo de principio a fin.
*/
#pragma once

#include "RobotModel.h"
#include "Scenario.h"
#include "SimHardware.h"
#include "TrackLibrary.h"

namespace sim {

class ScenarioRunner {
 public:
  explicit ScenarioRunner(RobotParams robot = RobotParams(), SensorModel sensors = SensorModel())
      : robotParams_(robot), sensorModel_(sensors) {}

  ScenarioResult run(const TrackEntry& entry, const ScenarioSpec& spec) const;

 private:
  RobotParams robotParams_;
  SensorModel sensorModel_;
};

}  // namespace sim
