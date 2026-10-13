/*
  ScenarioRunner.h - Ensambla firmware real (AtlasCore) + hardware simulado
  + física + operador, y ejecuta un escenario completo de principio a fin.
*/
#pragma once

#include "Config.h"
#include "RobotModel.h"
#include "Scenario.h"
#include "SimHardware.h"
#include "TrackLibrary.h"

namespace sim {

class ScenarioRunner {
 public:
  explicit ScenarioRunner(RobotParams robot = RobotParams(), SensorModel sensors = SensorModel(),
                          atlas::RobotConfig firmware = atlas::RobotConfig())
      : robotParams_(robot), sensorModel_(sensors), firmware_(firmware) {}

  ScenarioResult run(const TrackEntry& entry, const ScenarioSpec& spec) const;

 private:
  RobotParams robotParams_;
  SensorModel sensorModel_;
  atlas::RobotConfig firmware_;  // configuración base del firmware a simular
};

}  // namespace sim
