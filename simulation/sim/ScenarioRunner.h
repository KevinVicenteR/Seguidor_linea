/*
  ScenarioRunner.h - Ensambla firmware real (AtlasCore) + hardware simulado
  + física + operador, y ejecuta un escenario completo de principio a fin.
*/
#pragma once

#include <functional>

#include "Config.h"
#include "LineFollower.h"
#include "LineSensorArray.h"
#include "Profiles.h"
#include "RobotModel.h"
#include "Scenario.h"
#include "SimHardware.h"
#include "TrackLibrary.h"

namespace sim {

// Observador opcional para depurar: se llama en cada iteración de loop()
// con el estado interno del firmware.
using ControlObserver = std::function<void(double timeS, const atlas::LineSensorArray& sensors,
                                           const atlas::LineFollower& follower, const RobotModel& robot)>;

class ScenarioRunner {
 public:
  explicit ScenarioRunner(RobotParams robot = RobotParams(), SensorModel sensors = SensorModel(),
                          atlas::RobotConfig firmware = atlas::safeProfile(),
                          atlas::RobotConfig fastFirmware = atlas::fastProfile())
      : robotParams_(robot), sensorModel_(sensors), firmware_(firmware), fastFirmware_(fastFirmware) {}

  ScenarioResult run(const TrackEntry& entry, const ScenarioSpec& spec) const;
  // Corre spec.attempts intentos seguidos sobre el mismo robot (con su EEPROM).
  std::vector<ScenarioResult> runAttempts(const TrackEntry& entry, const ScenarioSpec& spec) const;
  void setObserver(ControlObserver observer) { observer_ = std::move(observer); }

 private:
  RobotParams robotParams_;
  SensorModel sensorModel_;
  atlas::RobotConfig firmware_;      // perfil Seguro (o configuración única)
  atlas::RobotConfig fastFirmware_;  // perfil Rápido para intentos con mapa
  ControlObserver observer_;
};

}  // namespace sim
