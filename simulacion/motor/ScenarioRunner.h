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

using ControlObserver = std::function<void(double timeS, const atlas::LineSensorArray& sensors,
                                           const atlas::LineFollower& follower, const RobotModel& robot)>;

class ScenarioRunner {
 public:
  explicit ScenarioRunner(RobotParams robot = RobotParams(), SensorModel sensors = SensorModel(),
                          atlas::RobotConfig firmware = atlas::safeProfile(),
                          atlas::RobotConfig fastFirmware = atlas::fastProfile())
      : robotParams_(robot), sensorModel_(sensors), firmware_(firmware), fastFirmware_(fastFirmware) {}

  ScenarioResult run(const TrackEntry& entry, const ScenarioSpec& spec) const;
  std::vector<ScenarioResult> runAttempts(const TrackEntry& entry, const ScenarioSpec& spec) const;
  void setObserver(ControlObserver observer) { observer_ = std::move(observer); }

 private:
  RobotParams robotParams_;
  SensorModel sensorModel_;
  atlas::RobotConfig firmware_;
  atlas::RobotConfig fastFirmware_;
  ControlObserver observer_;
};

}
