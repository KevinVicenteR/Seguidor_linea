#pragma once

#include "Config.h"
#include "RaceMonitor.h"
#include "RobotModel.h"
#include "SimHardware.h"

namespace sim {

struct OperatorPlan {
  int speedPresses = 0;
  Pose calibrationPose;
  Pose startPose;
  double raceTimeoutS = 90.0;
};

class RaceOperator {
 public:
  RaceOperator(SimOperatorPanel& panel, RobotModel& robot, RaceMonitor& monitor, const OperatorPlan& plan);

  void update(double timeS);
  bool hasReleasedGo() const { return phase_ == Phase::Done; }
  void nextAttempt(RaceMonitor& monitor, double timeS);
  RaceMonitor& monitor() { return *monitor_; }

 private:
  enum class Phase {
    WaitBoot,
    PressStartCalibration,
    Sweep,
    PressEndCalibration,
    WaitConfirm,
    AdjustSpeed,
    Reposition,
    Ready,
    Race,
    Done
  };

  static constexpr double kBootS = 1.0;
  static constexpr double kPressS = 0.12;
  static constexpr double kSweepS = 2.4;
  static constexpr double kSweepAmplitude = 0.09;
  static constexpr double kConfirmWaitS = 2.0;
  static constexpr double kSpeedPressPeriodS = 0.8;
  static constexpr double kReadyS = 1.3;
  static constexpr double kRepositionS = 2.0;

  void enter(Phase phase, double timeS);
  void holdAtLateralOffset(double offset);
  void pressSpeedButton(bool pressed);

  SimOperatorPanel& panel_;
  RobotModel& robot_;
  RaceMonitor* monitor_;
  OperatorPlan plan_;
  Phase phase_ = Phase::WaitBoot;
  double phaseStart_ = 0.0;
  int pressesLeft_;
};

}
