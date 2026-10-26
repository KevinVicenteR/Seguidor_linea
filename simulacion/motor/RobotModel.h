#pragma once

#include <deque>

#include "Geometry.h"

namespace sim {

struct RobotParams {
  double wheelBase = 0.120;
  double sensorBarAhead = 0.180;
  double sensorPitch = 0.009;

  double maxWheelSpeed = 3.2;
  double motorTimeConstant = 0.035;
  double coastTimeConstant = 0.300;

  double commandLatencyS = 0.0;
  double yawTimeConstant = 0.0;
  int motorDeadbandPwm = 0;
  int suctionDeadbandPwm = 45;
  double rightMotorGain = 1.0;

  double mass = 0.115;
  double tireFriction = 1.0;
  double impellerDownforce = 2.5;
  double impellerReferencePwm = 200;
  double impellerTimeConstant = 0.120;
};

struct DriveInputs {
  int leftPwm = 0;
  int rightPwm = 0;
  int impellerPwm = 0;
  bool driversEnabled = false;
};

class RobotModel {
 public:
  static constexpr double kGravity = 9.81;

  explicit RobotModel(const RobotParams& params) : params_(params) {}

  void place(const Pose& pose);
  void setHeld(bool held) { held_ = held; }
  bool isHeld() const { return held_; }

  void step(double dt, const DriveInputs& inputs);

  const Pose& pose() const { return pose_; }
  double speed() const { return speed_; }
  double yawRate() const { return yawRate_; }
  bool isSliding() const { return sliding_; }
  double gripLimit() const;

  Vec2 sensorPosition(int index, int sensorCount) const;
  Vec2 sensorBarCenter() const { return pose_.toWorld(params_.sensorBarAhead, 0.0); }
  const RobotParams& params() const { return params_; }

 private:
  double wheelTarget(int pwm, bool enabled, double gain) const;
  int effectiveDeadband() const;
  DriveInputs delayed(const DriveInputs& inputs, double dt);

  RobotParams params_;
  Pose pose_;
  bool held_ = true;
  double leftWheel_ = 0.0;
  double rightWheel_ = 0.0;
  double speed_ = 0.0;
  double yawRate_ = 0.0;
  double downforce_ = 0.0;
  bool sliding_ = false;
  std::deque<DriveInputs> commandQueue_;
};

}
