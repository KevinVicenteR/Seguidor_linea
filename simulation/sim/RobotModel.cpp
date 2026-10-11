#include "RobotModel.h"

#include <algorithm>
#include <cmath>

namespace sim {

constexpr double RobotModel::kGravity;

void RobotModel::place(const Pose& pose) {
  pose_ = pose;
  leftWheel_ = rightWheel_ = speed_ = yawRate_ = 0.0;
  sliding_ = false;
}

double RobotModel::wheelTarget(int pwm, bool enabled) const {
  return enabled ? params_.maxWheelSpeed * pwm / 255.0 : 0.0;
}

double RobotModel::gripLimit() const {
  return params_.tireFriction * (kGravity + downforce_ / params_.mass);
}

void RobotModel::step(double dt, const DriveInputs& inputs) {
  // El impeller (MOSFET) gira aunque los drivers de rueda estén inhibidos.
  const double impellerRatio = inputs.impellerPwm / params_.impellerReferencePwm;
  const double targetDownforce = params_.impellerDownforce * impellerRatio * impellerRatio;
  downforce_ += (targetDownforce - downforce_) * dt / params_.impellerTimeConstant;

  if (held_) return;

  const double tau = inputs.driversEnabled ? params_.motorTimeConstant : params_.coastTimeConstant;
  leftWheel_ += (wheelTarget(inputs.leftPwm, inputs.driversEnabled) - leftWheel_) * dt / tau;
  rightWheel_ += (wheelTarget(inputs.rightPwm, inputs.driversEnabled) - rightWheel_) * dt / tau;

  const double grip = gripLimit();

  // Aceleración longitudinal limitada por adherencia.
  const double desiredSpeed = (leftWheel_ + rightWheel_) / 2.0;
  const double maxDeltaV = grip * dt;
  speed_ += std::max(-maxDeltaV, std::min(maxDeltaV, desiredSpeed - speed_));

  // Giro limitado por adherencia lateral: |v·ω| <= grip.
  const double desiredYaw = (rightWheel_ - leftWheel_) / params_.wheelBase;
  const double maxYaw = std::fabs(speed_) > 1e-3 ? grip / std::fabs(speed_) : 1e9;
  sliding_ = std::fabs(desiredYaw) > maxYaw;
  yawRate_ = std::max(-maxYaw, std::min(maxYaw, desiredYaw));

  pose_.heading = wrapAngle(pose_.heading + yawRate_ * dt);
  pose_.position = pose_.position + pose_.forward() * (speed_ * dt);
}

// Sensor 0 = extremo izquierdo, sensor 15 = extremo derecho.
Vec2 RobotModel::sensorPosition(int index, int sensorCount) const {
  const double lateral = ((sensorCount - 1) / 2.0 - index) * params_.sensorPitch;
  return pose_.toWorld(params_.sensorBarAhead, lateral);
}

}  // namespace sim
