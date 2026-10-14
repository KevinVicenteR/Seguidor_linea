#include "RobotModel.h"

#include <algorithm>
#include <cmath>

namespace sim {

constexpr double RobotModel::kGravity;

void RobotModel::place(const Pose& pose) {
  pose_ = pose;
  leftWheel_ = rightWheel_ = speed_ = yawRate_ = 0.0;
  sliding_ = false;
  commandQueue_.clear();
}

// Velocidad de rueda en régimen para un PWM, con zona muerta del motor.
int RobotModel::effectiveDeadband() const {
  const double suction = params_.impellerDownforce > 0.0 ? downforce_ / params_.impellerDownforce : 0.0;
  return params_.motorDeadbandPwm + static_cast<int>(params_.suctionDeadbandPwm * suction);
}

double RobotModel::wheelTarget(int pwm, bool enabled, double gain) const {
  if (!enabled) return 0.0;
  const int deadband = effectiveDeadband();
  const int magnitude = pwm < 0 ? -pwm : pwm;
  if (magnitude <= deadband) return 0.0;
  const double effective = 255.0 * (magnitude - deadband) / (255.0 - deadband);
  return (pwm < 0 ? -1.0 : 1.0) * gain * params_.maxWheelSpeed * effective / 255.0;
}

// Retraso de comandos: el motor recibe lo que el firmware ordenó hace
// commandLatencyS segundos.
DriveInputs RobotModel::delayed(const DriveInputs& inputs, double dt) {
  if (params_.commandLatencyS <= 0.0) return inputs;
  commandQueue_.push_back(inputs);
  const std::size_t depth = static_cast<std::size_t>(params_.commandLatencyS / dt) + 1;
  while (commandQueue_.size() > depth) commandQueue_.pop_front();
  return commandQueue_.front();
}

double RobotModel::gripLimit() const {
  return params_.tireFriction * (kGravity + downforce_ / params_.mass);
}

void RobotModel::step(double dt, const DriveInputs& command) {
  const DriveInputs inputs = delayed(command, dt);
  // El impeller (MOSFET) gira aunque los drivers de rueda estén inhibidos.
  const double impellerRatio = inputs.impellerPwm / params_.impellerReferencePwm;
  const double targetDownforce = params_.impellerDownforce * impellerRatio * impellerRatio;
  downforce_ += (targetDownforce - downforce_) * dt / params_.impellerTimeConstant;

  if (held_) return;

  const double tau = inputs.driversEnabled ? params_.motorTimeConstant : params_.coastTimeConstant;
  leftWheel_ += (wheelTarget(inputs.leftPwm, inputs.driversEnabled, 1.0) - leftWheel_) * dt / tau;
  rightWheel_ +=
      (wheelTarget(inputs.rightPwm, inputs.driversEnabled, params_.rightMotorGain) - rightWheel_) * dt / tau;

  const double grip = gripLimit();

  // Aceleración longitudinal limitada por adherencia.
  const double desiredSpeed = (leftWheel_ + rightWheel_) / 2.0;
  const double maxDeltaV = grip * dt;
  speed_ += std::max(-maxDeltaV, std::min(maxDeltaV, desiredSpeed - speed_));

  // Giro limitado por adherencia lateral: |v·ω| <= grip.
  const double desiredYaw = (rightWheel_ - leftWheel_) / params_.wheelBase;
  const double maxYaw = std::fabs(speed_) > 1e-3 ? grip / std::fabs(speed_) : 1e9;
  sliding_ = std::fabs(desiredYaw) > maxYaw;
  const double targetYaw = std::max(-maxYaw, std::min(maxYaw, desiredYaw));
  // Inercia de giro: el robot no cambia de rumbo instantáneamente.
  if (params_.yawTimeConstant > 0.0) {
    yawRate_ += (targetYaw - yawRate_) * std::min(1.0, dt / params_.yawTimeConstant);
  } else {
    yawRate_ = targetYaw;
  }

  pose_.heading = wrapAngle(pose_.heading + yawRate_ * dt);
  pose_.position = pose_.position + pose_.forward() * (speed_ * dt);
}

// Sensor 0 = extremo izquierdo, sensor 15 = extremo derecho.
Vec2 RobotModel::sensorPosition(int index, int sensorCount) const {
  const double lateral = ((sensorCount - 1) / 2.0 - index) * params_.sensorPitch;
  return pose_.toWorld(params_.sensorBarAhead, lateral);
}

}  // namespace sim
