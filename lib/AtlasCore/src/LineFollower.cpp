#include "LineFollower.h"

namespace atlas {

LineFollower::LineFollower(LineSensorArray& sensors, IDrivetrain& drivetrain, const RobotConfig& config)
    : sensors_(sensors),
      drivetrain_(drivetrain),
      config_(config),
      pd_(config.control.kp, config.control.kd, config.control.maxCorrectionPwm) {}

void LineFollower::start(int16_t cruisePwm, uint32_t nowMs) {
  drivetrain_.enable();
  pd_.reset();
  sensors_.resetTracking();

  velocityRamp_.start(nowMs, config_.speed.runRampMs, 0, cruisePwm);
  impellerRamp_.start(nowMs, config_.impeller.runRampMs, 0, config_.impeller.pwm);

  running_ = true;
  stopping_ = false;
  lastOnLineMs_ = nowMs;
}

void LineFollower::requestStop(uint32_t nowMs) {
  if (!running_ || stopping_) return;
  brakeRamp_.start(nowMs, config_.speed.stopBrakeMs, currentVelocity(nowMs), 0);
  stopping_ = true;
}

void LineFollower::update(uint32_t nowMs) {
  if (!running_) return;

  if (stopping_ && brakeRamp_.isFinished(nowMs)) {
    running_ = false;
    return;
  }

  reading_ = sensors_.readLine();
  command_ = reading_.isTrackable() ? followLine(currentVelocity(nowMs), nowMs) : recoverLine(nowMs);
  command_.left = clampWheel(command_.left);
  command_.right = clampWheel(command_.right);
  command_.impeller = impellerRamp_.valueAt(nowMs);
  apply(command_);
}

int16_t LineFollower::currentVelocity(uint32_t nowMs) const {
  return stopping_ ? brakeRamp_.valueAt(nowMs) : velocityRamp_.valueAt(nowMs);
}

MotorCommand LineFollower::followLine(int16_t velocity, uint32_t nowMs) {
  rememberLastSide();
  lastOnLineMs_ = nowMs;

  const int32_t error = static_cast<int32_t>(reading_.position) - kLinePositionCenter;
  const int16_t correction = pd_.update(error);

  MotorCommand command;
  command.left = velocity + correction;
  command.right = velocity - correction;
  return command;
}

// Gira hacia el último lado donde se vio la línea: rueda exterior al
// máximo y rueda interior frenando en reversa durante un instante.
MotorCommand LineFollower::recoverLine(uint32_t nowMs) const {
  const LostLineConfig& lost = config_.lostLine;
  const bool innerBraking = nowMs - lastOnLineMs_ <= lost.innerBrakeMs;
  const int16_t inner = innerBraking ? lost.innerMotorPwm : 0;

  MotorCommand command;
  if (lastSide_ == Side::Left) {
    command.left = inner;
    command.right = lost.outerMotorPwm;
  } else {
    command.left = lost.outerMotorPwm;
    command.right = inner;
  }
  return command;
}

void LineFollower::rememberLastSide() {
  if (reading_.leftEdgeDetected) lastSide_ = Side::Left;
  if (reading_.rightEdgeDetected) lastSide_ = Side::Right;
}

int16_t LineFollower::clampWheel(int16_t pwm) const {
  return clampValue(pwm, config_.control.maxBackwardPwm, config_.control.maxForwardPwm);
}

void LineFollower::apply(const MotorCommand& command) {
  drivetrain_.left().setPwm(command.left);
  drivetrain_.right().setPwm(command.right);
  drivetrain_.impeller().setPwm(command.impeller);
}

}  // namespace atlas
