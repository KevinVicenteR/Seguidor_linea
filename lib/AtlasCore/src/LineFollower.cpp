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
  // Si arranca sobre un hueco, avanza recto hasta encontrar la línea.
  centered_ = true;
  centeredSinceMs_ = nowMs - config_.lostLine.gapStraightMs;
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
  const int16_t velocity = currentVelocity(nowMs);
  if (reading_.isTrackable()) {
    command_ = followLine(velocity, nowMs);
  } else if (isCrossingGap(nowMs)) {
    command_ = bridgeGap(velocity);
  } else {
    command_ = recoverLine(nowMs);
  }
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
  trackStraightRun(error, nowMs);
  const int16_t correction = pd_.update(error);
  const int16_t base = slowDownForCurve(velocity, error);

  MotorCommand command;
  command.left = base + correction;
  command.right = base - correction;
  return command;
}

int16_t LineFollower::slowDownForCurve(int16_t velocity, int32_t error) const {
  const int32_t magnitude = error < 0 ? -error : error;
  const int32_t reduction =
      static_cast<int32_t>(velocity) * config_.control.curveSlowdownPercent * magnitude / (100L * kLinePositionCenter);
  return static_cast<int16_t>(velocity - reduction);
}

void LineFollower::trackStraightRun(int32_t error, uint32_t nowMs) {
  const bool centered = looksStraight(error);
  if (centered && !centered_) centeredSinceMs_ = nowMs;
  centered_ = centered;
}

// Línea limpia y centrada: sin cruce/esquina, sin extremos y sin otras ramas.
bool LineFollower::looksStraight(int32_t error) const {
  const int32_t magnitude = error < 0 ? -error : error;
  return magnitude <= config_.lostLine.gapCenterTolerance && !reading_.ambiguous && !reading_.leftEdgeDetected &&
         !reading_.rightEdgeDetected && !reading_.otherLineLeft && !reading_.otherLineRight;
}

// Hueco: la línea desapareció justo después de un tramo recto y centrado.
bool LineFollower::isCrossingGap(uint32_t nowMs) const {
  const LostLineConfig& lost = config_.lostLine;
  const bool cameFromStraight = centered_ && lastOnLineMs_ - centeredSinceMs_ >= lost.gapStraightMs;
  return !reading_.onLine && cameFromStraight && nowMs - lastOnLineMs_ <= lost.gapBridgeMs;
}

MotorCommand LineFollower::bridgeGap(int16_t velocity) const {
  MotorCommand command;
  command.left = velocity;
  command.right = velocity;
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

// Lado hacia donde girar si se pierde la línea: el extremo por donde salió
// o el lado por donde apareció otra rama (pico agudo).
void LineFollower::rememberLastSide() {
  if (reading_.leftEdgeDetected || reading_.otherLineLeft) lastSide_ = Side::Left;
  if (reading_.rightEdgeDetected || reading_.otherLineRight) lastSide_ = Side::Right;
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
