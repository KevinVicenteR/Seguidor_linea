#include "LineFollower.h"

#include "NeuralDerivativeWeights.h"
#ifndef ATLAS_NO_HIERARCHY
#include "NeuralHierarchyWeights.h"
#endif
#include "TinyMlp.h"

namespace atlas {

LineFollower::LineFollower(LineSensorArray& sensors, IDrivetrain& drivetrain, const RobotConfig& config)
    : sensors_(sensors),
      drivetrain_(drivetrain),
      config_(&config),
      pd_(config.control.kp, config.control.kd, config.control.maxCorrectionPwm, config.control.derivativeFilterShift),
      guard_(config.adaptive),
      governor_(config.adaptive),
      planner_(config.planner) {
  pd_.setIntegral(config.control.ki, config.control.integralLimit);
}

void LineFollower::useConfig(const RobotConfig& config) {
  if (running_) return;
  config_ = &config;
  const ControlConfig& control = config.control;
  pd_.configure(control.kp, control.kd, control.maxCorrectionPwm, control.derivativeFilterShift);
  pd_.setIntegral(control.ki, control.integralLimit);
  guard_.configure(config.adaptive);
  governor_.configure(config.adaptive);
  planner_.configure(config.planner);
}

void LineFollower::start(int16_t cruisePwm, uint32_t nowMs, RunMode mode) {
  mode_ = map_ == nullptr ? RunMode::Plain : mode;
  if (mode_ == RunMode::MapGuided && !map_->isValid()) mode_ = RunMode::Mapping;
  if (mode_ == RunMode::Mapping) map_->startRecording();
  if (mode_ == RunMode::MapGuided) cruisePwm = config_->planner.maxPwm;
  distanceUnits_ = 0;
  mappingTicks_ = 0;
  mappingLostTicks_ = 0;
  runTicks_ = 0;
  runLostTicks_ = 0;
  longestLostMs_ = 0;
  lostSinceMs_ = nowMs;
  lostStartBin_ = 0;
  failureBin_ = 0;
  stoppedWhileLost_ = false;
  stoppedIncomplete_ = false;
  bridging_ = false;

  drivetrain_.enable();
  pd_.reset();
  predictor_.reset();
  lastError_ = 0;
  historyCount_ = 0;
  guard_.reset(nowMs);
  governor_.reset(nowMs);
  recovering_ = false;
  sensors_.resetTracking();

  startMs_ = nowMs;
  limitedBase_ = 0;
  limitedAtMs_ = nowMs;
  const SpeedConfig& speed = config_->speed;
  velocityRamp_.start(nowMs + speed.startKickMs, speed.runRampMs, speed.startKickPwm, cruisePwm);
  impellerRamp_.start(nowMs, config_->impeller.runRampMs, 0, config_->impeller.pwm);

  running_ = true;
  stopping_ = false;
  lastOnLineMs_ = nowMs;
  curveSinceMs_ = nowMs;
  curveAmbiguous_ = false;
  centered_ = true;
  centeredSinceMs_ = nowMs - config_->lostLine.gapStraightMs;
  cornerSeen_ = false;
  bridgedOnce_ = false;
  lastResyncBin_ = 0xFFFFFFFFUL;
}

void LineFollower::requestStop(uint32_t nowMs) {
  if (!running_ || stopping_) return;
  brakeRamp_.start(nowMs, config_->speed.stopBrakeMs, currentVelocity(nowMs), 0);
  stopping_ = true;
  drivetrain_.impeller().setPwm(0);
  noteHowRunEnded(nowMs);
}

void LineFollower::noteHowRunEnded(uint32_t nowMs) {
  const PlannerConfig& c = config_->planner;
  stoppedWhileLost_ = recovering_ && nowMs - lostSinceMs_ >= c.stoppedWhileLostMs;
  stoppedIncomplete_ = mode_ == RunMode::MapGuided && map_ != nullptr &&
                       static_cast<uint32_t>(currentBin()) * 100 < static_cast<uint32_t>(map_->length()) * c.completedMapPercent;
  if (stoppedWhileLost_) {
    failureBin_ = lostStartBin_;
  } else if (stoppedIncomplete_ && longestLostMs_ <= c.failedRunLostMs) {
    failureBin_ = mapBin();
  }
}

void LineFollower::update(uint32_t nowMs) {
  if (!running_) return;

  if (stopping_ && brakeRamp_.isFinished(nowMs)) {
    running_ = false;
    return;
  }

  const int16_t velocity = currentVelocity(nowMs);
  scaleCrossingHoldToSpeed(velocity);
  reading_ = sensors_.readLine();
  if (reading_.isTrackable()) {
    if (recovering_) {
      pd_.reset();
      predictor_.reset();
      lastError_ = 0;
      historyCount_ = 0;
      reacquiredMs_ = nowMs;
      curveSinceMs_ = nowMs;
    }
    if (bridging_) {
      lastBridgeMs_ = nowMs;
      bridgedOnce_ = true;
    }
    recovering_ = false;
    bridging_ = false;
    command_ = followLine(governor_.apply(guidedVelocity(velocity)), nowMs);
  } else if (isCrossingGap(nowMs)) {
    if (!bridging_) onGapEntered();
    bridging_ = true;
    command_ = bridgeGap(limitAcceleration(guidedVelocity(velocity), nowMs));
  } else {
    bridging_ = false;
    if (!recovering_) governor_.penalize(config_->adaptive.speedDropOnLoss, nowMs);
    recovering_ = true;
    command_ = recoverLine(nowMs);
    if (limitedBase_ > config_->lostLine.outerMotorPwm) limitedBase_ = config_->lostLine.outerMotorPwm;
    limitedAtMs_ = nowMs;
  }
  command_.left = clampWheel(command_.left);
  command_.right = clampWheel(command_.right);
  command_.impeller = stopping_ ? 0 : impellerRamp_.valueAt(nowMs);
  apply(command_);
  if (mode_ == RunMode::Mapping) recordCurvature();
  trackDistance();
  trackLineLoss(nowMs);
}

void LineFollower::trackLineLoss(uint32_t nowMs) {
  if (stopping_) return;
  runTicks_++;
  if (!recovering_) {
    lostSinceMs_ = nowMs;
    lostStartBin_ = mapBin();
    return;
  }
  runLostTicks_++;
  const uint32_t lostMs = nowMs - lostSinceMs_;
  if (lostMs > longestLostMs_) {
    longestLostMs_ = lostMs;
    failureBin_ = lostStartBin_;
  }
}

bool LineFollower::runFailed() const {
  return longestLostMs_ > config_->planner.failedRunLostMs || stoppedWhileLost_ || stoppedIncomplete_;
}

bool LineFollower::correctMapAfterFailedRun() {
  if (mode_ != RunMode::MapGuided || map_ == nullptr || !runFailed()) return false;
  const PlannerConfig& c = config_->planner;
  map_->markDangerZone(failureBin_, c.dangerZoneBinsBefore, c.dangerZoneBinsAfter, c.dangerCurvatureBoost);
  map_->reduceCurveSpeed(c.curveSpeedStepPercent, c.minCurveSpeedPercent);
  return true;
}

void LineFollower::trackDistance() {
  const int32_t average = (static_cast<int32_t>(command_.left) + command_.right) / 2;
  if (average > 0) distanceUnits_ += static_cast<uint32_t>(average) * config_->control.loopPeriodUs / 100;
}

uint16_t LineFollower::currentBin() const {
  const uint32_t bin = distanceUnits_ / config_->planner.binUnits;
  return bin > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(bin);
}

uint16_t LineFollower::mapBin() const {
  if (mode_ != RunMode::MapGuided || map_ == nullptr || map_->length() == 0) return currentBin();
  return currentBin() % map_->length();
}

int16_t LineFollower::guidedVelocity(int16_t velocity) const {
  if (mode_ != RunMode::MapGuided) return velocity;
  const int16_t planned = planner_.speedAt(*map_, mapBin());
  return planned < velocity ? planned : velocity;
}

void LineFollower::recordCurvature() {
  uint8_t curvature = 0;
  if (!recovering_) {
    const int32_t left = command_.left < 0 ? -command_.left : command_.left;
    const int32_t right = command_.right < 0 ? -command_.right : command_.right;
    const int32_t difference = command_.left > command_.right ? command_.left - command_.right
                                                              : command_.right - command_.left;
    const int32_t scaled = left + right > 0 ? difference * 255 / (left + right) : 0;
    curvature = static_cast<uint8_t>(scaled > 255 ? 255 : scaled);
  }
  if (bridging_) curvature = 0;
  map_->recordCurvature(currentBin(), curvature, recovering_);
  mappingTicks_++;
  if (recovering_) mappingLostTicks_++;
}

void LineFollower::onGapEntered() {
  if (map_ == nullptr) return;
  if (mode_ == RunMode::Mapping) {
    map_->recordGap(currentBin());
  } else if (mode_ == RunMode::MapGuided) {
    const int32_t gap = map_->nearestGap(mapBin(), config_->planner.resyncWindowBins);
    const uint32_t lapStart = static_cast<uint32_t>(currentBin() / map_->length()) * map_->length();
    if (gap < 0) return;
    const uint32_t target = lapStart + static_cast<uint32_t>(gap);
    if (target == lastResyncBin_) return;
    lastResyncBin_ = target;
    distanceUnits_ = target * config_->planner.binUnits;
  }
}

bool LineFollower::finishMapping() {
  if (mode_ != RunMode::Mapping || map_ == nullptr) return false;
  const uint16_t lostPerMille =
      mappingTicks_ ? static_cast<uint16_t>(mappingLostTicks_ * 1000 / mappingTicks_) : 0;
  map_->finishRecording(currentBin(), config_->planner.minimumBins, lostPerMille);
  if (runFailed()) map_->invalidate();
  return map_->isValid();
}

void LineFollower::scaleCrossingHoldToSpeed(int16_t velocity) {
  const SensorConfig& sensors = config_->sensors;
  const int32_t speed = velocity < 50 ? 50 : velocity;
  const int32_t reads = static_cast<int32_t>(sensors.maxHeldReads) * sensors.heldReadsReferencePwm / speed;
  sensors_.setHeldReadsLimit(static_cast<uint16_t>(reads > 2000 ? 2000 : reads));
}

int16_t LineFollower::limitAcceleration(int16_t target, uint32_t nowMs) {
  const uint16_t accel = config_->speed.maxAccelPwmPerS;
  const uint32_t elapsedMs = nowMs - limitedAtMs_;
  if (accel == 0 || target <= limitedBase_) {
    limitedBase_ = target;
    limitedAtMs_ = nowMs;
    return target;
  }
  const int32_t step = static_cast<int32_t>(accel) * elapsedMs / 1000;
  if (step == 0) return limitedBase_;
  const int32_t raised = limitedBase_ + step;
  if (raised >= target) {
    limitedBase_ = target;
    limitedAtMs_ = nowMs;
  } else {
    limitedBase_ = static_cast<int16_t>(raised);
    limitedAtMs_ += static_cast<uint32_t>(step) * 1000 / accel;
  }
  return limitedBase_;
}

int16_t LineFollower::currentVelocity(uint32_t nowMs) const {
  if (stopping_) return brakeRamp_.valueAt(nowMs);
  if (nowMs - startMs_ < config_->speed.startKickMs) return config_->speed.startKickPwm;
  return velocityRamp_.valueAt(nowMs);
}

MotorCommand LineFollower::followLine(int16_t velocity, uint32_t nowMs) {
  rememberLastSide();
  lastOnLineMs_ = nowMs;
  curveAmbiguous_ = reading_.ambiguous;

  const int32_t error = static_cast<int32_t>(reading_.position) - kLinePositionCenter;
  const int32_t magnitude = error < 0 ? -error : error;
  const int16_t trigger = config_->corner.triggerError;
  if (magnitude < trigger || (error < 0 ? lastError_ > -trigger : lastError_ < trigger)) curveSinceMs_ = nowMs;
  trackStraightRun(error, nowMs);
  detectCorner(error, nowMs);
  if (guard_.update(error, nowMs)) governor_.penalize(config_->adaptive.speedDropOnOscillation, nowMs);
  governor_.recover(nowMs);
  rememberError(static_cast<int16_t>(error), velocity);
  const ControlConfig& control = config_->control;
  const uint16_t guard = guard_.gainPercent();
  uint8_t kpPercent = static_cast<uint8_t>(guard);
  uint8_t kdPercent = static_cast<uint8_t>(guard);
#ifndef ATLAS_NO_HIERARCHY
  if (control.neuralGains) {
    neuralGains(error, velocity, nowMs, &kpPercent, &kdPercent);
    kpPercent = static_cast<uint8_t>(kpPercent * guard / 100);
    kdPercent = static_cast<uint8_t>(kdPercent * guard / 100);
  }
#endif
  const int32_t pdError =
      control.latencyCompensation
          ? error - predictor_.pendingErrorShift(control.predictorShift, control.predictorGain)
          : error;
  int16_t correction;
  if (control.neuralDerivative && historyCount_ >= kErrorHistory) {
    correction = pd_.updateWithDerivative(pdError, neuralDerivative(), control.derivativeBlend, kpPercent, kdPercent);
  } else if (control.neuralGains) {
    correction = pd_.update(pdError, kpPercent, kdPercent);
  } else {
    correction = pd_.update(pdError, guard_.gainPercent());
  }
  pdCorrection_ = correction;
#ifndef ATLAS_NO_TRAINING_HOOKS
  if (control.residualNet != nullptr && historyCount_ >= kErrorHistory) {
    for (uint8_t k = 0; k < 5; k++) residualInputs_[k] = derivativeInputs_[k];
    const int16_t e = static_cast<int16_t>(error >> 6);
    residualInputs_[5] = static_cast<int8_t>(e > 127 ? 127 : (e < -127 ? -127 : e));
    residualInputs_[6] = derivativeInputs_[5];
    const int16_t c = correction >> 3;
    residualInputs_[7] = static_cast<int8_t>(c > 127 ? 127 : (c < -127 ? -127 : c));
    int32_t residual = 0;
    TinyMlp(*control.residualNet).evaluate(residualInputs_, &residual);
    correction += static_cast<int16_t>(clampValue<int32_t>(residual, -control.residualLimit, control.residualLimit));
  }
#endif
  predictor_.remember(correction);
  lastError_ = error;
  const int16_t base = limitAcceleration(slowDownForCurve(scaleForCorner(velocity, nowMs), error), nowMs);

  MotorCommand command;
  command.left = base + correction;
  command.right = base - correction;
  return command;
}

#ifndef ATLAS_NO_HIERARCHY
namespace {
int8_t toInput(int16_t value) { return static_cast<int8_t>(value > 127 ? 127 : value); }
}

void LineFollower::neuralGains(int32_t error, int16_t velocity, uint32_t nowMs, uint8_t* kpPercent,
                               uint8_t* kdPercent) {
  const ControlConfig& control = config_->control;
  const int16_t e = static_cast<int16_t>(error);
  const int16_t delta = e - static_cast<int16_t>(lastError_);
  const int16_t magnitude = e < 0 ? -e : e;
  const int16_t change = delta < 0 ? -delta : delta;
  const uint32_t sinceReacquired = nowMs - reacquiredMs_;
  int8_t* input = neuralInputs_;
  input[0] = toInput(magnitude >> 6);
  input[1] = toInput(change >> 2);
  input[2] = delta == 0 || e == 0 ? 0 : ((e > 0) == (delta > 0) ? 127 : -127);
  input[3] = toInput(velocity >> 1);
  input[4] = toInput(curvatureAhead() >> 1);
  input[5] = toInput(guard_.gainPercent());
  input[6] = reading_.ambiguous || reading_.otherLineLeft || reading_.otherLineRight ? 127 : 0;
  input[7] = toInput(static_cast<int16_t>(sinceReacquired > 254 ? 127 : sinceReacquired >> 1));
  evaluateHierarchy(control.hierarchy != nullptr ? *control.hierarchy : neural::kHierarchy, input, kpPercent,
                    kdPercent, situationWeights_);
}

#endif

__attribute__((noinline)) void LineFollower::rememberError(int16_t error, int16_t velocity) {
  for (uint8_t i = kErrorHistory - 1; i > 0; i--) errorHistory_[i] = errorHistory_[i - 1];
  errorHistory_[0] = error;
  if (historyCount_ < kErrorHistory) historyCount_++;
  for (uint8_t k = 0; k < 5; k++) {
    const uint8_t span = static_cast<uint8_t>(1 << k);
    const int16_t slope = historyCount_ > span ? (error - errorHistory_[span]) >> (k + 1) : 0;
    derivativeInputs_[k] = static_cast<int8_t>(slope > 127 ? 127 : (slope < -127 ? -127 : slope));
  }
  derivativeInputs_[5] = static_cast<int8_t>(velocity > 254 ? 127 : velocity >> 1);
  const int16_t magnitude = error < 0 ? -error : error;
  derivativeInputs_[6] = static_cast<int8_t>(magnitude >> 6 > 127 ? 127 : magnitude >> 6);
}

int32_t LineFollower::neuralDerivative() const {
  int32_t derivative = 0;
  TinyMlp(neural::kDerivativeNet).evaluate(derivativeInputs_, &derivative);
  return derivative;
}

uint8_t LineFollower::curvatureAhead() const {
  if (mode_ != RunMode::MapGuided || map_ == nullptr || map_->length() == 0) return 0;
  uint8_t highest = 0;
  uint16_t bin = mapBin();
  for (uint8_t k = 0; k < 5; k++) {
    if (++bin >= map_->length()) bin = 0;
    const uint8_t c = map_->curvature(bin);
    if (c > highest) highest = c;
  }
  return highest;
}

void LineFollower::detectCorner(int32_t error, uint32_t nowMs) {
  const CornerConfig& corner = config_->corner;
  if (!corner.enabled) return;
  const int32_t magnitude = error < 0 ? -error : error;
  const bool cornerAhead = reading_.ambiguous || reading_.otherLineLeft || reading_.otherLineRight ||
                           reading_.leftEdgeDetected || reading_.rightEdgeDetected || magnitude >= corner.triggerError;
  if (cornerAhead) {
    lastCornerMs_ = nowMs;
    cornerSeen_ = true;
  }
}

uint8_t LineFollower::cornerSpeedPercent(uint32_t nowMs) const {
  const CornerConfig& corner = config_->corner;
  if (!corner.enabled || !cornerSeen_) return 100;
  const uint32_t since = nowMs - lastCornerMs_;
  if (since <= corner.holdMs) return corner.speedPercent;
  const uint32_t ramp = since - corner.holdMs;
  if (ramp >= corner.recoveryMs) return 100;
  return static_cast<uint8_t>(corner.speedPercent + (100 - corner.speedPercent) * ramp / corner.recoveryMs);
}

int16_t LineFollower::scaleForCorner(int16_t pwm, uint32_t nowMs) const {
  return static_cast<int16_t>(static_cast<int32_t>(pwm) * cornerSpeedPercent(nowMs) / 100);
}

int16_t LineFollower::slowDownForCurve(int16_t velocity, int32_t error) const {
  const int32_t magnitude = error < 0 ? -error : error;
  const int32_t reduction =
      static_cast<int32_t>(velocity) * config_->control.curveSlowdownPercent * magnitude / (100L * kLinePositionCenter);
  return static_cast<int16_t>(velocity - reduction);
}

void LineFollower::trackStraightRun(int32_t error, uint32_t nowMs) {
  const LostLineConfig& lost = config_->lostLine;
  const bool centered = looksStraight(error);
  if (centered && !centered_) {
    const bool chainedGap = bridgedOnce_ && nowMs - lastBridgeMs_ <= lost.gapChainMs;
    centeredSinceMs_ = chainedGap ? nowMs - lost.gapStraightMs : nowMs;
  }
  centered_ = centered;
}

bool LineFollower::looksStraight(int32_t error) const {
  const int32_t magnitude = error < 0 ? -error : error;
  return magnitude <= config_->lostLine.gapCenterTolerance && !reading_.ambiguous && !reading_.leftEdgeDetected &&
         !reading_.rightEdgeDetected && !reading_.otherLineLeft && !reading_.otherLineRight;
}

bool LineFollower::isCrossingGap(uint32_t nowMs) const {
  const LostLineConfig& lost = config_->lostLine;
  const bool cameFromStraight = centered_ && lastOnLineMs_ - centeredSinceMs_ >= lost.gapStraightMs;
  const bool knownGap = mode_ == RunMode::MapGuided && map_ != nullptr && map_->nearestGap(mapBin(), 2) >= 0;
  return !reading_.onLine && (cameFromStraight || knownGap) && nowMs - lastOnLineMs_ <= lost.gapBridgeMs;
}

MotorCommand LineFollower::bridgeGap(int16_t velocity) const {
  MotorCommand command;
  command.left = velocity;
  command.right = velocity;
  return command;
}

MotorCommand LineFollower::recoverLine(uint32_t nowMs) const {
  const LostLineConfig& lost = config_->lostLine;
  const uint32_t curveMs = lastOnLineMs_ - curveSinceMs_;
  const int16_t turn = pdCorrection_ < 0 ? -pdCorrection_ : pdCorrection_;
  const bool tightCurve = curveAmbiguous_ && lost.curveBrakeMs > lost.innerBrakeMs && curveMs >= lost.curveBrakeMs &&
                          curveMs <= lost.curveWindowMs && limitedBase_ > 2L * turn &&
                          limitedBase_ >= scaleForCorner(lost.outerMotorPwm, lastOnLineMs_);
  const uint16_t brakeMs = tightCurve ? lost.curveBrakeMs : lost.innerBrakeMs;
  const bool innerBraking = nowMs - lastOnLineMs_ <= brakeMs;
  const int16_t inner = innerBraking ? lost.innerMotorPwm : 0;
  const int16_t outer = scaleForCorner(lost.outerMotorPwm, nowMs);

  MotorCommand command;
  if (lastSide_ == Side::Left) {
    command.left = inner;
    command.right = outer;
  } else {
    command.left = outer;
    command.right = inner;
  }
  return command;
}

void LineFollower::rememberLastSide() {
  if (reading_.leftEdgeDetected || reading_.otherLineLeft) lastSide_ = Side::Left;
  if (reading_.rightEdgeDetected || reading_.otherLineRight) lastSide_ = Side::Right;
}

int16_t LineFollower::clampWheel(int16_t pwm) const {
  return clampValue(pwm, config_->control.maxBackwardPwm, config_->control.maxForwardPwm);
}

int16_t LineFollower::compensateDeadband(int16_t pwm) const {
  const int32_t minimum = config_->control.minMotorPwm;
  if (pwm == 0 || minimum <= 0) return pwm;
  const int32_t magnitude = pwm < 0 ? -pwm : pwm;
  const int32_t compensated = minimum + magnitude * (kPwmResolution - minimum) / kPwmResolution;
  return static_cast<int16_t>(pwm < 0 ? -compensated : compensated);
}

void LineFollower::apply(const MotorCommand& command) {
  drivetrain_.left().setPwm(compensateDeadband(command.left));
  drivetrain_.right().setPwm(compensateDeadband(command.right));
  drivetrain_.impeller().setPwm(command.impeller);
}

}
