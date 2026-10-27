#include "PdController.h"

#include "Ramp.h"

namespace atlas {

PdController::PdController(float kp, float kd, int16_t outputLimit, uint8_t derivativeFilterShift)
    : kp_(kp), kd_(kd), outputLimit_(outputLimit), filterShift_(derivativeFilterShift) {}

void PdController::configure(float kp, float kd, int16_t outputLimit, uint8_t derivativeFilterShift) {
  kp_ = kp;
  kd_ = kd;
  outputLimit_ = outputLimit;
  filterShift_ = derivativeFilterShift;
  reset();
}

void PdController::reset() {
  previousError_ = 0;
  filteredDelta_ = 0;
  primed_ = false;
  integral_ = 0;
}

int16_t PdController::update(int32_t error, uint8_t gainPercent) { return update(error, gainPercent, gainPercent); }

int16_t PdController::update(int32_t error, uint8_t kpPercent, uint8_t kdPercent) {
  filterDelta(error);
#ifndef ATLAS_NO_TRAINING_HOOKS
  integrate(error);
#endif
  return combine(error, filteredDelta_, kpPercent, kdPercent);
}

int16_t PdController::updateWithDerivative(int32_t error, int32_t derivative, uint8_t blendPercent,
                                           uint8_t kpPercent, uint8_t kdPercent) {
  filterDelta(error);
  const int32_t blended = filteredDelta_ + (derivative - filteredDelta_) * blendPercent / 100;
  return combine(error, blended, kpPercent, kdPercent);
}

#ifndef ATLAS_NO_TRAINING_HOOKS
void PdController::integrate(int32_t error) {
  if (integralLimit_ <= 0) return;
  const int32_t pd = static_cast<int32_t>(error * kp_ + filteredDelta_ * kd_ + integral_ * ki_);
  const int32_t saturation = outputLimit_ < 255 ? outputLimit_ : 255;
  const bool saturated = (pd >= saturation && error > 0) || (pd <= -saturation && error < 0);
  if (saturated) return;
  integral_ = clampValue<int32_t>(integral_ + error, -integralLimit_, integralLimit_);
}
#endif

__attribute__((noinline)) int16_t PdController::combine(int32_t error, int32_t derivative, uint8_t kpPercent,
                                                        uint8_t kdPercent) const {
#ifdef ATLAS_NO_TRAINING_HOOKS
  const int32_t output = static_cast<int32_t>((error * kp_ * kpPercent + derivative * kd_ * kdPercent) * 0.01f);
#else
  const int32_t output = static_cast<int32_t>((error * kp_ * kpPercent + derivative * kd_ * kdPercent) * 0.01f +
                                              integral_ * ki_);
#endif
  return clampOutput(output);
}

void PdController::filterDelta(int32_t error) {
  if (!primed_) {
    previousError_ = error;
    primed_ = true;
  }
  const int32_t deltaError = error - previousError_;
  previousError_ = error;
  filteredDelta_ += (deltaError - filteredDelta_) / (1L << filterShift_);
}

int16_t PdController::clampOutput(int32_t output) const {
  return static_cast<int16_t>(clampValue<int32_t>(output, -outputLimit_, outputLimit_));
}

}
