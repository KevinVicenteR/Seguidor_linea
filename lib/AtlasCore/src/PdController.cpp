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
}

int16_t PdController::update(int32_t error, uint8_t gainPercent) {
  const int32_t deltaError = error - previousError_;
  previousError_ = error;

  // El derivativo amplifica el ruido de los sensores; se suaviza con una
  // media exponencial (con filterShift_ = 0 queda igual que el original).
  filteredDelta_ += (deltaError - filteredDelta_) / (1L << filterShift_);

  const float scale = gainPercent / 100.0f;
  const int32_t output = static_cast<int32_t>((error * kp_ + filteredDelta_ * kd_) * scale);
  return static_cast<int16_t>(clampValue<int32_t>(output, -outputLimit_, outputLimit_));
}

}  // namespace atlas
