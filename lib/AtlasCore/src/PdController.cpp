#include "PdController.h"

#include "Ramp.h"

namespace atlas {

PdController::PdController(float kp, float kd, int16_t outputLimit)
    : kp_(kp), kd_(kd), outputLimit_(outputLimit) {}

void PdController::reset() { previousError_ = 0; }

int16_t PdController::update(int32_t error) {
  const int32_t deltaError = error - previousError_;
  previousError_ = error;

  const int32_t output = static_cast<int32_t>(error * kp_ + deltaError * kd_);
  return static_cast<int16_t>(clampValue<int32_t>(output, -outputLimit_, outputLimit_));
}

}  // namespace atlas
