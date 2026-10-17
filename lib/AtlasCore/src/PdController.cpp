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

int16_t PdController::update(int32_t error, uint8_t gainPercent) { return update(error, gainPercent, gainPercent); }

int16_t PdController::update(int32_t error, uint8_t kpPercent, uint8_t kdPercent) {
  filterDelta(error);
  return combine(error, filteredDelta_, kpPercent, kdPercent);
}

int16_t PdController::updateWithDerivative(int32_t error, int32_t derivative, uint8_t kpPercent, uint8_t kdPercent) {
  filterDelta(error);
  return combine(error, derivative, kpPercent, kdPercent);
}

// Única copia de la cuenta en punto flotante (la flash del AVR está justa).
__attribute__((noinline)) int16_t PdController::combine(int32_t error, int32_t derivative, uint8_t kpPercent,
                                                        uint8_t kdPercent) const {
  const int32_t output = static_cast<int32_t>((error * kp_ * kpPercent + derivative * kd_ * kdPercent) * 0.01f);
  return clampOutput(output);
}

// El derivativo amplifica el ruido de los sensores; se suaviza con una media
// exponencial (con filterShift_ = 0 queda igual que el original).
void PdController::filterDelta(int32_t error) {
  const int32_t deltaError = error - previousError_;
  previousError_ = error;
  filteredDelta_ += (deltaError - filteredDelta_) / (1L << filterShift_);
}

int16_t PdController::clampOutput(int32_t output) const {
  return static_cast<int16_t>(clampValue<int32_t>(output, -outputLimit_, outputLimit_));
}

}  // namespace atlas
