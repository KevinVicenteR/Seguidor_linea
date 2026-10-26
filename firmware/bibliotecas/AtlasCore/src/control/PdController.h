#pragma once

#include <stdint.h>

namespace atlas {

class PdController {
 public:
  PdController(float kp, float kd, int16_t outputLimit, uint8_t derivativeFilterShift = 0);

  void configure(float kp, float kd, int16_t outputLimit, uint8_t derivativeFilterShift);
  void reset();
  int16_t update(int32_t error, uint8_t gainPercent = 100);
  int16_t update(int32_t error, uint8_t kpPercent, uint8_t kdPercent);
  int16_t updateWithDerivative(int32_t error, int32_t derivative, uint8_t blendPercent, uint8_t kpPercent,
                               uint8_t kdPercent);
  int32_t filteredDelta() const { return filteredDelta_; }

 private:
  void filterDelta(int32_t error);
  int16_t combine(int32_t error, int32_t derivative, uint8_t kpPercent, uint8_t kdPercent) const;
  int16_t clampOutput(int32_t output) const;

  float kp_;
  float kd_;
  int16_t outputLimit_;
  uint8_t filterShift_;
  int32_t previousError_ = 0;
  int32_t filteredDelta_ = 0;
  bool primed_ = false;
};

}
