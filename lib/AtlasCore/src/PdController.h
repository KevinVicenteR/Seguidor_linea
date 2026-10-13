/*
  PdController.h - Controlador proporcional-derivativo con salida saturada.
*/
#pragma once

#include <stdint.h>

namespace atlas {

class PdController {
 public:
  PdController(float kp, float kd, int16_t outputLimit, uint8_t derivativeFilterShift = 0);

  void reset();
  int16_t update(int32_t error);

 private:
  float kp_;
  float kd_;
  int16_t outputLimit_;
  uint8_t filterShift_;
  int32_t previousError_ = 0;
  int32_t filteredDelta_ = 0;
};

}  // namespace atlas
