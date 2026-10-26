#pragma once

#include <stdint.h>

namespace atlas {

class LatencyPredictor {
 public:
  void reset() {
    realized16_ = 0;
    lastCorrection_ = 0;
  }
  void remember(int16_t correction) { lastCorrection_ = correction; }

  int16_t pendingErrorShift(uint8_t shift, int8_t gain) {
    const int32_t commanded16 = static_cast<int32_t>(lastCorrection_) * 16;
    realized16_ += (commanded16 - realized16_) >> shift;
    const int32_t pending = (commanded16 - realized16_) >> 4;
    return static_cast<int16_t>((pending * gain) >> 2);
  }

 private:
  int32_t realized16_ = 0;
  int16_t lastCorrection_ = 0;
};

}
