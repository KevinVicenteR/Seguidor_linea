#pragma once

#include <Arduino.h>

#include "Hal.h"

namespace atlas {

class AvrClock : public IClock {
 public:
  uint32_t micros() const override { return ::micros(); }
  uint32_t millis() const override { return ::millis(); }
  void delayMs(uint32_t ms) override { ::delay(ms); }
};

}
