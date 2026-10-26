#pragma once

#include <Arduino.h>

#include "Hal.h"

namespace atlas {

class AvrLineSensorBus : public ILineSensorBus {
 public:
  void begin();
  uint8_t read(uint8_t channel) override;

 private:
  static uint8_t convert();
};

}
