/*
  AvrLineSensorBus.h - Multiplexor analógico 16:1 (selección en PC0-PC3)
  conectado al canal ADC4, lectura de 8 bits (ADLAR) con prescaler /16.
*/
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

}  // namespace atlas
