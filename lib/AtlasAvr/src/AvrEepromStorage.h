#pragma once

#include <Arduino.h>
#include <avr/eeprom.h>

#include "Hal.h"

namespace atlas {

// EEPROM interna del ATmega328P (1 KB, ~100 000 escrituras por celda).
class AvrEepromStorage : public IPersistentStorage {
 public:
  uint8_t read(uint16_t address) override {
    return eeprom_read_byte(reinterpret_cast<const uint8_t*>(address));
  }
  void write(uint16_t address, uint8_t value) override {
    eeprom_update_byte(reinterpret_cast<uint8_t*>(address), value);
  }
};

}  // namespace atlas
