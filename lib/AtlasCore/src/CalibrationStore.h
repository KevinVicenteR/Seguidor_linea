/*
  CalibrationStore.h - Guarda y recupera la calibración
  en memoria no volátil, para no recalibrar cada vez que se enciende el robot.

  Formato: firma, versión, 16 mínimos, 16 máximos y suma de control.
  Solo se escriben los bytes que cambian (la EEPROM tiene vida limitada).
*/
#pragma once

#include <stdint.h>

#include "Hal.h"
#include "LineSensorArray.h"

namespace atlas {

class CalibrationStore {
 public:
  explicit CalibrationStore(IPersistentStorage& storage) : storage_(storage) {}

  void save(const LineSensorArray& sensors);
  // Devuelve false si no hay una calibración guardada válida.
  bool load(LineSensorArray& sensors);

 private:
  static constexpr uint8_t kSignature = 0xA7;
  static constexpr uint8_t kVersion = 2;
  static constexpr uint16_t kBaseAddress = 0;

  void writeByte(uint16_t offset, uint8_t value, uint8_t* checksum);

  IPersistentStorage& storage_;
};

}  // namespace atlas
