#pragma once

#include <stdint.h>

#include "Hal.h"
#include "LineSensorArray.h"

namespace atlas {

class CalibrationStore {
 public:
  explicit CalibrationStore(IPersistentStorage& storage) : storage_(storage) {}

  void save(const LineSensorArray& sensors);
  bool load(LineSensorArray& sensors);

 private:
  static constexpr uint8_t kSignature = 0xA7;
  static constexpr uint8_t kVersion = 2;
  static constexpr uint16_t kBaseAddress = 0;

  void writeByte(uint16_t offset, uint8_t value, uint8_t* checksum);

  IPersistentStorage& storage_;
};

}
