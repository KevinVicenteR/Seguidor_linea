#pragma once

#include <stdint.h>

#include "Hal.h"
#include "TrackMap.h"

namespace atlas {

class TrackMapStore {
 public:
  explicit TrackMapStore(IPersistentStorage& storage) : storage_(storage) {}

  void save(TrackMap& map);
  bool load(TrackMap& map);
  void erase();

 private:
  static constexpr uint16_t kBaseAddress = 64;
  static constexpr uint8_t kSignature = 0x5F;

  void put(uint16_t offset, uint8_t value, uint8_t* checksum);
  uint8_t get(uint16_t offset, uint8_t* checksum);

  IPersistentStorage& storage_;
};

}
