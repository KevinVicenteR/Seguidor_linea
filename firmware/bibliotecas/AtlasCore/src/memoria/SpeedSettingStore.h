#pragma once

#include <stdint.h>

#include "Hal.h"

namespace atlas {

class SpeedSettingStore {
 public:
  explicit SpeedSettingStore(IPersistentStorage& storage) : storage_(storage) {}

  void save(int16_t cruisePwm);
  bool load(int16_t* cruisePwm);

 private:
  static constexpr uint16_t kAddress = 48;
  static constexpr uint8_t kSignature = 0x56;

  IPersistentStorage& storage_;
};

}
