#include "SpeedSettingStore.h"

namespace atlas {

constexpr uint16_t SpeedSettingStore::kAddress;
constexpr uint8_t SpeedSettingStore::kSignature;

// Formato: firma, PWM (2 bytes), complemento del byte bajo (verificación).
void SpeedSettingStore::save(int16_t cruisePwm) {
  const uint8_t bytes[4] = {kSignature, static_cast<uint8_t>(cruisePwm & 0xFF), static_cast<uint8_t>(cruisePwm >> 8),
                            static_cast<uint8_t>(~cruisePwm & 0xFF)};
  for (uint8_t i = 0; i < 4; i++) {
    if (storage_.read(kAddress + i) != bytes[i]) storage_.write(kAddress + i, bytes[i]);
  }
}

bool SpeedSettingStore::load(int16_t* cruisePwm) {
  if (storage_.read(kAddress) != kSignature) return false;
  const uint8_t low = storage_.read(kAddress + 1);
  const uint8_t high = storage_.read(kAddress + 2);
  if (storage_.read(kAddress + 3) != static_cast<uint8_t>(~low)) return false;
  *cruisePwm = static_cast<int16_t>(low | (high << 8));
  return true;
}

}  // namespace atlas
