#include "TrackMapStore.h"

namespace atlas {

constexpr uint16_t TrackMapStore::kBaseAddress;
constexpr uint8_t TrackMapStore::kSignature;

// Formato: firma, largo (2), ‰ sin línea (2), % velocidad de curva (1),
// huecos (1), posiciones de huecos (2 c/u), curvaturas (1 por tramo), suma de control.
void TrackMapStore::put(uint16_t offset, uint8_t value, uint8_t* checksum) {
  if (storage_.read(kBaseAddress + offset) != value) storage_.write(kBaseAddress + offset, value);
  *checksum ^= value;
}

uint8_t TrackMapStore::get(uint16_t offset, uint8_t* checksum) {
  const uint8_t value = storage_.read(kBaseAddress + offset);
  *checksum ^= value;
  return value;
}

void TrackMapStore::save(TrackMap& map) {
  uint8_t checksum = 0;
  uint16_t offset = 0;
  put(offset++, kSignature, &checksum);
  put(offset++, map.length() & 0xFF, &checksum);
  put(offset++, map.length() >> 8, &checksum);
  put(offset++, map.lostPerMille() & 0xFF, &checksum);
  put(offset++, map.lostPerMille() >> 8, &checksum);
  put(offset++, map.curveSpeedPercent(), &checksum);
  put(offset++, map.gapCount(), &checksum);
  for (uint8_t i = 0; i < map.gapCount(); i++) {
    put(offset++, map.gap(i) & 0xFF, &checksum);
    put(offset++, map.gap(i) >> 8, &checksum);
  }
  for (uint16_t i = 0; i < map.length(); i++) put(offset++, map.curvature(i), &checksum);
  uint8_t ignored = 0;
  put(offset, checksum, &ignored);
}

bool TrackMapStore::load(TrackMap& map) {
  uint8_t checksum = 0;
  uint16_t offset = 0;
  if (get(offset++, &checksum) != kSignature) return false;
  const uint16_t length = get(offset, &checksum) | (static_cast<uint16_t>(get(offset + 1, &checksum)) << 8);
  offset += 2;
  const uint16_t lostPerMille = get(offset, &checksum) | (static_cast<uint16_t>(get(offset + 1, &checksum)) << 8);
  offset += 2;
  const uint8_t curveSpeedPercent = get(offset++, &checksum);
  const uint8_t gapCount = get(offset++, &checksum);
  if (length == 0 || length > kMaxMapBins || gapCount > kMaxMapGaps) return false;

  for (uint8_t i = 0; i < gapCount; i++) {
    map.rawGaps()[i] = get(offset, &checksum) | (static_cast<uint16_t>(get(offset + 1, &checksum)) << 8);
    offset += 2;
  }
  for (uint16_t i = 0; i < length; i++) map.rawBins()[i] = get(offset++, &checksum);
  uint8_t ignored = 0;
  if (get(offset, &ignored) != checksum) {
    map.clear();
    return false;
  }
  map.restore(length, gapCount, lostPerMille, curveSpeedPercent);
  return true;
}

void TrackMapStore::erase() {
  if (storage_.read(kBaseAddress) != 0xFF) storage_.write(kBaseAddress, 0xFF);
}

}  // namespace atlas
