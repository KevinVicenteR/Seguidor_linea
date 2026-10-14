/*
  TrackMapStore.h - Guarda el mapa de la pista en memoria no volátil para que
  sobreviva al apagado entre intentos.
*/
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
  static constexpr uint16_t kBaseAddress = 64;  // después de la calibración
  static constexpr uint8_t kSignature = 0x5F;  // v3: confianza y velocidad de curva

  void put(uint16_t offset, uint8_t value, uint8_t* checksum);
  uint8_t get(uint16_t offset, uint8_t* checksum);

  IPersistentStorage& storage_;
};

}  // namespace atlas
