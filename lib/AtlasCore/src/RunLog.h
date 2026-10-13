/*
  RunLog.h - Registro de las últimas carreras en memoria no volátil.

  Guarda, para cada carrera: tipo (reconocimiento / con mapa / normal), si
  falló, el tiempo y el tramo del mapa donde se salió. Se lee por serial en el
  modo diagnóstico y se analiza en simulation_output/analizador.html.
*/
#pragma once

#include <stdint.h>

#include "Hal.h"

namespace atlas {

struct RunRecord {
  uint8_t mode = 0;  // RunMode: 0 normal, 1 reconocimiento, 2 con mapa
  bool failed = false;
  uint16_t timeCentiseconds = 0;
  uint16_t failureBin = 0;
  uint16_t mapLength = 0;
};

class RunLog {
 public:
  static constexpr uint8_t kCapacity = 8;

  explicit RunLog(IPersistentStorage& storage) : storage_(storage) {}

  void append(const RunRecord& record);
  uint8_t count();
  // index 0 = la carrera más antigua guardada.
  RunRecord at(uint8_t index);
  void clear();

 private:
  static constexpr uint16_t kBaseAddress = 512;  // después del mapa
  static constexpr uint8_t kSignature = 0x4C;
  static constexpr uint8_t kRecordSize = 8;

  bool isInitialized();
  uint8_t nextSlot();
  void writeByte(uint16_t offset, uint8_t value);

  IPersistentStorage& storage_;
};

}  // namespace atlas
