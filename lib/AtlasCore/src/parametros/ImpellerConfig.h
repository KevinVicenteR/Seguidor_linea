// Parámetros comunes de turbina. Velocidades por ronda en rondas/.
#pragma once
#include <stdint.h>

namespace atlas {

struct ImpellerConfig {
  int16_t pwm = 200;
  uint16_t startRampMs = 1000;  // PROGRAM / READY: ruedas quietas
  uint16_t runRampMs = 250;     // START / GO: transición a carrera
  bool spinWithReady = true;
};
}  // namespace atlas
