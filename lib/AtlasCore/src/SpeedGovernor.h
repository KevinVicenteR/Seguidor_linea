/*
  SpeedGovernor.h - Gobernador adaptativo de velocidad.

  Cada vez que el robot pierde la línea o zigzaguea, la velocidad baja unos
  puntos; mientras sigue la línea limpiamente se recupera 1 % cada cierto
  tiempo. Así se adapta solo a batería más cargada, poco agarre o pistas
  más difíciles de lo esperado, en vez de salirse.
*/
#pragma once

#include <stdint.h>

#include "Config.h"

namespace atlas {

class SpeedGovernor {
 public:
  explicit SpeedGovernor(const AdaptiveConfig& config) : config_(&config) {}
  void configure(const AdaptiveConfig& config) { config_ = &config; }

  void reset(uint32_t nowMs);
  void penalize(uint8_t percent, uint32_t nowMs);
  void recover(uint32_t nowMs);
  uint8_t speedPercent() const { return speedPercent_; }
  int16_t apply(int16_t pwm) const { return static_cast<int16_t>(static_cast<int32_t>(pwm) * speedPercent_ / 100); }

 private:
  const AdaptiveConfig* config_;
  uint8_t speedPercent_ = 100;
  uint32_t lastChangeMs_ = 0;
};

}  // namespace atlas
