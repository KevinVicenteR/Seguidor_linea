/*
  OscillationGuard.h - Auto-amortiguación del control de dirección.

  Detecta zigzag (el error cambia de lado varias veces en poco tiempo con
  amplitud apreciable) y reduce las ganancias del PD en escalones. Mientras
  el robot sigue estable, las ganancias regresan poco a poco al 100 %.
  Compensa robots más lentos o con más inercia que el modelo de ajuste.
*/
#pragma once

#include <stdint.h>

#include "Config.h"

namespace atlas {

class OscillationGuard {
 public:
  explicit OscillationGuard(const AdaptiveConfig& config) : config_(&config) {}
  void configure(const AdaptiveConfig& config) { config_ = &config; }

  void reset(uint32_t nowMs);
  // Devuelve true si en esta lectura se detectó un zigzag.
  bool update(int32_t error, uint32_t nowMs);
  uint8_t gainPercent() const { return gainPercent_; }

 private:
  static constexpr uint8_t kMaxCrossings = 8;

  bool registerCrossing(uint32_t nowMs);
  void recover(uint32_t nowMs);

  const AdaptiveConfig* config_;
  uint8_t gainPercent_ = 100;
  int8_t lastSide_ = 0;
  uint32_t crossings_[kMaxCrossings] = {};
  uint8_t crossingCount_ = 0;
  uint32_t lastChangeMs_ = 0;
};

}  // namespace atlas
