/*
  Ramp.h - Utilidades numéricas: saturación y rampa lineal en el tiempo.
*/
#pragma once

#include <stdint.h>

namespace atlas {

template <typename T>
constexpr T clampValue(T value, T low, T high) {
  return value < low ? low : (value > high ? high : value);
}

// Interpola linealmente de `from` a `to` durante `durationMs`.
class LinearRamp {
 public:
  void start(uint32_t startMs, uint16_t durationMs, int16_t from, int16_t to) {
    startMs_ = startMs;
    durationMs_ = durationMs;
    from_ = from;
    to_ = to;
  }

  int16_t valueAt(uint32_t nowMs) const {
    const uint32_t elapsed = nowMs - startMs_;
    if (durationMs_ == 0 || elapsed >= durationMs_) return to_;
    const int32_t delta = static_cast<int32_t>(to_) - from_;
    return static_cast<int16_t>(from_ + delta * static_cast<int32_t>(elapsed) / durationMs_);
  }

  bool isFinished(uint32_t nowMs) const { return nowMs - startMs_ >= durationMs_; }

 private:
  uint32_t startMs_ = 0;
  uint16_t durationMs_ = 0;
  int16_t from_ = 0;
  int16_t to_ = 0;
};

}  // namespace atlas
