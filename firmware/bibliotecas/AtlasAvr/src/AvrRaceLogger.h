#pragma once

#include <Arduino.h>

#include "LineFollower.h"

namespace atlas {

class AvrRaceLogger {
 public:
  explicit AvrRaceLogger(uint8_t every) : every_(every) {}

  void begin() { Serial.begin(115200); }

  void update(const LineFollower& follower, uint32_t nowMs) {
    if (!follower.isRunning()) return;
    const uint32_t cycle = follower.controlCycles();
    if (cycle == lastCycle_ || cycle % every_ != 0) return;
    lastCycle_ = cycle;
    const MotorCommand& command = follower.lastCommand();
    const int16_t error = static_cast<int16_t>(follower.lastReading().position) - kLinePositionCenter;
    const uint8_t flags = (follower.isRecovering() ? 1 : 0) | (follower.isBridging() ? 2 : 0) |
                          (static_cast<uint8_t>(follower.mode()) << 2);
    uint8_t frame[12] = {0xA5, 0x5A};
    put(frame + 2, static_cast<int16_t>(nowMs));
    put(frame + 4, error);
    put(frame + 6, command.left);
    put(frame + 8, command.right);
    frame[10] = flags;
    uint8_t check = 0;
    for (uint8_t i = 2; i < 11; i++) check ^= frame[i];
    frame[11] = check;
    if (Serial.availableForWrite() >= static_cast<int>(sizeof(frame))) Serial.write(frame, sizeof(frame));
  }

 private:
  static void put(uint8_t* at, int16_t value) {
    at[0] = static_cast<uint8_t>(value);
    at[1] = static_cast<uint8_t>(value >> 8);
  }

  uint8_t every_;
  uint32_t lastCycle_ = 0;
};

}
