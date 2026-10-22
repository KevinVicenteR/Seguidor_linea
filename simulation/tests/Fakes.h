/*
  Fakes.h - Dobles de prueba para las interfaces de AtlasCore.
*/
#pragma once

#include "parametros/Config.h"
#include "Hal.h"

namespace test {

// Bus de sensores con lecturas fijas por canal.
class FakeSensorBus : public atlas::ILineSensorBus {
 public:
  uint8_t read(uint8_t channel) override { return values[channel]; }

  void fill(uint8_t value) {
    for (uint8_t& v : values) v = value;
  }
  // Línea negra (ADC bajo) bajo los sensores [first, last]; fondo blanco en el resto.
  void lineAt(int first, int last, uint8_t line = 30, uint8_t surface = 220) {
    fill(surface);
    for (int i = first; i <= last; i++) values[i] = line;
  }

  uint8_t values[atlas::kSensorCount] = {};
};

class FakeMotor : public atlas::IMotor {
 public:
  void setPwm(int16_t value) override { pwm = value; }
  int16_t pwm = 0;
};

class FakeDrivetrain : public atlas::IDrivetrain {
 public:
  void enable() override { enabled = true; }
  void disable() override { enabled = false; }
  atlas::IMotor& left() override { return leftMotor; }
  atlas::IMotor& right() override { return rightMotor; }
  atlas::IMotor& impeller() override { return impellerMotor; }

  bool enabled = false;
  FakeMotor leftMotor;
  FakeMotor rightMotor;
  FakeMotor impellerMotor;
};

}  // namespace test
