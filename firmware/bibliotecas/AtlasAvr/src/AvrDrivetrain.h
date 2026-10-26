#pragma once

#include <Arduino.h>

#include "Config.h"
#include "Hal.h"

namespace atlas {

enum class MotorPolarity : int8_t { Normal = 1, Inverted = -1 };

class AvrWheelMotor : public IMotor {
 public:
  AvrWheelMotor(uint8_t directionPin, volatile uint16_t& compareRegister, int8_t polarity, const bool& enabled);
  void setPwm(int16_t pwm) override;

 private:
  uint8_t directionPin_;
  volatile uint16_t& compare_;
  int8_t polarity_;
  const bool& enabled_;
};

class AvrImpeller : public IMotor {
 public:
  explicit AvrImpeller(const bool& enabled) : enabled_(enabled) {}
  void setPwm(int16_t pwm) override;

 private:
  const bool& enabled_;
};

class AvrDrivetrain : public IDrivetrain {
 public:
  AvrDrivetrain(MotorPolarity leftPolarity, MotorPolarity rightPolarity, bool motorsEnabled);

  void begin();
  void enable() override;
  void disable() override;
  IMotor& left() override { return left_; }
  IMotor& right() override { return right_; }
  IMotor& impeller() override { return impeller_; }

 private:
  bool motorsEnabled_;
  AvrWheelMotor left_;
  AvrWheelMotor right_;
  AvrImpeller impeller_;
};

}
