#include "AvrDrivetrain.h"

namespace atlas {

namespace {
constexpr uint8_t kInhibitPin = 12;
constexpr uint8_t kLeftDirectionPin = 6;
constexpr uint8_t kLeftPwmPin = 9;
constexpr uint8_t kRightDirectionPin = 5;
constexpr uint8_t kRightPwmPin = 10;
constexpr uint8_t kImpellerPin = 11;

constexpr uint16_t kWakeUpUs = 5;
constexpr uint16_t kStandbyUs = 300;

constexpr uint16_t kWheelPulseMax = 400 - 1;
constexpr uint16_t kImpellerPulseMax = 0xFF;

uint16_t pwmToPulse(int16_t pwm, uint16_t pulseMax) {
  const uint32_t magnitude = pwm < 0 ? -static_cast<int32_t>(pwm) : pwm;
  const uint32_t pulse = magnitude * pulseMax / kPwmResolution;
  return pulse > pulseMax ? pulseMax : static_cast<uint16_t>(pulse);
}
}

AvrWheelMotor::AvrWheelMotor(uint8_t directionPin, volatile uint16_t& compareRegister, int8_t polarity,
                             const bool& enabled)
    : directionPin_(directionPin), compare_(compareRegister), polarity_(polarity), enabled_(enabled) {}

void AvrWheelMotor::setPwm(int16_t pwm) {
  if (!enabled_) return;

  const uint16_t pulse = pwmToPulse(pwm, kWheelPulseMax);
  const int16_t direction = pwm * polarity_;

  if (direction > 0) {
    digitalWrite(directionPin_, LOW);
    compare_ = pulse;
  } else if (direction < 0) {
    digitalWrite(directionPin_, HIGH);
    compare_ = kWheelPulseMax - pulse;
  } else {
    digitalWrite(directionPin_, LOW);
    compare_ = 0;
  }
}

void AvrImpeller::setPwm(int16_t pwm) {
  if (!enabled_) return;
  OCR2A = static_cast<uint8_t>(pwmToPulse(pwm, kImpellerPulseMax));
}

AvrDrivetrain::AvrDrivetrain(MotorPolarity leftPolarity, MotorPolarity rightPolarity, bool motorsEnabled)
    : motorsEnabled_(motorsEnabled),
      left_(kLeftDirectionPin, OCR1A, static_cast<int8_t>(leftPolarity), motorsEnabled_),
      right_(kRightDirectionPin, OCR1B, -static_cast<int8_t>(rightPolarity), motorsEnabled_),
      impeller_(motorsEnabled_) {}

void AvrDrivetrain::begin() {
  pinMode(kInhibitPin, OUTPUT);
  pinMode(kLeftDirectionPin, OUTPUT);
  pinMode(kLeftPwmPin, OUTPUT);
  pinMode(kRightDirectionPin, OUTPUT);
  pinMode(kRightPwmPin, OUTPUT);
  pinMode(kImpellerPin, OUTPUT);

  digitalWrite(kInhibitPin, LOW);
  digitalWrite(kLeftDirectionPin, LOW);
  digitalWrite(kRightDirectionPin, LOW);

  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = kWheelPulseMax;
  OCR1A = 0;
  OCR1B = 0;

  TCCR2A = _BV(COM2A1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  OCR2A = 0;
}

void AvrDrivetrain::enable() {
  digitalWrite(kInhibitPin, HIGH);
  delayMicroseconds(kWakeUpUs);
}

void AvrDrivetrain::disable() {
  digitalWrite(kInhibitPin, LOW);
  digitalWrite(kLeftDirectionPin, LOW);
  digitalWrite(kRightDirectionPin, LOW);
  OCR1A = 0;
  OCR1B = 0;
  delayMicroseconds(kStandbyUs);
}

}
