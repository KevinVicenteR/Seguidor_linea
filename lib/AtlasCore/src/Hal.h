/*
  Hal.h - Interfaces de abstracción de hardware.

  El núcleo de control (core/) depende solo de estas interfaces; nunca de
  registros AVR ni de Arduino.h. Las implementaciones reales viven en
  hal/avr/ y las simuladas en simulation/sim/.
*/
#pragma once

#include <stdint.h>

namespace atlas {

class IClock {
 public:
  virtual ~IClock() {}
  virtual uint32_t micros() const = 0;
  virtual uint32_t millis() const = 0;
  virtual void delayMs(uint32_t ms) = 0;
};

// Lee el canal i del multiplexor de sensores de línea (ADC de 8 bits).
class ILineSensorBus {
 public:
  virtual ~ILineSensorBus() {}
  virtual uint8_t read(uint8_t channel) = 0;
};

// PWM con signo en escala [-255, 255].
class IMotor {
 public:
  virtual ~IMotor() {}
  virtual void setPwm(int16_t pwm) = 0;
};

class IDrivetrain {
 public:
  virtual ~IDrivetrain() {}
  virtual void enable() = 0;
  virtual void disable() = 0;
  virtual IMotor& left() = 0;
  virtual IMotor& right() = 0;
  virtual IMotor& impeller() = 0;
};

// Botones SW1/SW2 y señales del módulo de arranque.
class IOperatorInput {
 public:
  virtual ~IOperatorInput() {}
  virtual bool button1() = 0;
  virtual bool button2() = 0;
  virtual bool startReady() = 0;
  virtual bool startGo() = 0;
};

enum class Led : uint8_t { Led0, Led1, Led2 };

class IIndicators {
 public:
  virtual ~IIndicators() {}
  virtual void setLed(Led led, bool on) = 0;
};

// Modo de diagnóstico opcional (solo existe en el hardware real).
class IDiagnostics {
 public:
  virtual ~IDiagnostics() {}
  virtual void run() = 0;
};

}  // namespace atlas
