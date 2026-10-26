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

class ILineSensorBus {
 public:
  virtual ~ILineSensorBus() {}
  virtual uint8_t read(uint8_t channel) = 0;
};

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

class IPersistentStorage {
 public:
  virtual ~IPersistentStorage() {}
  virtual uint8_t read(uint16_t address) = 0;
  virtual void write(uint16_t address, uint8_t value) = 0;
};

class IDiagnostics {
 public:
  virtual ~IDiagnostics() {}
  virtual void run() = 0;
};

}
