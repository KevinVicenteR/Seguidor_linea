/*
  SimHardware.h - Implementaciones simuladas de las interfaces de AtlasCore.

  El firmware real corre sin cambios sobre estas clases: el reloj es virtual
  y cada lectura de ADC "consume" tiempo, de modo que el lazo de control
  tiene la misma temporización que en el ATmega328P.
*/
#pragma once

#include <cstdint>
#include <random>

#include "Config.h"
#include "Hal.h"
#include "RobotModel.h"
#include "Track.h"

namespace sim {

class TimeListener {
 public:
  virtual ~TimeListener() = default;
  virtual void onTimeAdvanced(uint64_t nowUs, uint32_t deltaUs) = 0;
};

class SimClock : public atlas::IClock {
 public:
  uint32_t micros() const override { return static_cast<uint32_t>(nowUs_); }
  uint32_t millis() const override { return static_cast<uint32_t>(nowUs_ / 1000); }
  void delayMs(uint32_t ms) override { advanceUs(ms * 1000); }

  void advanceUs(uint64_t us);
  void setListener(TimeListener* listener) { listener_ = listener; }
  double seconds() const { return nowUs_ / 1e6; }
  uint64_t nowUs() const { return nowUs_; }

 private:
  static constexpr uint32_t kMaxChunkUs = 100;
  uint64_t nowUs_ = 0;
  TimeListener* listener_ = nullptr;
};

struct SensorModel {
  uint8_t surfaceLevel = 35;   // lectura ADC sobre la superficie
  uint8_t lineLevel = 215;     // lectura ADC sobre la línea blanca
  double spotRadius = 0.0035;  // radio del área que ve cada QRE1113
  double noiseSigma = 4.0;     // ruido gaussiano en cuentas ADC
  double gainSpread = 0.12;    // dispersión de ganancia entre sensores
  uint32_t adcReadUs = 20;     // tiempo de conversión + multiplexor
};

class SimLineSensorBus : public atlas::ILineSensorBus {
 public:
  SimLineSensorBus(const Track& track, const RobotModel& robot, SimClock& clock, const SensorModel& model,
                   atlas::LineColor trackLineColor, uint32_t seed);

  uint8_t read(uint8_t channel) override;

 private:
  double reflectance(const Vec2& point) const;

  const Track& track_;
  const RobotModel& robot_;
  SimClock& clock_;
  SensorModel model_;
  atlas::LineColor trackLineColor_;
  std::mt19937 random_;
  std::normal_distribution<double> noise_;
  double gain_[atlas::kSensorCount];
  double offset_[atlas::kSensorCount];
};

class SimMotor : public atlas::IMotor {
 public:
  void setPwm(int16_t pwm) override { pwm_ = pwm; }
  int16_t pwm() const { return pwm_; }

 private:
  int16_t pwm_ = 0;
};

class SimDrivetrain : public atlas::IDrivetrain {
 public:
  void enable() override { enabled_ = true; }
  void disable() override { enabled_ = false; }
  atlas::IMotor& left() override { return left_; }
  atlas::IMotor& right() override { return right_; }
  atlas::IMotor& impeller() override { return impeller_; }

  DriveInputs inputs() const { return {left_.pwm(), right_.pwm(), impeller_.pwm(), enabled_}; }

 private:
  bool enabled_ = false;
  SimMotor left_;
  SimMotor right_;
  SimMotor impeller_;
};

class SimOperatorPanel : public atlas::IOperatorInput, public atlas::IIndicators {
 public:
  bool button1() override { return button1_; }
  bool button2() override { return button2_; }
  bool startReady() override { return ready_; }
  bool startGo() override { return go_; }
  void setLed(atlas::Led led, bool on) override { leds_[static_cast<int>(led)] = on; }

  void pressButton1(bool pressed) { button1_ = pressed; }
  void pressButton2(bool pressed) { button2_ = pressed; }
  void setReady(bool on) { ready_ = on; }
  void setGo(bool on) { go_ = on; }
  bool led(atlas::Led led) const { return leds_[static_cast<int>(led)]; }

 private:
  bool button1_ = false;
  bool button2_ = false;
  bool ready_ = false;
  bool go_ = false;
  bool leds_[3] = {false, false, false};
};

}  // namespace sim
