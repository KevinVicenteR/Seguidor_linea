/*
  LineFollower.h - Control de carrera: rampas de arranque/frenado,
  seguimiento PD y recuperación cuando se pierde la línea.

  No bloquea: el dueño llama update() una vez por periodo de control.
*/
#pragma once

#include <stdint.h>

#include "Config.h"
#include "Hal.h"
#include "LineSensorArray.h"
#include "PdController.h"
#include "Ramp.h"

namespace atlas {

struct MotorCommand {
  int16_t left = 0;
  int16_t right = 0;
  int16_t impeller = 0;
};

enum class Side : uint8_t { Left, Right };

class LineFollower {
 public:
  LineFollower(LineSensorArray& sensors, IDrivetrain& drivetrain, const RobotConfig& config);

  void start(int16_t cruisePwm, uint32_t nowMs);
  void requestStop(uint32_t nowMs);
  void update(uint32_t nowMs);

  bool isRunning() const { return running_; }
  bool isStopping() const { return stopping_; }
  const MotorCommand& lastCommand() const { return command_; }
  const LineReading& lastReading() const { return reading_; }

 private:
  int16_t currentVelocity(uint32_t nowMs) const;
  MotorCommand followLine(int16_t velocity, uint32_t nowMs);
  MotorCommand recoverLine(uint32_t nowMs) const;
  void rememberLastSide();
  int16_t clampWheel(int16_t pwm) const;
  void apply(const MotorCommand& command);

  LineSensorArray& sensors_;
  IDrivetrain& drivetrain_;
  const RobotConfig& config_;
  PdController pd_;

  LinearRamp velocityRamp_;
  LinearRamp impellerRamp_;
  LinearRamp brakeRamp_;

  bool running_ = false;
  bool stopping_ = false;
  uint32_t lastOnLineMs_ = 0;
  Side lastSide_ = Side::Left;

  LineReading reading_;
  MotorCommand command_;
};

}  // namespace atlas
