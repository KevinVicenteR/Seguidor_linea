/*
  AtlasApp.h - Máquina de estados de la secuencia de operación del manual:

    Boot -> SelectLineColor -> Calibrating -> Ready -> Running -> Finished
             (SW1 blanca / SW2 negra)        (velocidad, READY, GO)

  tick() se llama continuamente desde loop(). Las animaciones cortas siguen
  siendo bloqueantes (igual que el original) a través de IClock::delayMs.
*/
#pragma once

#include <stdint.h>

#include "Config.h"
#include "Hal.h"
#include "LineFollower.h"
#include "LineSensorArray.h"
#include "Ramp.h"
#include "StatusLeds.h"

namespace atlas {

enum class AppState : uint8_t { Boot, SelectLineColor, Calibrating, Ready, Running, Finished };

class AtlasApp {
 public:
  AtlasApp(IClock& clock, IOperatorInput& input, IIndicators& indicators, LineSensorArray& sensors,
           IDrivetrain& drivetrain, LineFollower& follower, const RobotConfig& config,
           IDiagnostics* diagnostics = nullptr);

  void begin();
  void tick();

  AppState state() const { return state_; }
  int16_t cruisePwm() const { return cruisePwm_; }

 private:
  bool anyButton() { return input_.button1() || input_.button2(); }

  void selectLineColor();
  void calibrate();
  void prepareToRun();
  void run();
  void showFinished();

  void startCalibration(LineColor color);
  void finishCalibration();
  void updateImpellerPreSpin();
  void adjustCruiseSpeed();
  void startRace();
  void shutdown();

  IClock& clock_;
  IOperatorInput& input_;
  StatusLeds leds_;
  LineSensorArray& sensors_;
  IDrivetrain& drivetrain_;
  LineFollower& follower_;
  const RobotConfig& config_;
  IDiagnostics* diagnostics_;

  AppState state_ = AppState::Boot;
  int16_t cruisePwm_;
  bool calibrationBlink_ = true;
  bool preSpinActive_ = false;
  LinearRamp preSpinRamp_;
  uint32_t lastControlUs_ = 0;
  uint32_t finishedSinceMs_ = 0;
};

}  // namespace atlas
