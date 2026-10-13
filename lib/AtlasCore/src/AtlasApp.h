/*
  AtlasApp.h - Máquina de estados de la secuencia de operación:

    Boot -> WaitCalibration -> Calibrating -> Ready <-> Running
            (SW1 o SW2: línea negra)    |      |  GO     quitar GO
                    ^-------------------+      |
                    calibración incompleta     +-- SW1 + SW2 juntos: recalibrar

  - Calibración válida: animación de confirmación y LED0 + LED1 fijos.
  - Quitar GO frena y regresa a Ready conservando calibración y velocidad:
    se puede volver a dar READY/GO sin recalibrar. RESET borra todo.

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

enum class AppState : uint8_t { Boot, WaitCalibration, Calibrating, Ready, Running };

class AtlasApp {
 public:
  AtlasApp(IClock& clock, IOperatorInput& input, IIndicators& indicators, LineSensorArray& sensors,
           IDrivetrain& drivetrain, LineFollower& follower, const RobotConfig& config,
           IDiagnostics* diagnostics = nullptr);

  void begin();
  void tick();

  AppState state() const { return state_; }
  int16_t cruisePwm() const { return cruisePwm_; }
  uint16_t racesCompleted() const { return racesCompleted_; }

 private:
  bool anyButton() { return input_.button1() || input_.button2(); }

  void waitCalibration();
  void calibrate();
  void prepareToRun();
  void run();

  void startCalibration();
  void finishCalibration();
  void restoreCalibratedIndicator();
  void updateImpellerPreSpin();
  void adjustCruiseSpeed();
  void startRace();
  void finishRace();
  bool recalibrationRequested();

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
  uint16_t racesCompleted_ = 0;
};

}  // namespace atlas
