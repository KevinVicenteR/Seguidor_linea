#include "../AtlasApp.h"
#include "../parametros/AppParameters.h"

namespace atlas {

//  ---------------------------------------------------------------------------
//  Preparación: selección de velocidad y pre-giro del impeller con READY
//  ---------------------------------------------------------------------------

void AtlasApp::prepareToRun() {
  if (input_.startGo()) {
    startRace();
    return;
  }
  if (recalibrationRequested()) {
    drivetrain_.impeller().setPwm(0);
    startCalibration();
    return;
  }
  updateImpellerPreSpin();
  adjustCruiseSpeed();
  updateReadyIndicator();
}

// SW1 + SW2 juntos en Ready: repetir la calibración sin presionar RESET.
bool AtlasApp::recalibrationRequested() { return input_.button1() && input_.button2(); }

void AtlasApp::adjustCruiseSpeed() {
  const SpeedConfig& speed = config_.speed;

  if (input_.button1() && cruisePwm_ >= speed.minPwm + speed.stepPwm) {
    leds_.blink(Led::Led0, kUiBlinkMs);
    cruisePwm_ -= speed.stepPwm;
    saveSpeed();
  } else if (input_.button2() && cruisePwm_ <= speed.maxPwm - speed.stepPwm) {
    leds_.blink(Led::Led1, kUiBlinkMs);
    cruisePwm_ += speed.stepPwm;
    saveSpeed();
  }
}

//  ---------------------------------------------------------------------------
//  Carrera
//  ---------------------------------------------------------------------------

void AtlasApp::startRace() {
  leds_.setAll(true);
  const bool guided = hasTrackMap();
  follower_.useConfig(rounds_.config(guided, config_, fastConfig_, thirdConfig_));
  const RunMode mode = rounds_.mode(trackMap_ != nullptr, guided);
  raceStartMs_ = clock_.millis();
  raceStopMs_ = raceStartMs_;
  // Mantener la turbina girando al pasar de PROGRAM/READY a START/GO.
  const int16_t initialImpellerPwm = turbine_.pwmAt(raceStartMs_);
  follower_.start(cruisePwm_, raceStartMs_, mode, initialImpellerPwm);
  turbine_.stop();
  lastControlUs_ = clock_.micros();
  state_ = AppState::Running;
}

void AtlasApp::run() {
  const uint32_t nowUs = clock_.micros();
  if (nowUs - lastControlUs_ >= config_.control.loopPeriodUs) {
    lastControlUs_ = nowUs;
    follower_.update(clock_.millis());
  }

  if (!input_.startGo() && !follower_.isStopping()) {
    leds_.setAll(false);
    raceStopMs_ = clock_.millis();
    follower_.requestStop(raceStopMs_);
  }

  if (!follower_.isRunning()) finishRace();
}

// Detiene todo y vuelve a Ready conservando calibración y velocidad.
void AtlasApp::finishRace() {
  drivetrain_.left().setPwm(0);
  drivetrain_.right().setPwm(0);
  drivetrain_.impeller().setPwm(0);
  drivetrain_.disable();
  clock_.delayMs(kStopSettleMs);

  logRace();
  saveTrackMapIfLearned();
  racesCompleted_++;
  rounds_.finish(follower_.mode() == RunMode::MapGuided);
  turbine_.stop();
  leds_.setAll(false);
  updateReadyIndicator();
  state_ = AppState::Ready;
}


}  // namespace atlas
