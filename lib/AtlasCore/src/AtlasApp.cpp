#include "AtlasApp.h"

namespace atlas {

namespace {
constexpr uint16_t kUiBlinkMs = 250;
constexpr uint16_t kConfirmHalfPeriodMs = 250;
constexpr uint16_t kStopSettleMs = 500;
constexpr uint16_t kFastIndicatorHalfPeriodMs = 300;
constexpr uint8_t kLapResetPulses = 3;        // pulsos de READY (PROGRAM del control)
constexpr uint16_t kLapResetWindowMs = 5000;  // deben caber en esta ventana
}  // namespace

AtlasApp::AtlasApp(IClock& clock, IOperatorInput& input, IIndicators& indicators, LineSensorArray& sensors,
                   IDrivetrain& drivetrain, LineFollower& follower, const RobotConfig& config,
                   const AppOptions& options)
    : clock_(clock),
      input_(input),
      leds_(indicators, clock),
      sensors_(sensors),
      drivetrain_(drivetrain),
      follower_(follower),
      config_(config),
      diagnostics_(options.diagnostics),
      fastConfig_(options.fastConfig),
      storage_(options.storage),
      trackMap_(options.trackMap),
      cruisePwm_(config.speed.basePwm) {
  follower_.attachTrackMap(trackMap_);
}

void AtlasApp::begin() {
  leds_.bootAnimation();

  // Mantener SW1 o SW2 durante la animación de arranque abre el diagnóstico.
  if (diagnostics_ != nullptr && anyButton()) {
    leds_.confirm(500, 1);
    diagnostics_->run();
  }
  loadSavedSpeed();
  state_ = loadSavedCalibration() ? AppState::Ready : AppState::WaitCalibration;
}

// Velocidad elegida la última vez con SW1/SW2 (si está dentro del rango).
void AtlasApp::loadSavedSpeed() {
  if (storage_ == nullptr) return;
  int16_t saved = 0;
  if (!SpeedSettingStore(*storage_).load(&saved)) return;
  if (saved >= config_.speed.minPwm && saved <= config_.speed.maxPwm) cruisePwm_ = saved;
}

void AtlasApp::saveSpeed() {
  if (storage_ != nullptr) SpeedSettingStore(*storage_).save(cruisePwm_);
}

// Usa la última calibración guardada si existe y sigue siendo válida.
bool AtlasApp::loadSavedCalibration() {
  if (storage_ == nullptr) return false;
  if (!CalibrationStore(*storage_).load(sensors_) || !sensors_.isCalibrationValid()) {
    sensors_.resetCalibration();
    return false;
  }
  if (trackMap_ != nullptr && !TrackMapStore(*storage_).load(*trackMap_)) trackMap_->clear();
  leds_.blink(Led::Led2, kUiBlinkMs);
  leds_.blink(Led::Led2, kUiBlinkMs);
  updateReadyIndicator();
  return true;
}

void AtlasApp::tick() {
  switch (state_) {
    case AppState::Boot:
      begin();
      break;
    case AppState::WaitCalibration:
      waitCalibration();
      break;
    case AppState::Calibrating:
      calibrate();
      break;
    case AppState::Ready:
      prepareToRun();
      break;
    case AppState::Running:
      run();
      break;
  }
}

//  ---------------------------------------------------------------------------
//  Calibración
//  ---------------------------------------------------------------------------

// Solo se calibra línea negra sobre fondo blanco: cualquier botón inicia.
void AtlasApp::waitCalibration() {
  if (anyButton()) startCalibration();
}

// Una calibración nueva implica pista o lugar nuevo: el mapa ya no sirve.
void AtlasApp::forgetTrackMap() {
  if (trackMap_ == nullptr) return;
  trackMap_->clear();
  if (storage_ != nullptr) TrackMapStore(*storage_).erase();
}

void AtlasApp::saveTrackMapIfLearned() {
  const bool wasMapping = follower_.mode() == RunMode::Mapping;
  const bool learned = follower_.finishMapping();
  if (wasMapping && !learned) {
    leds_.mapRejected();
    if (storage_ != nullptr) TrackMapStore(*storage_).erase();
    return;
  }
  const bool corrected = follower_.correctMapAfterFailedRun();
  if ((learned || corrected) && storage_ != nullptr) TrackMapStore(*storage_).save(*trackMap_);
}

// Registra la carrera (tipo, tiempo, si falló y dónde) para el análisis.
void AtlasApp::logRace() {
  if (storage_ == nullptr) return;
  RunRecord record;
  record.mode = static_cast<uint8_t>(follower_.mode());
  record.failed = follower_.runFailed();
  const uint32_t centiseconds = (raceStopMs_ - raceStartMs_) / 10;
  record.timeCentiseconds = centiseconds > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(centiseconds);
  record.failureBin = follower_.failureBin();
  record.mapLength = trackMap_ != nullptr ? trackMap_->length() : 0;
  RunLog(*storage_).append(record);
}

void AtlasApp::startCalibration() {
  forgetTrackMap();
  leds_.blink(Led::Led2, kUiBlinkMs);
  sensors_.resetCalibration();
  state_ = AppState::Calibrating;
}

void AtlasApp::calibrate() {
  if (anyButton()) {
    finishCalibration();
    return;
  }

  // Línea negra: parpadean LED0 y LED1. Línea blanca: parpadea LED2.
  const bool black = sensors_.lineColor() == LineColor::Black;
  leds_.set(Led::Led0, black && calibrationBlink_);
  leds_.set(Led::Led1, black && calibrationBlink_);
  leds_.set(Led::Led2, !black && calibrationBlink_);
  calibrationBlink_ = !calibrationBlink_;

  sensors_.collectCalibrationSamples();
}

void AtlasApp::finishCalibration() {
  sensors_.finishCalibration();
  leds_.setAll(false);
  clock_.delayMs(kUiBlinkMs);

  if (!sensors_.isCalibrationValid()) {
    leds_.calibrationError();
    state_ = AppState::WaitCalibration;
    return;
  }

  leds_.confirm(kConfirmHalfPeriodMs, 2);
  if (sensors_.calibratedSensorCount() < kSensorCount) leds_.damagedSensorsWarning();
  if (storage_ != nullptr) CalibrationStore(*storage_).save(sensors_);
  updateReadyIndicator();
  clock_.delayMs(kUiBlinkMs);
  state_ = AppState::Ready;
}

// Sin mapa (reconocimiento): LED0 + LED1 fijos. Con mapa (Rápido): alternados.
void AtlasApp::updateReadyIndicator() {
  if (!hasTrackMap()) {
    leds_.showCalibrated();
    return;
  }
  const bool phase = (clock_.millis() / kFastIndicatorHalfPeriodMs) % 2;
  leds_.set(Led::Led0, phase);
  leds_.set(Led::Led1, !phase);
}

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
  if (lapResetRequested()) {
    returnToFirstLap();
    return;
  }
  updateImpellerPreSpin();
  adjustCruiseSpeed();
  updateReadyIndicator();
}

// SW1 + SW2 juntos en Ready: repetir la calibración sin presionar RESET.
bool AtlasApp::recalibrationRequested() { return input_.button1() && input_.button2(); }

// PROGRAM del control remoto (señal READY) 3 veces seguidas, sin dar GO, en
// menos de kLapResetWindowMs: borrar el mapa y volver a la vuelta de
// reconocimiento. Un READY normal antes de GO es un solo pulso.
bool AtlasApp::lapResetRequested() {
  const bool readyOn = input_.startReady();
  const uint32_t now = clock_.millis();
  const bool pulseEnded = readyWasOn_ && !readyOn;
  readyWasOn_ = readyOn;
  if (readyPulses_ > 0 && now - firstReadyPulseMs_ > kLapResetWindowMs) readyPulses_ = 0;
  if (!pulseEnded) return false;
  if (readyPulses_ == 0) firstReadyPulseMs_ = now;
  readyPulses_++;
  return readyPulses_ >= kLapResetPulses;
}

// Conserva la calibración y la velocidad; solo olvida el mapa.
void AtlasApp::returnToFirstLap() {
  readyPulses_ = 0;
  drivetrain_.impeller().setPwm(0);
  forgetTrackMap();
  leds_.mapRejected();  // 4 destellos de LED2; luego LED0 + LED1 fijos = vuelta 1
  updateReadyIndicator();
}

void AtlasApp::updateImpellerPreSpin() {
  const uint32_t now = clock_.millis();

  if (!config_.impeller.spinWithReady || !input_.startReady()) {
    preSpinActive_ = false;
    leds_.set(Led::Led2, false);
    drivetrain_.impeller().setPwm(0);
    return;
  }

  if (!preSpinActive_) {
    preSpinRamp_.start(now, config_.impeller.startRampMs, 0, config_.impeller.pwm);
    preSpinActive_ = true;
  }
  drivetrain_.impeller().setPwm(preSpinRamp_.valueAt(now));
  leds_.set(Led::Led2, preSpinRamp_.isFinished(now));
}

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
  readyPulses_ = 0;
  drivetrain_.impeller().setPwm(0);
  leds_.setAll(true);
  const bool guided = hasTrackMap();
  follower_.useConfig(guided && fastConfig_ != nullptr ? *fastConfig_ : config_);
  const RunMode mode = trackMap_ == nullptr ? RunMode::Plain : (guided ? RunMode::MapGuided : RunMode::Mapping);
  raceStartMs_ = clock_.millis();
  raceStopMs_ = raceStartMs_;
  follower_.start(cruisePwm_, raceStartMs_, mode);
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
  preSpinActive_ = false;
  leds_.setAll(false);
  updateReadyIndicator();
  state_ = AppState::Ready;
}


}  // namespace atlas
