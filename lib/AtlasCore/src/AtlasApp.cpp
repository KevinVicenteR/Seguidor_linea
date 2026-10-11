#include "AtlasApp.h"

namespace atlas {

namespace {
constexpr uint16_t kUiBlinkMs = 250;
constexpr uint16_t kConfirmHalfPeriodMs = 250;
constexpr uint16_t kShutdownSettleMs = 2000;
constexpr uint16_t kStandbyFlashMs = 50;
constexpr uint16_t kStandbyPeriodMs = 1050;
}  // namespace

AtlasApp::AtlasApp(IClock& clock, IOperatorInput& input, IIndicators& indicators, LineSensorArray& sensors,
                   IDrivetrain& drivetrain, LineFollower& follower, const RobotConfig& config,
                   IDiagnostics* diagnostics)
    : clock_(clock),
      input_(input),
      leds_(indicators, clock),
      sensors_(sensors),
      drivetrain_(drivetrain),
      follower_(follower),
      config_(config),
      diagnostics_(diagnostics),
      cruisePwm_(config.speed.basePwm) {}

void AtlasApp::begin() {
  leds_.bootAnimation();

  // Mantener SW1 o SW2 durante la animación de arranque abre el diagnóstico.
  if (diagnostics_ != nullptr && anyButton()) {
    leds_.confirm(500, 1);
    diagnostics_->run();
  }
  state_ = AppState::SelectLineColor;
}

void AtlasApp::tick() {
  switch (state_) {
    case AppState::Boot:
      begin();
      break;
    case AppState::SelectLineColor:
      selectLineColor();
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
    case AppState::Finished:
      showFinished();
      break;
  }
}

//  ---------------------------------------------------------------------------
//  Calibración
//  ---------------------------------------------------------------------------

void AtlasApp::selectLineColor() {
  if (!anyButton()) return;
  startCalibration(input_.button2() ? LineColor::Black : LineColor::White);
}

void AtlasApp::startCalibration(LineColor color) {
  sensors_.setLineColor(color);
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
  leds_.confirm(kConfirmHalfPeriodMs, 2);
  clock_.delayMs(kUiBlinkMs);
  state_ = AppState::Ready;
}

//  ---------------------------------------------------------------------------
//  Preparación: selección de velocidad y pre-giro del impeller con READY
//  ---------------------------------------------------------------------------

void AtlasApp::prepareToRun() {
  if (input_.startGo()) {
    startRace();
    return;
  }
  updateImpellerPreSpin();
  adjustCruiseSpeed();
}

void AtlasApp::updateImpellerPreSpin() {
  const uint32_t now = clock_.millis();

  if (!input_.startReady()) {
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
  } else if (input_.button2() && cruisePwm_ <= speed.maxPwm - speed.stepPwm) {
    leds_.blink(Led::Led1, kUiBlinkMs);
    cruisePwm_ += speed.stepPwm;
  }
}

//  ---------------------------------------------------------------------------
//  Carrera
//  ---------------------------------------------------------------------------

void AtlasApp::startRace() {
  drivetrain_.impeller().setPwm(0);
  leds_.setAll(true);
  follower_.start(cruisePwm_, clock_.millis());
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
    follower_.requestStop(clock_.millis());
  }

  if (!follower_.isRunning()) shutdown();
}

void AtlasApp::shutdown() {
  drivetrain_.left().setPwm(0);
  drivetrain_.right().setPwm(0);
  drivetrain_.impeller().setPwm(0);
  drivetrain_.disable();
  clock_.delayMs(kShutdownSettleMs);
  finishedSinceMs_ = clock_.millis();
  state_ = AppState::Finished;
}

void AtlasApp::showFinished() {
  const bool flash = (clock_.millis() - finishedSinceMs_) % kStandbyPeriodMs < kStandbyFlashMs;
  leds_.set(Led::Led0, flash);
  leds_.set(Led::Led1, flash);
}

}  // namespace atlas
