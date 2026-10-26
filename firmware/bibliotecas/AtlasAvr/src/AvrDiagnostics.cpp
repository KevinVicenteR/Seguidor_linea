#include "AvrDiagnostics.h"

#include "LogExporter.h"
#include "Ramp.h"
#include "RunLog.h"
#include "TrackMapStore.h"

namespace atlas {

namespace {
constexpr uint8_t kTestCount = 5;

class SerialSink : public ITextSink {
 public:
  void text(const char* value) override { Serial.print(value); }
  void number(int32_t value) override { Serial.print(value); }
  void endLine() override { Serial.println(); }
};
constexpr uint16_t kDebounceMs = 250;
constexpr uint16_t kMotorOnMs = 400;
constexpr uint16_t kMotorOffMs = 1000;
constexpr int16_t kMotorTestPwm = 60;
}

AvrDiagnostics::AvrDiagnostics(IClock& clock, IOperatorInput& input, IIndicators& indicators,
                               LineSensorArray& sensors, IDrivetrain& drivetrain, const RobotConfig& config,
                               IPersistentStorage* storage, TrackMap* trackMap)
    : clock_(clock),
      input_(input),
      leds_(indicators, clock),
      sensors_(sensors),
      drivetrain_(drivetrain),
      config_(config),
      storage_(storage),
      trackMap_(trackMap) {}

void AvrDiagnostics::run() {
  Serial.begin(115200);
  Serial.println(F("\n=== ATLAS OOP - MODO DIAGNOSTICO ==="));

  while (true) {
    switch (selectTest()) {
      case 0:
        testWheelMotors();
        break;
      case 1:
        testImpeller();
        break;
      case 2:
        testSensors();
        break;
      case 3:
        testUserInterface();
        break;
      case 4:
        exportRaceLog();
        break;
    }
  }
}

uint8_t AvrDiagnostics::selectTest() {
  Serial.println(F("[0] MOTORES  [1] IMPELLER  [2] SENSORES  [3] INTERFAZ  [4] REGISTRO"));
  Serial.println(F("CONFIRMAR [SW1] / CAMBIAR [SW2]\n> 0"));

  uint8_t selection = 0;
  leds_.showNumber(selection);
  while (!input_.button1()) {
    if (input_.button2()) {
      selection = (selection + 1) % kTestCount;
      leds_.showNumber(selection);
      Serial.print(F("> "));
      Serial.println(selection);
      clock_.delayMs(kDebounceMs);
    }
  }
  leds_.confirm(kDebounceMs, 2);
  return selection;
}

void AvrDiagnostics::waitForAnyButton() {
  while (!anyButton()) {
  }
  leds_.confirm(kDebounceMs, 1);
}

void AvrDiagnostics::testWheelMotors() {
  Serial.println(F("*MOTORES* Ruedas en el aire. Presione un boton."));
  waitForAnyButton();

  drivetrain_.enable();
  pulseMotor(drivetrain_.left(), Led::Led0, kMotorTestPwm);
  pulseMotor(drivetrain_.left(), Led::Led0, -kMotorTestPwm);
  pulseMotor(drivetrain_.right(), Led::Led1, kMotorTestPwm);
  pulseMotor(drivetrain_.right(), Led::Led1, -kMotorTestPwm);
  drivetrain_.disable();
  Serial.println(F("*FIN MOTORES*"));
}

void AvrDiagnostics::pulseMotor(IMotor& motor, Led led, int16_t pwm) {
  leds_.set(led, true);
  motor.setPwm(pwm);
  clock_.delayMs(kMotorOnMs);
  leds_.set(led, false);
  motor.setPwm(0);
  clock_.delayMs(kMotorOffMs);
}

void AvrDiagnostics::testImpeller() {
  Serial.println(F("*IMPELLER* SW2 alterna encendido/apagado. RESET para salir."));
  bool spinning = false;
  LinearRamp ramp;

  while (true) {
    if (!input_.button2()) continue;
    spinning = !spinning;
    Serial.println(spinning ? F("IMP ON") : F("IMP OFF"));

    if (spinning) {
      ramp.start(clock_.millis(), config_.impeller.startRampMs, 0, config_.impeller.pwm);
      while (!ramp.isFinished(clock_.millis())) drivetrain_.impeller().setPwm(ramp.valueAt(clock_.millis()));
    }
    drivetrain_.impeller().setPwm(spinning ? config_.impeller.pwm : 0);
    leds_.set(Led::Led2, spinning);
    clock_.delayMs(kDebounceMs);
  }
}

void AvrDiagnostics::testSensors() {
  Serial.println(F("*SENSORES* CALIBRADO [SW1] / CRUDO [SW2]"));
  while (!anyButton()) {
  }
  const bool calibrated = input_.button1();
  leds_.confirm(kDebounceMs, 1);

  Serial.println(F("Modo: linea negra sobre fondo blanco"));
  if (calibrated) calibrateInteractively();

  while (true) {
    if (calibrated && !anyButton()) {
      sensors_.readCalibrated();
    } else {
      sensors_.readRaw();
    }
    printRow(F(""), sensors_.values());
    delayMicroseconds(config_.control.loopPeriodUs);
  }
}

void AvrDiagnostics::calibrateInteractively() {
  Serial.println(F("Presione un boton para iniciar y otro para terminar la calibracion."));
  waitForAnyButton();
  sensors_.resetCalibration();
  while (!anyButton()) sensors_.collectCalibrationSamples();
  sensors_.finishCalibration();
  leds_.confirm(kDebounceMs, 2);

  printRow(F("MAX:\t"), sensors_.maxValues());
  printRow(F("MIN:\t"), sensors_.minValues());
  printRow(F("TH:\t"), sensors_.thresholds());
  Serial.print(F("Sensores calibrados: "));
  Serial.print(sensors_.calibratedSensorCount());
  Serial.println(sensors_.isCalibrationValid() ? F("/16 OK") : F("/16 INCOMPLETA, repita la pasada"));
  Serial.println(F("Presione un boton para iniciar la lectura."));
  waitForAnyButton();
}

void AvrDiagnostics::printRow(const __FlashStringHelper* label, const uint8_t* values) {
  Serial.print(label);
  for (uint8_t i = 0; i < kSensorCount; i++) {
    Serial.print(values[i]);
    Serial.print('\t');
  }
  Serial.println();
}

void AvrDiagnostics::exportRaceLog() {
  if (storage_ == nullptr || trackMap_ == nullptr) {
    Serial.println(F("Sin memoria para el registro."));
    return;
  }
  TrackMapStore(*storage_).load(*trackMap_);
  RunLog log(*storage_);
  SerialSink sink;
  Serial.println(F("--- copia desde ATLAS-REGISTRO hasta FIN ---"));
  exportRunLog(sink, log, *trackMap_);
  Serial.println(F("SW2 borra el registro de carreras; SW1 regresa al menu."));
  while (!anyButton()) {
  }
  if (input_.button2()) {
    log.clear();
    Serial.println(F("Registro borrado."));
  }
  leds_.confirm(kDebounceMs, 1);
}

void AvrDiagnostics::testUserInterface() {
  Serial.println(F("*INTERFAZ* LED0=SW1  LED1=SW2  LED2=SW1+SW2. RESET para salir."));
  waitForAnyButton();

  while (true) {
    const bool sw1 = input_.button1();
    const bool sw2 = input_.button2();
    leds_.set(Led::Led0, sw1);
    leds_.set(Led::Led1, sw2);
    leds_.set(Led::Led2, sw1 && sw2);

    Serial.print(F("SW1:"));
    Serial.print(sw1);
    Serial.print(F("  SW2:"));
    Serial.print(sw2);
    Serial.print(F("  READY:"));
    Serial.print(input_.startReady());
    Serial.print(F("  GO:"));
    Serial.println(input_.startGo());
  }
}

}
