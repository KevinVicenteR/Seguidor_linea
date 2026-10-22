/*
  AvrDiagnostics.h - Modo de diagnóstico por consola serial (115200 baud).
  Pruebas: [0] motores, [1] impeller, [2] sensores, [3] interfaz,
  [4] registro de carreras y mapa (para el analizador en la PC).
*/
#pragma once

#include <Arduino.h>

#include "parametros/Config.h"
#include "Hal.h"
#include "LineSensorArray.h"
#include "StatusLeds.h"
#include "TrackMap.h"

namespace atlas {

class AvrDiagnostics : public IDiagnostics {
 public:
  AvrDiagnostics(IClock& clock, IOperatorInput& input, IIndicators& indicators, LineSensorArray& sensors,
                 IDrivetrain& drivetrain, const RobotConfig& config, IPersistentStorage* storage = nullptr,
                 TrackMap* trackMap = nullptr);

  void run() override;

 private:
  uint8_t selectTest();
  void waitForAnyButton();
  bool anyButton() { return input_.button1() || input_.button2(); }

  void testWheelMotors();
  void testImpeller();
  void testSensors();
  void testUserInterface();
  void exportRaceLog();

  void pulseMotor(IMotor& motor, Led led, int16_t pwm);
  void calibrateInteractively();
  void printRow(const __FlashStringHelper* label, const uint8_t* values);

  IClock& clock_;
  IOperatorInput& input_;
  StatusLeds leds_;
  LineSensorArray& sensors_;
  IDrivetrain& drivetrain_;
  const RobotConfig& config_;
  IPersistentStorage* storage_;
  TrackMap* trackMap_;
};

}  // namespace atlas
