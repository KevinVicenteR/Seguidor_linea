/*
  main.cpp - Firmware orientado a objetos para el seguidor de línea
  ATLAS Rev. 1.4 (ATmega328P, 16 sensores, BTN9960LV, control PD).

  Refactorización del firmware ATLAS 1.4.3 de EXOTIC TEAM MX:
    - lib/AtlasCore: configuración y lógica de control independiente del
                     hardware (se compila igual en el robot y en el simulador).
    - lib/AtlasAvr : implementación de las interfaces sobre registros AVR.

  Subir con PlatformIO:  pio run -e atlas -t upload   (programador USBasp)
*/

#include <Arduino.h>

#include "AtlasApp.h"
#include "AvrClock.h"
#include "AvrDiagnostics.h"
#include "AvrDrivetrain.h"
#include "AvrEepromStorage.h"
#include "AvrLineSensorBus.h"
#include "AvrOperatorPanel.h"
#include "Config.h"
#include "LineFollower.h"
#include "LineSensorArray.h"
#include "Profiles.h"
#include "TrackMap.h"

using namespace atlas;


constexpr MotorPolarity kLeftMotorPolarity = MotorPolarity::Normal;
constexpr MotorPolarity kRightMotorPolarity = MotorPolarity::Normal;


void applyUserSettings(RobotConfig& config) {
  config.speed.basePwm = 170;
  config.speed.minPwm = 60;
  config.speed.maxPwm = 250;
  config.speed.stepPwm = 10;
  config.speed.runRampMs = 150;

  config.impeller.pwm = 190;  
  config.impeller.runRampMs = 250;
}

RobotConfig makeConfig(RobotConfig profile) {
  applyUserSettings(profile);
  return profile;
}

// Vueltas 2 y 3 (con mapa): van más rápido, la turbina aprieta más para
// tener agarre en las curvas.
RobotConfig makeFastConfig() {
  RobotConfig config = makeConfig(fastProfile());
  config.impeller.pwm = 235;
  return config;
}


const RobotConfig kSafeConfig = makeConfig(safeProfile());  // 1er intento: reconocimiento
const RobotConfig kFastConfig = makeFastConfig();  // intentos con mapa aprendido

AvrClock robotClock;
AvrOperatorPanel panel;
AvrLineSensorBus sensorBus;
AvrEepromStorage eeprom;
TrackMap trackMap;
AvrDrivetrain drivetrain(kLeftMotorPolarity, kRightMotorPolarity, kSafeConfig.motorsEnabled);

LineSensorArray sensors(sensorBus, kSafeConfig.sensors);
LineFollower follower(sensors, drivetrain, kSafeConfig);
AvrDiagnostics diagnostics(robotClock, panel, panel, sensors, drivetrain, kSafeConfig, &eeprom, &trackMap);

AppOptions makeAppOptions() {
  AppOptions options;
  options.diagnostics = &diagnostics;
  options.fastConfig = &kFastConfig;
  options.storage = &eeprom;
  options.trackMap = &trackMap;
  return options;
}

AtlasApp app(robotClock, panel, panel, sensors, drivetrain, follower, kSafeConfig, makeAppOptions());

void setup() {
  cli();
  panel.begin();
  sensorBus.begin();
  drivetrain.begin();
  sei();

  app.begin();
}

void loop() { app.tick(); }
