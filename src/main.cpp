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
#include "FirmwareConfig.h"
#include "TrackMap.h"
#ifdef ATLAS_LOG
#include "AvrRaceLogger.h"
#endif

using namespace atlas;


constexpr MotorPolarity kLeftMotorPolarity = MotorPolarity::Normal;
constexpr MotorPolarity kRightMotorPolarity = MotorPolarity::Normal;

// Ajustes compartidos con el modo --firmware del simulador.
// constexpr: calculadas al compilar (sin código de inicialización al arrancar).
constexpr RobotConfig kSafeConfig = firmwareSafeConfig();
constexpr RobotConfig kFastConfig = firmwareFastConfig();


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

#ifdef ATLAS_LOG
AvrRaceLogger raceLogger(4);  // una trama cada 4 ciclos de control (~3 ms)
#endif

void setup() {
  cli();
  panel.begin();
  sensorBus.begin();
  drivetrain.begin();
  sei();

#ifdef ATLAS_LOG
  raceLogger.begin();
#endif
  app.begin();
}

void loop() {
  app.tick();
#ifdef ATLAS_LOG
  raceLogger.update(follower, robotClock.millis());
#endif
}
