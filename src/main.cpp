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
#include "AvrLineSensorBus.h"
#include "AvrOperatorPanel.h"
#include "Config.h"
#include "LineFollower.h"
#include "LineSensorArray.h"

using namespace atlas;

//  =============================
//  U S E R   P A R A M E T E R S
//  =============================

constexpr MotorPolarity kLeftMotorPolarity = MotorPolarity::Normal;
constexpr MotorPolarity kRightMotorPolarity = MotorPolarity::Normal;

RobotConfig makeRobotConfig() {
  RobotConfig config;

  // Control PD optimizado en simulación para precisión a alta velocidad
  // (simulation/tools/atlas_tune). Originales: kp 0.05, kd 0.35, sin filtro.
  config.control.kp = 0.09f;
  config.control.kd = 1.15f;
  config.control.derivativeFilterShift = 2;  // suaviza el derivativo (0 = sin filtro)
  config.control.curveSlowdownPercent = 8;   // frena en curva según el error
  config.control.maxForwardPwm = 255;        // original: 200 (limitaba la velocidad a ~2.5 m/s)
  config.control.maxBackwardPwm = -210;      // freno fuerte de la rueda interior al buscar la línea
  config.lostLine.innerMotorPwm = -210;
  config.lostLine.innerBrakeMs = 45;

  config.sensors.trackNearestCluster = true;  // cruces oblicuos y rizos; false = algoritmo original

  config.speed.basePwm = 100;
  config.speed.minPwm = 60;
  config.speed.maxPwm = 250;
  config.speed.stepPwm = 10;
  config.speed.runRampMs = 150;

  config.impeller.pwm = 200;  // el manual recomienda no superar 215
  config.impeller.runRampMs = 250;

  return config;
}

//  ===========================
//  O B J E C T   W I R I N G
//  ===========================

const RobotConfig kConfig = makeRobotConfig();

AvrClock robotClock;
AvrOperatorPanel panel;
AvrLineSensorBus sensorBus;
AvrDrivetrain drivetrain(kLeftMotorPolarity, kRightMotorPolarity, kConfig.motorsEnabled);

LineSensorArray sensors(sensorBus, kConfig.sensors);
LineFollower follower(sensors, drivetrain, kConfig);
AvrDiagnostics diagnostics(robotClock, panel, panel, sensors, drivetrain, kConfig);
AtlasApp app(robotClock, panel, panel, sensors, drivetrain, follower, kConfig, &diagnostics);

void setup() {
  cli();
  panel.begin();
  sensorBus.begin();
  drivetrain.begin();
  sei();

  app.begin();
}

void loop() { app.tick(); }
