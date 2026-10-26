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
AvrRaceLogger raceLogger(4);
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
