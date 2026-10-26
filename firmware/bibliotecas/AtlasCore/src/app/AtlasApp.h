#pragma once

#include <stdint.h>

#include "Config.h"
#include "CalibrationStore.h"
#include "Hal.h"
#include "LineFollower.h"
#include "LineSensorArray.h"
#include "RunLog.h"
#include "SpeedSettingStore.h"
#include "TrackMap.h"
#include "TrackMapStore.h"
#include "Ramp.h"
#include "StatusLeds.h"

namespace atlas {

struct AppOptions {
  IDiagnostics* diagnostics = nullptr;
  const RobotConfig* fastConfig = nullptr;
  IPersistentStorage* storage = nullptr;
  TrackMap* trackMap = nullptr;
};

enum class AppState : uint8_t { Boot, WaitCalibration, Calibrating, Ready, Running };

class AtlasApp {
 public:
  AtlasApp(IClock& clock, IOperatorInput& input, IIndicators& indicators, LineSensorArray& sensors,
           IDrivetrain& drivetrain, LineFollower& follower, const RobotConfig& config,
           const AppOptions& options = AppOptions());

  void begin();
  void tick();

  AppState state() const { return state_; }
  int16_t cruisePwm() const { return cruisePwm_; }
  uint16_t racesCompleted() const { return racesCompleted_; }
  bool hasTrackMap() const { return trackMap_ != nullptr && trackMap_->isValid(); }

 private:
  bool anyButton() { return input_.button1() || input_.button2(); }

  void waitCalibration();
  void calibrate();
  void prepareToRun();
  void run();

  bool loadSavedCalibration();
  void loadSavedSpeed();
  void saveSpeed();
  void forgetTrackMap();
  void saveTrackMapIfLearned();
  void logRace();
  void startCalibration();
  void finishCalibration();
  void updateReadyIndicator();
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
  const RobotConfig* fastConfig_;
  IPersistentStorage* storage_;
  TrackMap* trackMap_;

  AppState state_ = AppState::Boot;
  int16_t cruisePwm_;
  bool calibrationBlink_ = true;
  bool preSpinActive_ = false;
  LinearRamp preSpinRamp_;
  uint32_t lastControlUs_ = 0;
  uint16_t racesCompleted_ = 0;
  uint32_t raceStartMs_ = 0;
  uint32_t raceStopMs_ = 0;
};

}
