/*
  RaceMonitor.h - Árbitro de la carrera: mide avance sobre la pista,
  vueltas, error lateral y detecta salidas de pista o atascos.
*/
#pragma once

#include <string>
#include <vector>

#include "RobotModel.h"
#include "Scenario.h"
#include "Track.h"

namespace sim {

class RaceMonitor {
 public:
  RaceMonitor(const Track& track, double startArcLength, int laps);

  void start(double timeS);
  void update(double timeS, const RobotModel& robot, const DriveInputs& inputs, bool trackable);

  bool isRacing() const { return racing_; }
  bool isFinished() const { return finished_; }
  bool hasFailed() const { return !failureReason_.empty(); }
  void fillResult(ScenarioResult& result) const;

 private:
  static constexpr double kOffTrackDistance = 0.12;
  static constexpr double kOffTrackGraceS = 0.4;
  static constexpr double kStallWindowS = 3.0;
  static constexpr double kStallMinProgress = 0.05;
  static constexpr double kTelemetryPeriodS = 0.02;

  void trackProgress(double timeS, const TrackProjection& projection);
  void checkFailures(double timeS, const TrackProjection& projection);
  void fail(const std::string& reason);

  const Track& track_;
  int laps_;
  double lastArcLength_;

  bool racing_ = false;
  bool finished_ = false;
  std::string failureReason_;
  double startTime_ = 0.0;
  double lastTime_ = 0.0;
  double progress_ = 0.0;
  std::vector<double> lapTimes_;

  double offTrackSince_ = -1.0;
  double stallCheckTime_ = 0.0;
  double stallCheckProgress_ = 0.0;
  double nextTelemetryTime_ = 0.0;

  long samples_ = 0;
  long trackableSamples_ = 0;
  long slidingSamples_ = 0;
  double sumSquaredOffset_ = 0.0;
  double maxOffset_ = 0.0;
  double maxSpeed_ = 0.0;
  std::vector<TelemetrySample> telemetry_;
};

}  // namespace sim
