#include "RaceMonitor.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace sim {

constexpr double RaceMonitor::kOffTrackDistance;
constexpr double RaceMonitor::kOffTrackGraceS;
constexpr double RaceMonitor::kStallWindowS;
constexpr double RaceMonitor::kStallMinProgress;
constexpr double RaceMonitor::kTelemetryPeriodS;
constexpr double RaceMonitor::kZigzagHysteresis;

RaceMonitor::RaceMonitor(const Track& track, double startArcLength, int laps)
    : track_(track), laps_(laps), lastArcLength_(startArcLength) {}

void RaceMonitor::start(double timeS) {
  racing_ = true;
  startTime_ = lastTime_ = stallCheckTime_ = nextTelemetryTime_ = timeS;
}

void RaceMonitor::update(double timeS, const RobotModel& robot, const DriveInputs& inputs, bool trackable) {
  if (!racing_) return;
  lastTime_ = timeS;

  const TrackProjection projection =
      track_.projectNear(robot.sensorBarCenter(), lastArcLength_, 0.15, 0.6);
  trackProgress(timeS, projection);
  checkFailures(timeS, projection);

  samples_++;
  if (trackable) trackableSamples_++;
  if (robot.isSliding()) slidingSamples_++;
  sumSquaredOffset_ += projection.distance * projection.distance;
  maxOffset_ = std::max(maxOffset_, projection.distance);
  maxSpeed_ = std::max(maxSpeed_, robot.speed());

  const double steering = (inputs.leftPwm - inputs.rightPwm) / 2.0;
  sumSquaredSteering_ += steering * steering;
  const int side = projection.signedOffset > kZigzagHysteresis ? 1 : (projection.signedOffset < -kZigzagHysteresis ? -1 : 0);
  if (side != 0 && offsetSide_ != 0 && side != offsetSide_) zigzags_++;
  if (side != 0) offsetSide_ = side;

  if (timeS >= nextTelemetryTime_) {
    nextTelemetryTime_ += kTelemetryPeriodS;
    TelemetrySample sample;
    sample.time = timeS - startTime_;
    sample.x = robot.pose().position.x;
    sample.y = robot.pose().position.y;
    sample.heading = robot.pose().heading;
    sample.speed = robot.speed();
    sample.lateralOffset = projection.signedOffset;
    sample.trackable = trackable;
    sample.sliding = robot.isSliding();
    sample.leftPwm = static_cast<int16_t>(inputs.leftPwm);
    sample.rightPwm = static_cast<int16_t>(inputs.rightPwm);
    telemetry_.push_back(sample);
  }

  if (finished_ || hasFailed()) racing_ = false;
}

// El avance se integra con la proyección "cercana" (ventana alrededor de la
// última posición), así un cruce nunca hace saltar el progreso a otra rama.
void RaceMonitor::trackProgress(double timeS, const TrackProjection& projection) {
  if (projection.distance > kOffTrackDistance) return;

  double delta = projection.arcLength - lastArcLength_;
  if (delta > track_.length() / 2.0) delta -= track_.length();
  if (delta < -track_.length() / 2.0) delta += track_.length();
  progress_ += delta;
  lastArcLength_ = projection.arcLength;

  while (static_cast<int>(lapTimes_.size()) < laps_ &&
         progress_ >= track_.length() * static_cast<double>(lapTimes_.size() + 1)) {
    const double previous = lapTimes_.empty() ? 0.0 : std::accumulate(lapTimes_.begin(), lapTimes_.end(), 0.0);
    lapTimes_.push_back(timeS - startTime_ - previous);
  }
  if (static_cast<int>(lapTimes_.size()) == laps_) finished_ = true;
}

void RaceMonitor::checkFailures(double timeS, const TrackProjection& projection) {
  if (projection.distance > kOffTrackDistance) {
    if (offTrackSince_ < 0.0) offTrackSince_ = timeS;
    if (timeS - offTrackSince_ > kOffTrackGraceS) fail("Salió de la pista");
  } else {
    offTrackSince_ = -1.0;
  }

  if (progress_ < -0.5) fail("Avanza en sentido contrario");

  if (timeS - stallCheckTime_ >= kStallWindowS) {
    if (progress_ - stallCheckProgress_ < kStallMinProgress) fail("Sin avance (atascado o girando en el lugar)");
    stallCheckTime_ = timeS;
    stallCheckProgress_ = progress_;
  }
}

void RaceMonitor::fail(const std::string& reason) {
  if (failureReason_.empty()) failureReason_ = reason;
}

void RaceMonitor::fillResult(ScenarioResult& result) const {
  result.completed = finished_;
  result.failureReason = failureReason_;
  result.lapTimes = lapTimes_;
  result.raceTime = lastTime_ - startTime_;
  result.distance = std::max(0.0, progress_);
  result.averageSpeed = result.raceTime > 0.0 ? result.distance / result.raceTime : 0.0;
  result.maxSpeed = maxSpeed_;
  result.rmsOffsetMm = samples_ ? 1000.0 * std::sqrt(sumSquaredOffset_ / samples_) : 0.0;
  result.maxOffsetMm = 1000.0 * maxOffset_;
  result.lostLinePercent = samples_ ? 100.0 * (samples_ - trackableSamples_) / samples_ : 0.0;
  result.slidingPercent = samples_ ? 100.0 * slidingSamples_ / samples_ : 0.0;
  result.steeringRmsPwm = samples_ ? std::sqrt(sumSquaredSteering_ / samples_) : 0.0;
  result.zigzagPerMeter = result.distance > 0.0 ? zigzags_ / result.distance : 0.0;
  result.telemetry = telemetry_;
}

}  // namespace sim
