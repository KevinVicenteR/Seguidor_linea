#include "ScenarioRunner.h"

#include <chrono>
#include <cmath>
#include <string>

#include "AtlasApp.h"
#include "LineFollower.h"
#include "LineSensorArray.h"
#include "RaceMonitor.h"
#include "RaceOperator.h"

namespace sim {

namespace {

constexpr uint32_t kLoopOverheadUs = 8;     // costo de una iteración de loop()
constexpr double kPhysicsStepS = 100e-6;    // paso fijo de integración
constexpr double kMonitorPeriodS = 1e-3;
constexpr double kHardLimitS = 150.0;
constexpr double kStoppedSpeed = 0.05;
constexpr uint8_t kCalibratedSpan = 40;     // max - min mínimo para un sensor útil

// Une el paso del tiempo virtual con la física, el operador y el árbitro.
class World : public TimeListener {
 public:
  World(RobotModel& robot, SimDrivetrain& drivetrain, RaceOperator& op, RaceMonitor& monitor,
        const atlas::LineFollower& follower)
      : robot_(robot), drivetrain_(drivetrain), operator_(op), monitor_(monitor), follower_(follower) {}

  void onTimeAdvanced(uint64_t nowUs, uint32_t deltaUs) override {
    const double now = nowUs / 1e6;
    operator_.update(now);

    pendingS_ += deltaUs / 1e6;
    while (pendingS_ >= kPhysicsStepS) {
      robot_.step(kPhysicsStepS, drivetrain_.inputs());
      pendingS_ -= kPhysicsStepS;
    }

    if (now >= nextMonitorS_) {
      nextMonitorS_ = now + kMonitorPeriodS;
      monitor_.update(now, robot_, drivetrain_.inputs(), follower_.lastReading().isTrackable());
    }
  }

 private:
  RobotModel& robot_;
  SimDrivetrain& drivetrain_;
  RaceOperator& operator_;
  RaceMonitor& monitor_;
  const atlas::LineFollower& follower_;
  double pendingS_ = 0.0;
  double nextMonitorS_ = 0.0;
};

Pose startPoseFor(const TrackEntry& entry, const RobotParams& params, double* startArcLength) {
  *startArcLength = entry.track.nearestArcLength(entry.startPoint);
  Pose onLine = entry.track.poseAt(*startArcLength);
  Pose robot = onLine;
  robot.position = onLine.position - onLine.forward() * params.sensorBarAhead;
  return robot;
}

int countCalibratedSensors(const atlas::LineSensorArray& sensors) {
  int count = 0;
  for (uint8_t i = 0; i < atlas::kSensorCount; i++) {
    if (sensors.maxValues()[i] >= sensors.minValues()[i] + kCalibratedSpan) count++;
  }
  return count;
}

}  // namespace

ScenarioResult ScenarioRunner::run(const TrackEntry& entry, const ScenarioSpec& spec) const {
  const auto wallStart = std::chrono::steady_clock::now();

  atlas::RobotConfig config;
  config.impeller.pwm = spec.impellerPwm;

  SensorModel sensorModel = sensorModel_;
  sensorModel.noiseSigma = spec.sensorNoiseSigma;

  // Hardware simulado + firmware real
  SimClock clock;
  SimOperatorPanel panel;
  SimDrivetrain drivetrain;
  RobotModel robot(robotParams_);
  SimLineSensorBus sensorBus(entry.track, robot, clock, sensorModel, spec.lineColor, spec.seed);

  atlas::LineSensorArray sensors(sensorBus, config.sensors);
  atlas::LineFollower follower(sensors, drivetrain, config);
  atlas::AtlasApp app(clock, panel, panel, sensors, drivetrain, follower, config);

  // Escenario
  double startArcLength = 0.0;
  OperatorPlan plan;
  plan.lineColor = spec.lineColor;
  plan.speedPresses = (spec.cruisePwm - config.speed.basePwm) / config.speed.stepPwm;
  plan.startPose = startPoseFor(entry, robotParams_, &startArcLength);
  plan.raceTimeoutS = spec.timeLimitS;

  RaceMonitor monitor(entry.track, startArcLength, entry.laps);
  RaceOperator op(panel, robot, monitor, plan);
  World world(robot, drivetrain, op, monitor, follower);
  clock.setListener(&world);

  app.begin();
  while (clock.seconds() < kHardLimitS) {
    app.tick();
    clock.advanceUs(kLoopOverheadUs);
    if (op.hasReleasedGo() && app.state() == atlas::AppState::Finished) break;
  }

  ScenarioResult result;
  result.spec = spec;
  result.trackName = entry.track.name();
  monitor.fillResult(result);
  if (!result.completed && result.failureReason.empty()) {
    result.failureReason = "No terminó en " + std::to_string(static_cast<int>(spec.timeLimitS)) + " s";
  }
  result.calibratedSensors = countCalibratedSensors(sensors);
  result.stoppedCleanly = app.state() == atlas::AppState::Finished && std::fabs(robot.speed()) < kStoppedSpeed;
  result.wallSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
  return result;
}

}  // namespace sim
