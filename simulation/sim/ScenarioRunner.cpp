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
constexpr double kStopCheckDelayS = 1.5;

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

// Pose del robot con la barra de sensores centrada sobre `point` de la pista.
Pose robotPoseAt(const Track& track, const Vec2& point, const RobotParams& params, double* arcLength) {
  *arcLength = track.nearestArcLength(point);
  Pose onLine = track.poseAt(*arcLength);
  Pose robot = onLine;
  robot.position = onLine.position - onLine.forward() * params.sensorBarAhead;
  return robot;
}

}  // namespace

ScenarioResult ScenarioRunner::run(const TrackEntry& entry, const ScenarioSpec& spec) const {
  const auto wallStart = std::chrono::steady_clock::now();

  atlas::RobotConfig config = firmware_;
  config.impeller.pwm = spec.impellerPwm;
  config.sensors.lineColor = spec.lineColor;

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
  plan.speedPresses = (spec.cruisePwm - config.speed.basePwm) / config.speed.stepPwm;
  double calibrationArcLength = 0.0;
  plan.calibrationPose = robotPoseAt(entry.track, entry.startPoint, robotParams_, &calibrationArcLength);
  plan.startPose = robotPoseAt(entry.track, spec.customStart ? spec.startPoint : entry.startPoint, robotParams_,
                               &startArcLength);
  plan.raceTimeoutS = spec.timeLimitS;

  RaceMonitor monitor(entry.track, startArcLength, entry.laps);
  RaceOperator op(panel, robot, monitor, plan);
  World world(robot, drivetrain, op, monitor, follower);
  clock.setListener(&world);

  app.begin();
  while (clock.seconds() < kHardLimitS) {
    app.tick();
    clock.advanceUs(kLoopOverheadUs);
    if (op.hasReleasedGo() && app.racesCompleted() > 0) break;
  }
  // Deja que el robot termine de detenerse antes de comprobarlo.
  const double settleUntil = clock.seconds() + kStopCheckDelayS;
  while (clock.seconds() < settleUntil) {
    app.tick();
    clock.advanceUs(kLoopOverheadUs);
  }

  ScenarioResult result;
  result.spec = spec;
  result.trackName = entry.track.name();
  monitor.fillResult(result);
  if (!result.completed && result.failureReason.empty()) {
    result.failureReason = "No terminó en " + std::to_string(static_cast<int>(spec.timeLimitS)) + " s";
  }
  result.calibratedSensors = sensors.calibratedSensorCount();
  result.stoppedCleanly = app.racesCompleted() > 0 && app.state() == atlas::AppState::Ready &&
                          std::fabs(robot.speed()) < kStoppedSpeed;
  if (result.completed && !result.stoppedCleanly) result.failureReason = "No se detuvo al quitar GO";
  result.wallSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
  return result;
}

}  // namespace sim
