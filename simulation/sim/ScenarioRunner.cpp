#include "ScenarioRunner.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <string>

#include "AtlasApp.h"
#include "LogExporter.h"
#include "RunLog.h"
#include "TrackMapStore.h"
#include "TrackMap.h"
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
// Acumula el texto exportado por el firmware.
class StringSink : public atlas::ITextSink {
 public:
  void text(const char* value) override { buffer += value; }
  void number(int32_t value) override { buffer += std::to_string(value); }
  void endLine() override { buffer += "\n"; }
  std::string buffer;
};

// EEPROM simulada del robot: conserva calibración y mapa entre intentos.
class SimEeprom : public atlas::IPersistentStorage {
 public:
  SimEeprom() : bytes_(1024, 0xFF) {}
  uint8_t read(uint16_t address) override { return bytes_[address]; }
  void write(uint16_t address, uint8_t value) override { bytes_[address] = value; }

 private:
  std::vector<uint8_t> bytes_;
};

class World : public TimeListener {
 public:
  World(RobotModel& robot, SimDrivetrain& drivetrain, RaceOperator& op, const atlas::LineFollower& follower)
      : robot_(robot), drivetrain_(drivetrain), operator_(op), follower_(follower) {}

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
      operator_.monitor().update(now, robot_, drivetrain_.inputs(), follower_.lastReading().isTrackable());
    }
  }

 private:
  RobotModel& robot_;
  SimDrivetrain& drivetrain_;
  RaceOperator& operator_;
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
  ScenarioSpec single = spec;
  single.attempts = 1;
  return runAttempts(entry, single).front();
}

std::vector<ScenarioResult> ScenarioRunner::runAttempts(const TrackEntry& entry, const ScenarioSpec& spec) const {
  const auto wallStart = std::chrono::steady_clock::now();

  atlas::RobotConfig config = firmware_;
  atlas::RobotConfig fastConfig = fastFirmware_;
  for (atlas::RobotConfig* c : {&config, &fastConfig}) {
    c->impeller.pwm = spec.impellerPwm;
    c->sensors.lineColor = spec.lineColor;
  }
  if (spec.fastImpellerPwm >= 0) fastConfig.impeller.pwm = spec.fastImpellerPwm;

  SensorModel sensorModel = sensorModel_;
  sensorModel.noiseSigma = spec.sensorNoiseSigma;

  // Hardware simulado + firmware real
  SimClock clock;
  SimOperatorPanel panel;
  SimDrivetrain drivetrain;
  SimEeprom eeprom;
  atlas::TrackMap trackMap;
  RobotModel robot(robotParams_);
  SimLineSensorBus sensorBus(entry.track, robot, clock, sensorModel, spec.lineColor, spec.seed);

  atlas::LineSensorArray sensors(sensorBus, config.sensors);
  atlas::LineFollower follower(sensors, drivetrain, config);
  atlas::AppOptions options;
  if (spec.learnTrack) {
    options.storage = &eeprom;
    options.trackMap = &trackMap;
    options.fastConfig = &fastConfig;
  }
  atlas::AtlasApp app(clock, panel, panel, sensors, drivetrain, follower, config, options);

  // Escenario
  double startArcLength = 0.0;
  OperatorPlan plan;
  plan.speedPresses = (spec.cruisePwm - config.speed.basePwm) / config.speed.stepPwm;
  double calibrationArcLength = 0.0;
  plan.calibrationPose = robotPoseAt(entry.track, entry.startPoint, robotParams_, &calibrationArcLength);
  plan.startPose = robotPoseAt(entry.track, spec.customStart ? spec.startPoint : entry.startPoint, robotParams_,
                               &startArcLength);
  plan.raceTimeoutS = spec.timeLimitS;

  std::vector<std::unique_ptr<RaceMonitor>> monitors;
  monitors.emplace_back(new RaceMonitor(entry.track, startArcLength, entry.laps));
  RaceOperator op(panel, robot, *monitors.back(), plan);
  World world(robot, drivetrain, op, follower);
  clock.setListener(&world);

  auto tickOnce = [&]() {
    app.tick();
    if (observer_) observer_(clock.seconds(), sensors, follower, robot);
    clock.advanceUs(kLoopOverheadUs);
  };
  auto tickFor = [&](double seconds) {
    const double until = clock.seconds() + seconds;
    while (clock.seconds() < until) tickOnce();
  };

  std::vector<ScenarioResult> results;
  app.begin();
  for (int attempt = 1; attempt <= std::max(1, spec.attempts); attempt++) {
    while (clock.seconds() < kHardLimitS * attempt) {
      tickOnce();
      if (op.hasReleasedGo() && app.racesCompleted() >= static_cast<uint16_t>(attempt)) break;
    }
    tickFor(kStopCheckDelayS);  // deja que el robot termine de detenerse

    ScenarioResult result;
    result.spec = spec;
    result.trackName = entry.track.name();
    monitors.back()->fillResult(result);
    if (!result.completed && result.failureReason.empty()) {
      result.failureReason = "No terminó en " + std::to_string(static_cast<int>(spec.timeLimitS)) + " s";
    }
    result.calibratedSensors = sensors.calibratedSensorCount();
    result.stoppedCleanly = app.racesCompleted() >= static_cast<uint16_t>(attempt) &&
                            app.state() == atlas::AppState::Ready && std::fabs(robot.speed()) < kStoppedSpeed;
    if (result.completed && !result.stoppedCleanly) result.failureReason = "No se detuvo al quitar GO";
    results.push_back(result);

    if (attempt < spec.attempts) {
      monitors.emplace_back(new RaceMonitor(entry.track, startArcLength, entry.laps));
      op.nextAttempt(*monitors.back(), clock.seconds());
    }
  }
  if (spec.learnTrack) {
    // Lo mismo que el modo diagnóstico enviaría por serial al terminar.
    atlas::TrackMap stored;
    atlas::TrackMapStore(eeprom).load(stored);
    atlas::RunLog log(eeprom);
    StringSink sink;
    atlas::exportRunLog(sink, log, stored);
    results.back().robotLog = sink.buffer;
  }
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
  for (ScenarioResult& r : results) r.wallSeconds = wall / results.size();
  return results;
}

}  // namespace sim
