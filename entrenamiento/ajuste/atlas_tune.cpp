#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "ScenarioRunner.h"
#include "TrackLibrary.h"

using namespace sim;

namespace {

constexpr double kGeometricLimitM = 0.18;

struct Params {
  double kp, kd;
  int filterShift;
  int curveSlowdown;
  bool cornerEnabled;
  int cornerTrigger, cornerSpeed, cornerHold, cornerRecovery;
  int maxBackward, innerBrakeMs;
  int outerPwm, minGainPercent;
  bool autoDamping, speedGovernor;
};

enum class Mode { Nominal, Robust, Fast, Competition };

std::vector<RobotParams> robotModels(Mode mode) {
  std::vector<RobotParams> models(1);
  if (mode == Mode::Nominal) return models;
  if (mode == Mode::Competition) {
    RobotParams moderate;
    moderate.motorTimeConstant = 0.055;
    moderate.yawTimeConstant = 0.020;
    moderate.commandLatencyS = 0.003;
    RobotParams real;
    real.motorTimeConstant = 0.070;
    real.yawTimeConstant = 0.040;
    real.commandLatencyS = 0.004;
    models.push_back(moderate);
    models.push_back(real);
    return models;
  }
  if (mode == Mode::Fast) {
    RobotParams moderate;
    moderate.motorTimeConstant = 0.055;
    moderate.yawTimeConstant = 0.020;
    moderate.commandLatencyS = 0.003;
    models.push_back(moderate);
    return models;
  }
  RobotParams slow;
  slow.motorTimeConstant = 0.090;
  RobotParams latency;
  latency.commandLatencyS = 0.008;
  RobotParams inertia;
  inertia.yawTimeConstant = 0.050;
  RobotParams combined;
  combined.motorTimeConstant = 0.070;
  combined.yawTimeConstant = 0.040;
  combined.commandLatencyS = 0.004;
  models.push_back(slow);
  models.push_back(latency);
  models.push_back(inertia);
  models.push_back(combined);
  return models;
}

atlas::RobotConfig toConfig(const Params& p) {
  atlas::RobotConfig c;
  c.control.kp = static_cast<float>(p.kp);
  c.control.kd = static_cast<float>(p.kd);
  c.control.derivativeFilterShift = static_cast<uint8_t>(p.filterShift);
  c.control.curveSlowdownPercent = static_cast<uint8_t>(p.curveSlowdown);
  c.control.maxBackwardPwm = static_cast<int16_t>(p.maxBackward);
  c.lostLine.innerMotorPwm = static_cast<int16_t>(p.maxBackward);
  c.lostLine.innerBrakeMs = static_cast<uint16_t>(p.innerBrakeMs);
  c.corner.enabled = p.cornerEnabled;
  c.corner.triggerError = static_cast<int16_t>(p.cornerTrigger);
  c.corner.speedPercent = static_cast<uint8_t>(p.cornerSpeed);
  c.corner.holdMs = static_cast<uint16_t>(p.cornerHold);
  c.corner.recoveryMs = static_cast<uint16_t>(p.cornerRecovery);
  c.lostLine.outerMotorPwm = static_cast<int16_t>(p.outerPwm);
  c.adaptive.minGainPercent = static_cast<uint8_t>(p.minGainPercent);
  c.adaptive.autoDamping = p.autoDamping;
  c.adaptive.speedGovernor = p.speedGovernor;
  return c;
}

Params currentDefaults() {
  const atlas::RobotConfig c;
  return {c.control.kp,         c.control.kd,        c.control.derivativeFilterShift, c.control.curveSlowdownPercent,
          c.corner.enabled,     c.corner.triggerError, c.corner.speedPercent,        c.corner.holdMs,
          c.corner.recoveryMs,  c.control.maxBackwardPwm, c.lostLine.innerBrakeMs,
          c.lostLine.outerMotorPwm, c.adaptive.minGainPercent, c.adaptive.autoDamping, c.adaptive.speedGovernor};
}

struct Case {
  std::size_t model;
  std::size_t track;
  int16_t pwm;
  uint32_t seed;
};

struct Score {
  int failures = 0;
  double time = 0.0, rms = 0.0, p95 = 0.0, cost = 1e18;
};

double timeWeight = 0.3;

Score evaluate(const Params& p, const std::vector<TrackEntry>& lib, const std::vector<RobotParams>& models,
               const std::vector<Case>& cases, const std::vector<double>* baseTimes, std::vector<double>* timesOut) {
  std::vector<ScenarioRunner> runners;
  for (const RobotParams& model : models) runners.emplace_back(model, SensorModel(), toConfig(p));
  std::vector<ScenarioResult> results(cases.size());
  std::atomic<std::size_t> next{0};
  auto worker = [&]() {
    for (std::size_t i = next++; i < cases.size(); i = next++) {
      ScenarioSpec spec;
      spec.trackIndex = cases[i].track;
      spec.cruisePwm = cases[i].pwm;
      spec.seed = cases[i].seed;
      results[i] = runners[cases[i].model].run(lib[cases[i].track], spec);
    }
  };
  std::vector<std::thread> pool;
  for (unsigned t = 0; t < std::max(1u, std::thread::hardware_concurrency()); t++) pool.emplace_back(worker);
  for (std::thread& th : pool) th.join();

  Score s;
  double timeRatioSum = 0.0;
  int passed = 0;
  for (std::size_t i = 0; i < results.size(); i++) {
    const ScenarioResult& r = results[i];
    if (timesOut) timesOut->push_back(r.raceTime);
    if (!r.passed()) {
      s.failures++;
      continue;
    }
    passed++;
    s.time += r.raceTime;
    s.rms += r.rmsOffsetMm;
    s.p95 += r.p95OffsetMm;
    if (baseTimes) timeRatioSum += r.raceTime / (*baseTimes)[i];
  }
  if (passed) {
    s.rms /= passed;
    s.p95 /= passed;
  }
  const double extraTimePercent = (baseTimes && passed) ? 100.0 * (timeRatioSum / passed - 1.0) : 0.0;
  s.cost = 1000.0 * s.failures + s.rms + 0.3 * s.p95 + timeWeight * extraTimePercent;
  return s;
}

void print(const char* tag, const Params& p, const Score& s) {
  std::printf("%s costo=%.1f fallas=%d rms=%.1fmm p95=%.1fmm tiempo=%.1fs | kp=%.3f kd=%.2f f=%d curva=%d "
              "esquina=%d trig=%d vel=%d%% hold=%d rec=%d atras=%d freno=%d ext=%d gmin=%d amort=%d gob=%d\n",
              tag, s.cost, s.failures, s.rms, s.p95, s.time, p.kp, p.kd, p.filterShift, p.curveSlowdown,
              p.cornerEnabled, p.cornerTrigger, p.cornerSpeed, p.cornerHold, p.cornerRecovery, p.maxBackward,
              p.innerBrakeMs, p.outerPwm, p.minGainPercent, p.autoDamping, p.speedGovernor);
  std::fflush(stdout);
}

template <typename T>
T clampTo(T v, T lo, T hi) {
  return std::max(lo, std::min(hi, v));
}

}

int main(int argc, char** argv) {
  const int randomSamples = argc > 1 ? std::atoi(argv[1]) : 150;
  const int refinements = argc > 2 ? std::atoi(argv[2]) : 150;
  std::mt19937 rng(argc > 3 ? std::atoi(argv[3]) : 42);
  const std::string flag = argc > 4 ? argv[4] : "";
  const Mode mode = flag == "--robusto"       ? Mode::Robust
                    : flag == "--rapido"      ? Mode::Fast
                    : flag == "--competencia" ? Mode::Competition
                                              : Mode::Nominal;
  const bool robust = mode != Mode::Nominal;
  if (mode == Mode::Fast) timeWeight = 1.5;
  const std::vector<RobotParams> models = robotModels(mode);

  const std::vector<TrackEntry> lib = buildTrackLibrary();
  std::vector<Case> cases;
  const std::vector<std::string> robustTracks = {"Pista de competencia", "Gran Premio", "Laberinto de esquinas vivas",
                                                 "Chicanas largas"};
  for (std::size_t m = 0; m < models.size(); m++)
    for (std::size_t t = 0; t < lib.size(); t++) {
      const std::string& name = lib[t].track.name();
      if (lib[t].track.minimumTurnRadius() < kGeometricLimitM && name == "Trébol de lazos") continue;
      const bool loopConstraint = name == "Triángulo de rizos" && m == 0;
      if (mode == Mode::Competition && name != "Pista de competencia" && !loopConstraint) continue;
      if (robust && std::find(robustTracks.begin(), robustTracks.end(), name) == robustTracks.end()) continue;
      const std::vector<int16_t> speeds = mode == Mode::Competition ? std::vector<int16_t>{100, 130, 160}
                                          : robust                  ? std::vector<int16_t>{100, 160, 220}
                                                                    : std::vector<int16_t>{100, 130, 160, 190, 220, 250};
      const std::vector<uint32_t> seeds =
          mode == Mode::Competition ? std::vector<uint32_t>{101u, 202u, 303u, 404u} : std::vector<uint32_t>{101u, 202u};
      for (int16_t pwm : speeds)
        for (uint32_t seed : seeds) cases.push_back({m, t, pwm, seed + static_cast<uint32_t>(t)});
    }

  Params reference = currentDefaults();
  reference.cornerEnabled = false;
  std::vector<double> baseTimes;
  print("referencia", reference, evaluate(reference, lib, models, cases, nullptr, &baseTimes));

  Params best = currentDefaults();
  Score bestScore = evaluate(best, lib, models, cases, &baseTimes, nullptr);
  print("actual    ", best, bestScore);

  auto uni = [&](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); };
  auto uniInt = [&](int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); };

  for (int i = 0; i < randomSamples; i++) {
    Params p{uni(0.02, 0.08), uni(0.2, 2.2), uniInt(0, 3), uniInt(0, 70), uniInt(0, 1) == 1, uniInt(2500, 6500),
             uniInt(35, 95), uniInt(0, 120), uniInt(40, 300), uniInt(-200, -60), uniInt(10, 80),
             uniInt(120, 255), uniInt(30, 80), uniInt(0, 3) > 0, uniInt(0, 3) > 0};
    const Score s = evaluate(p, lib, models, cases, &baseTimes, nullptr);
    if (s.cost < bestScore.cost) {
      best = p;
      bestScore = s;
      print("mejor     ", best, bestScore);
    }
  }

  for (int i = 0; i < refinements; i++) {
    Params p = best;
    const double scale = 1.0 - 0.8 * i / std::max(1, refinements);
    p.kp = clampTo(p.kp + uni(-0.01, 0.01) * scale, 0.02, 0.1);
    p.kd = clampTo(p.kd + uni(-0.3, 0.3) * scale, 0.3, 3.0);
    if (uniInt(0, 5) == 0) p.filterShift = clampTo(p.filterShift + uniInt(-1, 1), 0, 4);
    p.curveSlowdown = clampTo(p.curveSlowdown + static_cast<int>(uni(-12, 12) * scale), 0, 80);
    if (uniInt(0, 10) == 0) p.cornerEnabled = !p.cornerEnabled;
    p.cornerTrigger = clampTo(p.cornerTrigger + static_cast<int>(uni(-700, 700) * scale), 2000, 7000);
    p.cornerSpeed = clampTo(p.cornerSpeed + static_cast<int>(uni(-12, 12) * scale), 30, 100);
    p.cornerHold = clampTo(p.cornerHold + static_cast<int>(uni(-25, 25) * scale), 0, 200);
    p.cornerRecovery = clampTo(p.cornerRecovery + static_cast<int>(uni(-60, 60) * scale), 20, 400);
    p.maxBackward = clampTo(p.maxBackward + static_cast<int>(uni(-25, 25) * scale), -255, -40);
    p.innerBrakeMs = clampTo(p.innerBrakeMs + static_cast<int>(uni(-12, 12) * scale), 0, 120);
    p.outerPwm = clampTo(p.outerPwm + static_cast<int>(uni(-25, 25) * scale), 100, 255);
    p.minGainPercent = clampTo(p.minGainPercent + static_cast<int>(uni(-8, 8) * scale), 20, 100);
    if (uniInt(0, 12) == 0) p.autoDamping = !p.autoDamping;
    if (uniInt(0, 12) == 0) p.speedGovernor = !p.speedGovernor;
    const Score s = evaluate(p, lib, models, cases, &baseTimes, nullptr);
    if (s.cost < bestScore.cost) {
      best = p;
      bestScore = s;
      print("refinado  ", best, bestScore);
    }
  }
  print("FINAL     ", best, bestScore);
  return 0;
}
