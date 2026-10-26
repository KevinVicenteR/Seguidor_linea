#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "Profiles.h"
#include "ScenarioRunner.h"
#include "TrackLibrary.h"

using namespace sim;

namespace {

struct Params {
  double kp, kd;
  int filterShift, curveSlowdown, outerPwm;
  int maxPwm, fullTurnPwm, straightCurvature, brakePerBin, lookahead;
};

atlas::RobotConfig toFastConfig(const Params& p) {
  atlas::RobotConfig c = atlas::fastProfile();
  c.control.kp = static_cast<float>(p.kp);
  c.control.kd = static_cast<float>(p.kd);
  c.control.derivativeFilterShift = static_cast<uint8_t>(p.filterShift);
  c.control.curveSlowdownPercent = static_cast<uint8_t>(p.curveSlowdown);
  c.lostLine.outerMotorPwm = static_cast<int16_t>(p.outerPwm);
  c.planner.maxPwm = static_cast<int16_t>(p.maxPwm);
  c.planner.fullTurnPwm = static_cast<int16_t>(p.fullTurnPwm);
  c.planner.straightCurvature = static_cast<uint8_t>(p.straightCurvature);
  c.planner.brakePwmPerBin = static_cast<uint8_t>(p.brakePerBin);
  c.planner.lookaheadBins = static_cast<uint8_t>(p.lookahead);
  return c;
}

Params defaults() {
  const atlas::RobotConfig c = atlas::fastProfile();
  return {c.control.kp,           c.control.kd,           c.control.derivativeFilterShift,
          c.control.curveSlowdownPercent, c.lostLine.outerMotorPwm, c.planner.maxPwm,
          c.planner.fullTurnPwm,  c.planner.straightCurvature, c.planner.brakePwmPerBin,
          c.planner.lookaheadBins};
}

std::vector<RobotParams> models() {
  RobotParams nominal;
  RobotParams moderate;
  moderate.motorTimeConstant = 0.055;
  moderate.yawTimeConstant = 0.020;
  moderate.commandLatencyS = 0.003;
  RobotParams real;
  real.motorTimeConstant = 0.070;
  real.yawTimeConstant = 0.040;
  real.commandLatencyS = 0.004;
  return {nominal, moderate, real};
}

struct Case {
  std::size_t model;
  std::size_t track;
  uint32_t seed;
};

struct Score {
  int failures = 0;
  double guidedTime = 0.0, p95 = 0.0, cost = 1e18;
};

Score evaluate(const Params& p, const std::vector<TrackEntry>& lib, const std::vector<RobotParams>& robotModels,
               const std::vector<Case>& cases) {
  const atlas::RobotConfig fast = toFastConfig(p);
  std::vector<std::vector<ScenarioResult>> results(cases.size());
  std::atomic<std::size_t> next{0};
  auto worker = [&]() {
    for (std::size_t i = next++; i < cases.size(); i = next++) {
      ScenarioSpec spec;
      spec.trackIndex = cases[i].track;
      spec.cruisePwm = 100;
      spec.seed = cases[i].seed;
      spec.learnTrack = true;
      spec.attempts = 3;
      results[i] = ScenarioRunner(robotModels[cases[i].model], SensorModel(), atlas::safeProfile(), fast)
                       .runAttempts(lib[cases[i].track], spec);
    }
  };
  std::vector<std::thread> pool;
  for (unsigned t = 0; t < std::max(1u, std::thread::hardware_concurrency()); t++) pool.emplace_back(worker);
  for (std::thread& th : pool) th.join();

  Score s;
  int guided = 0;
  for (const std::vector<ScenarioResult>& attempts : results) {
    for (std::size_t a = 0; a < attempts.size(); a++) {
      if (!attempts[a].passed()) {
        s.failures++;
        continue;
      }
      if (a == 0) continue;
      s.guidedTime += attempts[a].raceTime;
      s.p95 += attempts[a].p95OffsetMm;
      guided++;
    }
  }
  if (guided) {
    s.guidedTime /= guided;
    s.p95 /= guided;
  }
  s.cost = 1000.0 * s.failures + s.guidedTime + 0.05 * s.p95;
  return s;
}

void print(const char* tag, const Params& p, const Score& s) {
  std::printf("%s costo=%.2f fallas=%d t_guiado=%.2fs p95=%.1fmm | kp=%.3f kd=%.2f f=%d curva=%d ext=%d "
              "max=%d giro=%d recta=%d freno=%d mira=%d\n",
              tag, s.cost, s.failures, s.guidedTime, s.p95, p.kp, p.kd, p.filterShift, p.curveSlowdown, p.outerPwm,
              p.maxPwm, p.fullTurnPwm, p.straightCurvature, p.brakePerBin, p.lookahead);
  std::fflush(stdout);
}

template <typename T>
T clampTo(T v, T lo, T hi) {
  return std::max(lo, std::min(hi, v));
}

}

int main(int argc, char** argv) {
  const int randomSamples = argc > 1 ? std::atoi(argv[1]) : 100;
  const int refinements = argc > 2 ? std::atoi(argv[2]) : 150;
  std::mt19937 rng(argc > 3 ? std::atoi(argv[3]) : 42);

  const std::vector<TrackEntry> lib = buildTrackLibrary();
  const std::vector<RobotParams> robotModels = models();
  const std::vector<std::string> trackNames = {"Pista de competencia", "Gran Premio", "Laberinto de esquinas vivas",
                                               "Chicanas largas"};
  std::vector<Case> cases;
  for (std::size_t m = 0; m < robotModels.size(); m++)
    for (std::size_t t = 0; t < lib.size(); t++)
      if (std::find(trackNames.begin(), trackNames.end(), lib[t].track.name()) != trackNames.end())
        cases.push_back({m, t, static_cast<uint32_t>(3001 + 17 * t + m)});

  Params best = defaults();
  Score bestScore = evaluate(best, lib, robotModels, cases);
  print("actual    ", best, bestScore);

  auto uni = [&](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); };
  auto uniInt = [&](int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); };

  for (int i = 0; i < randomSamples; i++) {
    Params p{uni(0.015, 0.08), uni(0.3, 2.2), uniInt(0, 3), uniInt(0, 50), uniInt(140, 255), uniInt(180, 255),
             uniInt(60, 160), uniInt(10, 70), uniInt(3, 20), uniInt(10, 40)};
    const Score s = evaluate(p, lib, robotModels, cases);
    if (s.cost < bestScore.cost) {
      best = p;
      bestScore = s;
      print("mejor     ", best, bestScore);
    }
  }

  for (int i = 0; i < refinements; i++) {
    Params p = best;
    const double scale = 1.0 - 0.8 * i / std::max(1, refinements);
    p.kp = clampTo(p.kp + uni(-0.008, 0.008) * scale, 0.01, 0.1);
    p.kd = clampTo(p.kd + uni(-0.25, 0.25) * scale, 0.2, 3.0);
    if (uniInt(0, 6) == 0) p.filterShift = clampTo(p.filterShift + uniInt(-1, 1), 0, 4);
    p.curveSlowdown = clampTo(p.curveSlowdown + static_cast<int>(uni(-8, 8) * scale), 0, 70);
    p.outerPwm = clampTo(p.outerPwm + static_cast<int>(uni(-20, 20) * scale), 120, 255);
    p.maxPwm = clampTo(p.maxPwm + static_cast<int>(uni(-15, 15) * scale), 150, 255);
    p.fullTurnPwm = clampTo(p.fullTurnPwm + static_cast<int>(uni(-12, 12) * scale), 50, 200);
    p.straightCurvature = clampTo(p.straightCurvature + static_cast<int>(uni(-8, 8) * scale), 5, 100);
    p.brakePerBin = clampTo(p.brakePerBin + static_cast<int>(uni(-3, 3) * scale), 2, 30);
    p.lookahead = clampTo(p.lookahead + static_cast<int>(uni(-5, 5) * scale), 5, 60);
    const Score s = evaluate(p, lib, robotModels, cases);
    if (s.cost < bestScore.cost) {
      best = p;
      bestScore = s;
      print("refinado  ", best, bestScore);
    }
  }
  print("FINAL     ", best, bestScore);
  return 0;
}
