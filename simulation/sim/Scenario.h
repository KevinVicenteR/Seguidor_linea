/*
  Scenario.h - Definición de un escenario de simulación y de su resultado.
*/
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "Config.h"

namespace sim {

struct ScenarioSpec {
  std::size_t trackIndex = 0;
  int16_t cruisePwm = 100;
  atlas::LineColor lineColor = atlas::LineColor::White;
  int16_t impellerPwm = 200;
  double sensorNoiseSigma = 4.0;
  uint32_t seed = 1;
  bool required = true;  // si falla, la simulación termina con error
  double timeLimitS = 40.0;  // la pista debe completarse antes de este tiempo
  std::string label;
};

struct TelemetrySample {
  double time = 0.0;  // s desde el GO
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double speed = 0.0;
  double lateralOffset = 0.0;  // de la barra de sensores respecto a la línea
  bool trackable = false;      // el PD tenía la línea (no en recuperación)
  bool sliding = false;
  int16_t leftPwm = 0;
  int16_t rightPwm = 0;
};

struct ScenarioResult {
  ScenarioSpec spec;
  std::string trackName;

  bool completed = false;
  std::string failureReason;
  std::vector<double> lapTimes;
  double raceTime = 0.0;
  double distance = 0.0;
  double averageSpeed = 0.0;
  double maxSpeed = 0.0;
  double rmsOffsetMm = 0.0;
  double maxOffsetMm = 0.0;
  double lostLinePercent = 0.0;
  double slidingPercent = 0.0;
  int calibratedSensors = 0;
  bool stoppedCleanly = false;
  double wallSeconds = 0.0;

  std::vector<TelemetrySample> telemetry;

  bool passed() const { return completed && stoppedCleanly && calibratedSensors == atlas::kSensorCount; }
};

}  // namespace sim
