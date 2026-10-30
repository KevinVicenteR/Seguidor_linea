#pragma once

#include <stdint.h>

namespace atlas {

struct NeuralHierarchy;
struct TinyMlpWeights;

constexpr uint8_t kSensorCount = 16;
constexpr uint16_t kSensorPositionScale = 1000;
constexpr uint16_t kLinePositionMax = (kSensorCount - 1) * kSensorPositionScale;
constexpr uint16_t kLinePositionCenter = kLinePositionMax / 2;
constexpr int16_t kPwmResolution = 255;

enum class LineColor : uint8_t { White, Black };

struct SensorConfig {
  LineColor lineColor = LineColor::Black;
  uint8_t thresholdPercent = 50;
  uint8_t averageSamples = 8;
  uint8_t collectorSamples = 10;
  uint8_t minCalibrationSpan = 40;
  uint8_t requiredCalibratedSensors = kSensorCount - 2;
  bool trackNearestCluster = true;
  uint8_t maxLineWidthSensors = 3;
  uint16_t maxPositionJump = 1000;
  uint8_t reacquireFreeReads = 40;
  uint16_t maxHeldReads = 150;
  int16_t heldReadsReferencePwm = 250;
  bool neuralPosition = false;
  uint16_t neuralMaxCorrection = 300;
  bool exposeNormalizedValues = false;
};

struct ControlConfig {
  float kp = 0.0075f;
  float kd = 0.86f;
  int16_t maxCorrectionPwm = 1000;
  uint8_t derivativeFilterShift = 1;
  uint8_t curveSlowdownPercent = 27;
  int16_t maxForwardPwm = 255;
  int16_t maxBackwardPwm = -134;
  int16_t minMotorPwm = 40;
  uint16_t loopPeriodUs = 750;
  bool neuralGains = false;
  const NeuralHierarchy* hierarchy = nullptr;
  bool neuralDerivative = false;
  uint8_t derivativeBlend = 100;
  bool latencyCompensation = true;
  uint8_t predictorShift = 7;
  int8_t predictorGain = -5;
  float ki = 0.0f;
  int32_t integralLimit = 0;
  const TinyMlpWeights* residualNet = nullptr;
  int16_t residualLimit = 0;
};

struct SpeedConfig {
  int16_t basePwm = 100;
  int16_t minPwm = 60;
  int16_t maxPwm = 250;
  int16_t stepPwm = 10;
  uint16_t runRampMs = 150;
  uint16_t stopBrakeMs = 500;
  int16_t startKickPwm = 100;
  uint16_t startKickMs = 200;
  uint16_t maxAccelPwmPerS = 0;
  uint16_t launchAccelPwmPerS = 0;
};

struct ImpellerConfig {
  int16_t pwm = 200;
  uint16_t startRampMs = 1000;
  uint16_t runRampMs = 250;
  bool spinWithReady = false;
};

struct LostLineConfig {
  int16_t outerMotorPwm = 175;
  int16_t innerMotorPwm = -134;
  uint16_t innerBrakeMs = 54;
  uint16_t curveBrakeMs = 0;
  uint16_t curveWindowMs = 0;
  uint16_t gapBridgeMs = 200;
  int16_t gapCenterTolerance = 1500;
  uint16_t gapStraightMs = 25;
  uint16_t gapChainMs = 300;
};

struct CornerConfig {
  bool enabled = true;
  int16_t triggerError = 2000;
  uint8_t speedPercent = 74;
  uint16_t holdMs = 7;
  uint16_t recoveryMs = 291;
};

struct AdaptiveConfig {
  bool autoDamping = true;
  int16_t oscillationAmplitude = 700;
  uint16_t oscillationWindowMs = 250;
  uint8_t oscillationCrossings = 4;
  uint8_t minGainPercent = 52;
  uint8_t gainStepDownPercent = 10;
  uint16_t gainRecoverMs = 80;

  bool speedGovernor = true;
  uint8_t minSpeedPercent = 60;
  uint8_t speedDropOnLoss = 6;
  uint8_t speedDropOnOscillation = 4;
  uint16_t speedRecoverMs = 60;
};

struct PlannerConfig {
  uint32_t binUnits = 80000;
  int16_t maxPwm = 240;
  int16_t offMapPwm = 110;
  int16_t fullTurnPwm = 90;
  uint8_t straightCurvature = 25;
  uint8_t brakePwmPerBin = 8;
  uint8_t lookaheadBins = 25;
  uint16_t minimumBins = 30;
  uint16_t resyncWindowBins = 25;

  uint16_t easyLostPerMille = 30;
  uint8_t slowdownPerLostPerMille = 1;
  uint8_t minConfidencePercent = 70;

  uint16_t failedRunLostMs = 1500;
  uint16_t stoppedWhileLostMs = 200;
  uint8_t completedMapPercent = 95;
  uint8_t dangerZoneBinsBefore = 10;
  uint8_t dangerZoneBinsAfter = 3;
  uint8_t dangerCurvatureBoost = 90;
  uint8_t curveSpeedStepPercent = 10;
  uint8_t minCurveSpeedPercent = 60;
};

struct RobotConfig {
  SensorConfig sensors;
  ControlConfig control;
  SpeedConfig speed;
  ImpellerConfig impeller;
  LostLineConfig lostLine;
  CornerConfig corner;
  AdaptiveConfig adaptive;
  PlannerConfig planner;
  bool motorsEnabled = true;
};

}
