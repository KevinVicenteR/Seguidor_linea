/*
  LineFollower.h - Control de carrera: rampas de arranque/frenado,
  seguimiento PD y recuperación cuando se pierde la línea.

  No bloquea: el dueño llama update() una vez por periodo de control.
*/
#pragma once

#include <stdint.h>

#include "Config.h"
#include "Hal.h"
#include "LineSensorArray.h"
#include "OscillationGuard.h"
#include "PdController.h"
#include "Ramp.h"
#include "SpeedGovernor.h"
#include "SpeedPlanner.h"
#include "TrackMap.h"

namespace atlas {

struct MotorCommand {
  int16_t left = 0;
  int16_t right = 0;
  int16_t impeller = 0;
};

enum class Side : uint8_t { Left, Right };

// Plain: seguimiento normal. Mapping: vuelta de reconocimiento (graba el mapa).
// MapGuided: la velocidad de cada tramo sale del mapa aprendido.
enum class RunMode : uint8_t { Plain, Mapping, MapGuided };

class LineFollower {
 public:
  LineFollower(LineSensorArray& sensors, IDrivetrain& drivetrain, const RobotConfig& config);

  // Cambia el perfil de manejo (Seguro / Rápido). Solo entre carreras.
  void useConfig(const RobotConfig& config);
  void attachTrackMap(TrackMap* map) { map_ = map; }
  void start(int16_t cruisePwm, uint32_t nowMs, RunMode mode = RunMode::Plain);
  // Cierra la vuelta de reconocimiento; devuelve true si el mapa quedó válido.
  bool finishMapping();
  // La carrera se considera fallida (robot fuera de la pista).
  bool runFailed() const;
  uint32_t longestLostMs() const { return longestLostMs_; }
  uint16_t lostPerMille() const { return runTicks_ ? static_cast<uint16_t>(runLostTicks_ * 1000 / runTicks_) : 0; }
  // Tras una vuelta guiada fallida marca la zona donde se salió como peligrosa
  // y hace más lentas todas las curvas; devuelve true si cambió el mapa.
  bool correctMapAfterFailedRun();
  // Tramo del mapa donde empezó la racha más larga sin línea (dónde se salió).
  uint16_t failureBin() const { return failureBin_; }
  RunMode mode() const { return mode_; }
  uint16_t currentBin() const;
  void requestStop(uint32_t nowMs);
  void update(uint32_t nowMs);

  bool isRunning() const { return running_; }
  bool isStopping() const { return stopping_; }
  const MotorCommand& lastCommand() const { return command_; }
  const LineReading& lastReading() const { return reading_; }
  uint8_t gainPercent() const { return guard_.gainPercent(); }
  uint8_t speedPercent() const { return governor_.speedPercent(); }

 private:
  int16_t currentVelocity(uint32_t nowMs) const;
  void scaleCrossingHoldToSpeed(int16_t velocity);
  MotorCommand followLine(int16_t velocity, uint32_t nowMs);
  int16_t slowDownForCurve(int16_t velocity, int32_t error) const;
  void detectCorner(int32_t error, uint32_t nowMs);
  uint8_t cornerSpeedPercent(uint32_t nowMs) const;
  int16_t scaleForCorner(int16_t pwm, uint32_t nowMs) const;
  MotorCommand recoverLine(uint32_t nowMs) const;
  bool isCrossingGap(uint32_t nowMs) const;
  MotorCommand bridgeGap(int16_t velocity) const;
  void rememberLastSide();
  void trackStraightRun(int32_t error, uint32_t nowMs);
  bool looksStraight(int32_t error) const;
  int16_t clampWheel(int16_t pwm) const;
  void apply(const MotorCommand& command);
  int16_t guidedVelocity(int16_t velocity) const;
  void trackDistance();
  void onGapEntered();
  void recordCurvature();
  void trackLineLoss(uint32_t nowMs);
  void noteHowRunEnded(uint32_t nowMs);

  LineSensorArray& sensors_;
  IDrivetrain& drivetrain_;
  const RobotConfig* config_;
  PdController pd_;
  OscillationGuard guard_;
  SpeedGovernor governor_;
  SpeedPlanner planner_;
  TrackMap* map_ = nullptr;
  RunMode mode_ = RunMode::Plain;
  uint32_t distanceUnits_ = 0;
  uint32_t mappingTicks_ = 0;
  uint32_t mappingLostTicks_ = 0;
  uint32_t runTicks_ = 0;
  uint32_t runLostTicks_ = 0;
  uint32_t lostSinceMs_ = 0;
  uint32_t longestLostMs_ = 0;
  uint16_t lostStartBin_ = 0;
  uint16_t failureBin_ = 0;
  bool stoppedWhileLost_ = false;
  bool stoppedIncomplete_ = false;
  bool bridging_ = false;
  bool recovering_ = false;

  LinearRamp velocityRamp_;
  LinearRamp impellerRamp_;
  LinearRamp brakeRamp_;

  bool running_ = false;
  bool stopping_ = false;
  uint32_t lastOnLineMs_ = 0;
  uint32_t centeredSinceMs_ = 0;
  uint32_t lastCornerMs_ = 0;
  bool cornerSeen_ = false;
  bool centered_ = false;
  Side lastSide_ = Side::Left;

  LineReading reading_;
  MotorCommand command_;
};

}  // namespace atlas
