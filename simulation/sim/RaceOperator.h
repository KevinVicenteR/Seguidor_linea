/*
  RaceOperator.h - Operador humano simulado. Ejecuta exactamente la
  secuencia del manual de ATLAS sobre el firmware real:

    1. Presiona SW1 para iniciar la calibración (línea negra).
    2. Pasa la barra lentamente sobre la línea y termina la calibración.
    3. Coloca el robot en la salida y ajusta la velocidad con SW1/SW2.
    4. Activa READY (pre-giro del impeller) y luego GO.
    5. Cuando se completan las vueltas (o el robot falla) quita GO.
    6. En el siguiente intento lleva el robot a la salida y repite READY/GO.
*/
#pragma once

#include "Config.h"
#include "RaceMonitor.h"
#include "RobotModel.h"
#include "SimHardware.h"

namespace sim {

struct OperatorPlan {
  int speedPresses = 0;  // positivo = SW2 (subir), negativo = SW1 (bajar)
  Pose calibrationPose;  // donde se hace la pasada de calibración (sobre línea pintada)
  Pose startPose;        // pose de salida (puede estar sobre un hueco)
  double raceTimeoutS = 90.0;
};

class RaceOperator {
 public:
  RaceOperator(SimOperatorPanel& panel, RobotModel& robot, RaceMonitor& monitor, const OperatorPlan& plan);

  void update(double timeS);
  bool hasReleasedGo() const { return phase_ == Phase::Done; }
  // Siguiente intento: lleva el robot a la salida y vuelve a dar READY y GO.
  void nextAttempt(RaceMonitor& monitor, double timeS);
  RaceMonitor& monitor() { return *monitor_; }

 private:
  enum class Phase {
    WaitBoot,
    PressStartCalibration,
    Sweep,
    PressEndCalibration,
    WaitConfirm,
    AdjustSpeed,
    Reposition,
    Ready,
    Race,
    Done
  };

  static constexpr double kBootS = 1.0;
  static constexpr double kPressS = 0.12;
  static constexpr double kSweepS = 2.4;
  static constexpr double kSweepAmplitude = 0.09;
  static constexpr double kConfirmWaitS = 2.0;
  static constexpr double kSpeedPressPeriodS = 0.8;
  static constexpr double kReadyS = 1.3;
  static constexpr double kRepositionS = 2.0;

  void enter(Phase phase, double timeS);
  void holdAtLateralOffset(double offset);
  void pressSpeedButton(bool pressed);

  SimOperatorPanel& panel_;
  RobotModel& robot_;
  RaceMonitor* monitor_;
  OperatorPlan plan_;
  Phase phase_ = Phase::WaitBoot;
  double phaseStart_ = 0.0;
  int pressesLeft_;
};

}  // namespace sim
