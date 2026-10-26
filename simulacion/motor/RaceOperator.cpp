#include "RaceOperator.h"

#include <cmath>
#include <cstdlib>

namespace sim {

constexpr double RaceOperator::kBootS;
constexpr double RaceOperator::kPressS;
constexpr double RaceOperator::kSweepS;
constexpr double RaceOperator::kSweepAmplitude;
constexpr double RaceOperator::kConfirmWaitS;
constexpr double RaceOperator::kSpeedPressPeriodS;
constexpr double RaceOperator::kReadyS;
constexpr double RaceOperator::kRepositionS;

RaceOperator::RaceOperator(SimOperatorPanel& panel, RobotModel& robot, RaceMonitor& monitor, const OperatorPlan& plan)
    : panel_(panel), robot_(robot), monitor_(&monitor), plan_(plan), pressesLeft_(std::abs(plan.speedPresses)) {
  robot_.setHeld(true);
  holdAtLateralOffset(-kSweepAmplitude);
}

void RaceOperator::nextAttempt(RaceMonitor& monitor, double timeS) {
  monitor_ = &monitor;
  robot_.setHeld(true);
  robot_.place(plan_.startPose);
  enter(Phase::Reposition, timeS);
}

void RaceOperator::enter(Phase phase, double timeS) {
  phase_ = phase;
  phaseStart_ = timeS;
}

void RaceOperator::holdAtLateralOffset(double offset) {
  Pose pose = plan_.calibrationPose;
  pose.position = pose.position + pose.left() * offset;
  robot_.place(pose);
}

void RaceOperator::pressSpeedButton(bool pressed) {
  if (plan_.speedPresses > 0) {
    panel_.pressButton2(pressed);
  } else {
    panel_.pressButton1(pressed);
  }
}

void RaceOperator::update(double timeS) {
  const double elapsed = timeS - phaseStart_;

  switch (phase_) {
    case Phase::WaitBoot:
      if (timeS >= kBootS) {
        panel_.pressButton1(true);
        enter(Phase::PressStartCalibration, timeS);
      }
      break;

    case Phase::PressStartCalibration:
      if (elapsed >= kPressS) {
        panel_.pressButton1(false);
        enter(Phase::Sweep, timeS);
      }
      break;

    case Phase::Sweep: {
      const double u = std::fmod(elapsed / kSweepS, 1.0);
      const double triangle = u < 0.5 ? 4.0 * u - 1.0 : 3.0 - 4.0 * u;
      holdAtLateralOffset(kSweepAmplitude * triangle);
      if (elapsed >= kSweepS) {
        panel_.pressButton1(true);
        enter(Phase::PressEndCalibration, timeS);
      }
      break;
    }

    case Phase::PressEndCalibration:
      if (elapsed >= kPressS) {
        panel_.pressButton1(false);
        robot_.place(plan_.startPose);
        enter(Phase::WaitConfirm, timeS);
      }
      break;

    case Phase::WaitConfirm:
      if (elapsed >= kConfirmWaitS) enter(Phase::AdjustSpeed, timeS);
      break;

    case Phase::AdjustSpeed:
      if (pressesLeft_ == 0) {
        panel_.setReady(true);
        enter(Phase::Ready, timeS);
      } else if (elapsed >= kSpeedPressPeriodS) {
        pressSpeedButton(false);
        pressesLeft_--;
        enter(Phase::AdjustSpeed, timeS);
      } else {
        pressSpeedButton(elapsed < kPressS);
      }
      break;

    case Phase::Reposition:
      if (elapsed >= kRepositionS) {
        panel_.setReady(true);
        enter(Phase::Ready, timeS);
      }
      break;

    case Phase::Ready:
      if (elapsed >= kReadyS) {
        panel_.setGo(true);
        robot_.setHeld(false);
        monitor_->start(timeS);
        enter(Phase::Race, timeS);
      }
      break;

    case Phase::Race:
      if (monitor_->isFinished() || monitor_->hasFailed() || elapsed >= plan_.raceTimeoutS) {
        panel_.setGo(false);
        panel_.setReady(false);
        enter(Phase::Done, timeS);
      }
      break;

    case Phase::Done:
      break;
  }
}

}
