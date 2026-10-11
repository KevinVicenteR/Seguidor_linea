#include "AtlasApp.h"
#include "Fakes.h"
#include "LineFollower.h"
#include "LineSensorArray.h"
#include "SimHardware.h"
#include "TestFramework.h"

using namespace atlas;

namespace {

struct AppRig {
  sim::SimClock clock;
  sim::SimOperatorPanel panel;
  test::FakeSensorBus bus;
  test::FakeDrivetrain drivetrain;
  RobotConfig config;
  LineSensorArray sensors{bus, config.sensors};
  LineFollower follower{sensors, drivetrain, config};
  AtlasApp app{clock, panel, panel, sensors, drivetrain, follower, config};

  void tickFor(uint32_t ms) {
    const uint64_t end = clock.nowUs() + ms * 1000ULL;
    while (clock.nowUs() < end) {
      app.tick();
      clock.advanceUs(50);
    }
  }
};

}  // namespace

TEST("Secuencia completa: color, calibración, velocidad, READY, GO y paro") {
  AppRig rig;
  rig.app.begin();
  EXPECT_TRUE(rig.app.state() == AppState::SelectLineColor);

  rig.panel.pressButton2(true);  // línea negra
  rig.app.tick();
  rig.panel.pressButton2(false);
  EXPECT_TRUE(rig.app.state() == AppState::Calibrating);
  EXPECT_TRUE(rig.sensors.lineColor() == LineColor::Black);

  rig.bus.fill(30);
  rig.tickFor(50);
  rig.bus.fill(220);
  rig.tickFor(50);
  rig.panel.pressButton1(true);
  rig.app.tick();
  rig.panel.pressButton1(false);
  EXPECT_TRUE(rig.app.state() == AppState::Ready);

  rig.panel.pressButton2(true);  // +10 de velocidad
  rig.app.tick();
  rig.panel.pressButton2(false);
  EXPECT_EQ(rig.app.cruisePwm(), rig.config.speed.basePwm + rig.config.speed.stepPwm);

  rig.panel.setReady(true);
  rig.tickFor(1100);
  EXPECT_EQ(rig.drivetrain.impellerMotor.pwm, rig.config.impeller.pwm);
  EXPECT_TRUE(rig.panel.led(Led::Led2));

  rig.bus.lineAt(7, 8, 30, 220);  // línea negra centrada
  rig.panel.setGo(true);
  rig.tickFor(300);
  EXPECT_TRUE(rig.app.state() == AppState::Running);
  EXPECT_TRUE(rig.drivetrain.enabled);

  rig.panel.setGo(false);
  rig.tickFor(600);
  EXPECT_TRUE(rig.app.state() == AppState::Finished);
  EXPECT_FALSE(rig.drivetrain.enabled);
}

TEST("La velocidad no baja del mínimo configurado") {
  AppRig rig;
  rig.config.speed.basePwm = 60;
  AtlasApp app{rig.clock, rig.panel, rig.panel, rig.sensors, rig.drivetrain, rig.follower, rig.config};
  app.begin();
  rig.panel.pressButton1(true);
  app.tick();
  rig.panel.pressButton1(false);
  rig.panel.pressButton1(true);
  app.tick();
  rig.panel.pressButton1(false);
  EXPECT_TRUE(app.state() == AppState::Ready);
  rig.panel.pressButton1(true);
  app.tick();
  rig.panel.pressButton1(false);
  EXPECT_EQ(app.cruisePwm(), 60);
}
