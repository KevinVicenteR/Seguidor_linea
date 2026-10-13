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

TEST("Secuencia completa: calibración, velocidad, READY, GO y paro") {
  AppRig rig;
  rig.app.begin();
  EXPECT_TRUE(rig.app.state() == AppState::WaitCalibration);

  rig.panel.pressButton2(true);
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
  EXPECT_TRUE(rig.panel.led(Led::Led0));  // indicador de calibrado
  EXPECT_TRUE(rig.panel.led(Led::Led1));

  rig.panel.pressButton2(true);  // +10 de velocidad
  rig.app.tick();
  rig.panel.pressButton2(false);
  EXPECT_EQ(rig.app.cruisePwm(), rig.config.speed.basePwm + rig.config.speed.stepPwm);

  rig.panel.setReady(true);
  rig.tickFor(1100);
  EXPECT_EQ(rig.drivetrain.impellerMotor.pwm, rig.config.impeller.pwm);
  EXPECT_TRUE(rig.panel.led(Led::Led2));

  rig.bus.lineAt(7, 8);  // línea negra centrada
  rig.panel.setGo(true);
  rig.tickFor(300);
  EXPECT_TRUE(rig.app.state() == AppState::Running);
  EXPECT_TRUE(rig.drivetrain.enabled);

  rig.panel.setGo(false);
  rig.tickFor(1200);
  EXPECT_TRUE(rig.app.state() == AppState::Ready);
  EXPECT_FALSE(rig.drivetrain.enabled);
  EXPECT_EQ(rig.app.racesCompleted(), 1);
}

TEST("Quitar GO conserva la calibración: GO de nuevo arranca sin recalibrar") {
  AppRig rig;
  rig.app.begin();
  rig.panel.pressButton1(true);
  rig.app.tick();
  rig.panel.pressButton1(false);
  rig.bus.fill(30);
  rig.tickFor(50);
  rig.bus.fill(220);
  rig.tickFor(50);
  rig.panel.pressButton1(true);
  rig.app.tick();
  rig.panel.pressButton1(false);
  const uint8_t threshold = rig.sensors.thresholds()[5];

  rig.bus.lineAt(7, 8);
  for (int race = 1; race <= 2; race++) {
    rig.panel.setGo(true);
    rig.tickFor(300);
    EXPECT_TRUE(rig.app.state() == AppState::Running);
    rig.panel.setGo(false);
    rig.tickFor(1200);
    EXPECT_TRUE(rig.app.state() == AppState::Ready);
    EXPECT_EQ(rig.app.racesCompleted(), race);
    EXPECT_TRUE(rig.panel.led(Led::Led0));  // sigue indicando "calibrado"
    EXPECT_TRUE(rig.panel.led(Led::Led1));
  }
  EXPECT_EQ(rig.sensors.thresholds()[5], threshold);
}

TEST("SW1 + SW2 juntos en Ready inician una nueva calibración") {
  AppRig rig;
  rig.app.begin();
  rig.panel.pressButton1(true);
  rig.app.tick();
  rig.panel.pressButton1(false);
  rig.bus.fill(30);
  rig.tickFor(50);
  rig.bus.fill(220);
  rig.tickFor(50);
  rig.panel.pressButton1(true);
  rig.app.tick();
  rig.panel.pressButton1(false);
  EXPECT_TRUE(rig.app.state() == AppState::Ready);

  rig.panel.pressButton1(true);
  rig.panel.pressButton2(true);
  rig.app.tick();
  rig.panel.pressButton1(false);
  rig.panel.pressButton2(false);
  EXPECT_TRUE(rig.app.state() == AppState::Calibrating);
  EXPECT_EQ(rig.app.cruisePwm(), rig.config.speed.basePwm);  // no cambió la velocidad
}

TEST("SW1 también inicia la calibración y siempre es línea negra") {
  AppRig rig;
  rig.app.begin();
  rig.panel.pressButton1(true);
  rig.app.tick();
  rig.panel.pressButton1(false);
  EXPECT_TRUE(rig.app.state() == AppState::Calibrating);
  EXPECT_TRUE(rig.sensors.lineColor() == LineColor::Black);
  rig.app.tick();  // patrón de línea negra: parpadean LED0 y LED1, LED2 apagado
  EXPECT_FALSE(rig.panel.led(Led::Led2));
  EXPECT_EQ(rig.panel.led(Led::Led0), rig.panel.led(Led::Led1));
}

TEST("Calibración incompleta: LED2 de error y vuelve a esperar calibración") {
  AppRig rig;
  rig.app.begin();
  rig.panel.pressButton1(true);
  rig.app.tick();
  rig.panel.pressButton1(false);
  rig.bus.fill(220);  // solo vio fondo blanco: ningún sensor vio la línea
  rig.tickFor(100);
  rig.panel.pressButton1(true);
  rig.app.tick();
  rig.panel.pressButton1(false);
  EXPECT_TRUE(rig.app.state() == AppState::WaitCalibration);
  EXPECT_FALSE(rig.panel.led(Led::Led0));
  EXPECT_EQ(rig.sensors.calibratedSensorCount(), 0);
}

TEST("Calibración válida: LED0 y LED1 siguen fijos tras ajustar la velocidad") {
  AppRig rig;
  rig.app.begin();
  rig.panel.pressButton1(true);
  rig.app.tick();
  rig.panel.pressButton1(false);
  rig.bus.fill(30);
  rig.tickFor(50);
  rig.bus.fill(220);
  rig.tickFor(50);
  rig.panel.pressButton2(true);
  rig.app.tick();
  rig.panel.pressButton2(false);
  EXPECT_TRUE(rig.app.state() == AppState::Ready);
  rig.panel.pressButton2(true);
  rig.app.tick();
  rig.panel.pressButton2(false);
  EXPECT_TRUE(rig.panel.led(Led::Led0));
  EXPECT_TRUE(rig.panel.led(Led::Led1));
}

TEST("La velocidad no baja del mínimo configurado") {
  AppRig rig;
  rig.config.speed.basePwm = 60;
  AtlasApp app{rig.clock, rig.panel, rig.panel, rig.sensors, rig.drivetrain, rig.follower, rig.config};
  app.begin();
  rig.panel.pressButton1(true);
  app.tick();
  rig.panel.pressButton1(false);
  rig.bus.fill(30);
  for (int i = 0; i < 4; i++) app.tick();
  rig.bus.fill(220);
  for (int i = 0; i < 4; i++) app.tick();
  rig.panel.pressButton1(true);
  app.tick();
  rig.panel.pressButton1(false);
  EXPECT_TRUE(app.state() == AppState::Ready);
  rig.panel.pressButton1(true);
  app.tick();
  rig.panel.pressButton1(false);
  EXPECT_EQ(app.cruisePwm(), 60);
}
