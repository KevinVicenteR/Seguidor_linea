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

namespace {
class MemoryStorage : public IPersistentStorage {
 public:
  uint8_t read(uint16_t address) override { return bytes[address]; }
  void write(uint16_t address, uint8_t value) override {
    bytes[address] = value;
    writes++;
  }
  uint8_t bytes[1024];
  int writes = 0;
  MemoryStorage() {
    for (uint8_t& b : bytes) b = 0xFF;  // EEPROM borrada
  }
};
}  // namespace

namespace {
// Calibra la aplicación (pasada sobre fondo y línea) y la deja en Ready.
void calibrateApp(AppRig& rig, AtlasApp& app) {
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
}

void runRace(AppRig& rig, AtlasApp& app, uint32_t ms) {
  rig.bus.lineAt(7, 8);
  rig.panel.setGo(true);
  for (uint64_t end = rig.clock.nowUs() + ms * 1000ULL; rig.clock.nowUs() < end; rig.clock.advanceUs(50)) app.tick();
  rig.panel.setGo(false);
  for (uint64_t end = rig.clock.nowUs() + 1200 * 1000ULL; rig.clock.nowUs() < end; rig.clock.advanceUs(50)) app.tick();
}
}  // namespace

TEST("La calibración se guarda y se carga al volver a encender") {
  MemoryStorage eeprom;
  {
    AppRig rig;
    AppOptions options;
    options.storage = &eeprom;
    AtlasApp app{rig.clock, rig.panel, rig.panel, rig.sensors, rig.drivetrain, rig.follower, rig.config, options};
    app.begin();
    EXPECT_TRUE(app.state() == AppState::WaitCalibration);  // EEPROM vacía
    calibrateApp(rig, app);
    EXPECT_TRUE(app.state() == AppState::Ready);
  }
  {
    AppRig rig;  // "se apagó y se volvió a encender": sensores sin calibrar
    AppOptions options;
    options.storage = &eeprom;
    AtlasApp app{rig.clock, rig.panel, rig.panel, rig.sensors, rig.drivetrain, rig.follower, rig.config, options};
    app.begin();
    EXPECT_TRUE(app.state() == AppState::Ready);
    EXPECT_EQ(rig.sensors.thresholds()[3], 130);
  }
}

TEST("Aprendizaje: el 1er intento graba el mapa y el siguiente corre guiado y rápido") {
  MemoryStorage eeprom;
  TrackMap map;
  RobotConfig fast;
  {
    AppRig rig;
    AppOptions options;
    options.storage = &eeprom;
    options.trackMap = &map;
    options.fastConfig = &fast;
    AtlasApp app{rig.clock, rig.panel, rig.panel, rig.sensors, rig.drivetrain, rig.follower, rig.config, options};
    app.begin();
    calibrateApp(rig, app);
    EXPECT_FALSE(app.hasTrackMap());
    runRace(rig, app, 4000);  // reconocimiento: ~4 m a PWM 100
    EXPECT_TRUE(app.hasTrackMap());
    EXPECT_TRUE(map.length() >= rig.config.planner.minimumBins);
  }
  {
    TrackMap reloaded;  // apagado entre intentos: el mapa sale de la EEPROM
    AppRig rig;
    AppOptions options;
    options.storage = &eeprom;
    options.trackMap = &reloaded;
    options.fastConfig = &fast;
    AtlasApp app{rig.clock, rig.panel, rig.panel, rig.sensors, rig.drivetrain, rig.follower, rig.config, options};
    app.begin();
    EXPECT_TRUE(app.state() == AppState::Ready);
    EXPECT_TRUE(app.hasTrackMap());
    rig.bus.lineAt(7, 8);
    rig.panel.setGo(true);
    for (int i = 0; i < 400; i++) {
      app.tick();
      rig.clock.advanceUs(50);
    }
    EXPECT_TRUE(rig.follower.mode() == RunMode::MapGuided);
  }
}

TEST("Recalibrar borra el mapa aprendido") {
  MemoryStorage eeprom;
  TrackMap map;
  AppRig rig;
  AppOptions options;
  options.storage = &eeprom;
  options.trackMap = &map;
  AtlasApp app{rig.clock, rig.panel, rig.panel, rig.sensors, rig.drivetrain, rig.follower, rig.config, options};
  app.begin();
  calibrateApp(rig, app);
  runRace(rig, app, 4000);
  EXPECT_TRUE(app.hasTrackMap());
  rig.panel.pressButton1(true);
  rig.panel.pressButton2(true);
  app.tick();
  rig.panel.pressButton1(false);
  rig.panel.pressButton2(false);
  EXPECT_FALSE(app.hasTrackMap());
  TrackMap reloaded;
  EXPECT_FALSE(TrackMapStore(eeprom).load(reloaded));
}

TEST("Guardar la misma calibración otra vez no reescribe la EEPROM") {
  MemoryStorage eeprom;
  AppRig rig;
  rig.sensors.resetCalibration();
  rig.bus.fill(30);
  rig.sensors.collectCalibrationSamples();
  rig.bus.fill(220);
  rig.sensors.collectCalibrationSamples();
  rig.sensors.finishCalibration();
  CalibrationStore store(eeprom);
  store.save(rig.sensors);
  const int firstWrites = eeprom.writes;
  store.save(rig.sensors);
  EXPECT_EQ(eeprom.writes, firstWrites);
}

TEST("EEPROM dañada o vacía: no carga nada") {
  MemoryStorage eeprom;
  AppRig rig;
  EXPECT_FALSE(CalibrationStore(eeprom).load(rig.sensors));
}

TEST("Reconocimiento fallido (robot fuera de la pista): no se guarda el mapa") {
  MemoryStorage eeprom;
  TrackMap map;
  AppRig rig;
  AppOptions options;
  options.storage = &eeprom;
  options.trackMap = &map;
  AtlasApp app{rig.clock, rig.panel, rig.panel, rig.sensors, rig.drivetrain, rig.follower, rig.config, options};
  app.begin();
  calibrateApp(rig, app);

  rig.bus.lineAt(7, 8);
  rig.panel.setGo(true);
  for (uint64_t end = rig.clock.nowUs() + 3000 * 1000ULL; rig.clock.nowUs() < end; rig.clock.advanceUs(50)) app.tick();
  rig.bus.fill(220);  // se salió: 2 s sin ver la línea
  for (uint64_t end = rig.clock.nowUs() + 2000 * 1000ULL; rig.clock.nowUs() < end; rig.clock.advanceUs(50)) app.tick();
  rig.panel.setGo(false);
  for (uint64_t end = rig.clock.nowUs() + 1500 * 1000ULL; rig.clock.nowUs() < end; rig.clock.advanceUs(50)) app.tick();

  EXPECT_TRUE(app.state() == AppState::Ready);
  EXPECT_FALSE(app.hasTrackMap());  // el siguiente intento vuelve a ser de reconocimiento
  TrackMap reloaded;
  EXPECT_FALSE(TrackMapStore(eeprom).load(reloaded));
}

TEST("Carrera guiada fallida: el siguiente intento sale más prudente") {
  MemoryStorage eeprom;
  TrackMap map;
  RobotConfig fast;
  AppRig rig;
  AppOptions options;
  options.storage = &eeprom;
  options.trackMap = &map;
  options.fastConfig = &fast;
  AtlasApp app{rig.clock, rig.panel, rig.panel, rig.sensors, rig.drivetrain, rig.follower, rig.config, options};
  app.begin();
  calibrateApp(rig, app);
  runRace(rig, app, 4000);  // reconocimiento correcto
  EXPECT_TRUE(app.hasTrackMap());
  const uint16_t before = map.lostPerMille();

  rig.bus.lineAt(7, 8);
  rig.panel.setGo(true);
  for (uint64_t end = rig.clock.nowUs() + 1000 * 1000ULL; rig.clock.nowUs() < end; rig.clock.advanceUs(50)) app.tick();
  rig.bus.fill(220);  // se sale en la carrera rápida
  for (uint64_t end = rig.clock.nowUs() + 2000 * 1000ULL; rig.clock.nowUs() < end; rig.clock.advanceUs(50)) app.tick();
  rig.panel.setGo(false);
  for (uint64_t end = rig.clock.nowUs() + 1500 * 1000ULL; rig.clock.nowUs() < end; rig.clock.advanceUs(50)) app.tick();

  EXPECT_TRUE(app.hasTrackMap());  // el mapa se conserva...
  EXPECT_EQ(map.lostPerMille(), before + rig.config.planner.confidencePenaltyPerMille);  // ...con menos confianza
  TrackMap reloaded;
  EXPECT_TRUE(TrackMapStore(eeprom).load(reloaded));
  EXPECT_EQ(reloaded.lostPerMille(), map.lostPerMille());
}
