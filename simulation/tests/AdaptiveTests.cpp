#include "Config.h"
#include "OscillationGuard.h"
#include "SpeedGovernor.h"
#include "TestFramework.h"

using namespace atlas;

TEST("OscillationGuard: zigzag rápido reduce la ganancia") {
  AdaptiveConfig config;
  OscillationGuard guard(config);
  guard.reset(0);
  bool detected = false;
  for (uint32_t t = 0; t < 200; t += 20) {  // cambia de lado cada 20 ms
    detected = guard.update((t / 20) % 2 ? 2000 : -2000, t) || detected;
  }
  EXPECT_TRUE(detected);
  EXPECT_TRUE(guard.gainPercent() < 100);
}

TEST("OscillationGuard: curva sostenida no es zigzag") {
  AdaptiveConfig config;
  OscillationGuard guard(config);
  guard.reset(0);
  for (uint32_t t = 0; t < 500; t++) EXPECT_FALSE(guard.update(3000, t));
  EXPECT_EQ(guard.gainPercent(), 100);
}

TEST("OscillationGuard: la ganancia no baja del mínimo y se recupera") {
  AdaptiveConfig config;
  config.minGainPercent = 50;
  OscillationGuard guard(config);
  guard.reset(0);
  uint32_t t = 0;
  for (int i = 0; i < 200; i++, t += 10) guard.update(i % 2 ? 2000 : -2000, t);
  EXPECT_EQ(guard.gainPercent(), 50);
  for (int i = 0; i < 6000; i++, t++) guard.update(0, t);  // 6 s estable
  EXPECT_EQ(guard.gainPercent(), 100);
}

TEST("OscillationGuard desactivado: ganancia fija") {
  AdaptiveConfig config;
  config.autoDamping = false;
  OscillationGuard guard(config);
  guard.reset(0);
  for (uint32_t t = 0; t < 200; t += 10) guard.update(t % 20 ? 2000 : -2000, t);
  EXPECT_EQ(guard.gainPercent(), 100);
}

TEST("SpeedGovernor: penaliza, respeta el mínimo y recupera") {
  AdaptiveConfig config;
  config.minSpeedPercent = 60;
  SpeedGovernor governor(config);
  governor.reset(0);
  governor.penalize(10, 0);
  EXPECT_EQ(governor.speedPercent(), 90);
  EXPECT_EQ(governor.apply(200), 180);
  for (int i = 0; i < 10; i++) governor.penalize(10, 0);
  EXPECT_EQ(governor.speedPercent(), 60);
  for (uint32_t t = 0; t < 5000; t++) governor.recover(t);
  EXPECT_EQ(governor.speedPercent(), 100);
}
