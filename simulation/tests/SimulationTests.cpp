#include <cmath>

#include "ScenarioRunner.h"
#include "TestFramework.h"
#include "Track.h"
#include "TrackBuilder.h"
#include "TrackLibrary.h"

using namespace sim;

TEST("Cuadrado redondeado: longitud = perímetro - esquinas + arcos") {
  FilletedLoopBuilder builder;
  builder.corner(0, 0, 0.2).corner(1, 0, 0.2).corner(1, 1, 0.2).corner(0, 1, 0.2);
  const Track track("cuadrado", "", builder.build());
  const double expected = 4.0 - 4 * 0.4 + 2 * kPi * 0.2;
  EXPECT_NEAR(track.length(), expected, 0.01);
}

TEST("Arcos que no caben en el lado lanzan excepción") {
  FilletedLoopBuilder builder;
  builder.corner(0, 0, 0.8).corner(1, 0, 0.8).corner(1, 1, 0.8).corner(0, 1, 0.8);
  EXPECT_THROWS(builder.build());
}

TEST("distanceToLine y projectNear sobre una recta") {
  FilletedLoopBuilder builder;
  builder.sharp(0, 0).sharp(4, 0).sharp(4, 2).sharp(0, 2);
  const Track track("rect", "", builder.build());
  EXPECT_NEAR(track.distanceToLine({1.0, 0.01}), 0.01, 1e-6);
  const TrackProjection projection = track.projectNear({1.0, 0.02}, 1.0, 0.2, 0.5);
  EXPECT_NEAR(projection.arcLength, 1.0, 0.006);
  EXPECT_NEAR(projection.signedOffset, 0.02, 1e-6);  // a la izquierda del sentido de avance
}

TEST("Las figuras de lado mantienen el cierre exacto de la pista") {
  FilletedLoopBuilder builder;
  builder.corner(0, 0, 0.3).with(features::loops(2, 0.2));
  builder.corner(4, 0, 0.3).with(features::zigzag(0.15, 5, 0.05));
  builder.corner(4, 3, 0.3).with(features::squareSteps(0.2, 2));
  builder.corner(0, 3, 0.3).with(features::sineWave(0.1, 2));
  const std::vector<Vec2> points = builder.build();
  for (std::size_t i = 0; i < points.size(); i++) {
    const Vec2 next = points[(i + 1) % points.size()];
    EXPECT_TRUE((next - points[i]).length() < 0.012);  // sin saltos en la polilínea
  }
}

TEST("Todas las pistas del catálogo se construyen y son largas") {
  const std::vector<TrackEntry> library = buildTrackLibrary();
  EXPECT_TRUE(library.size() >= 10);
  for (const TrackEntry& entry : library) {
    EXPECT_TRUE(entry.track.length() > 8.0);
    EXPECT_TRUE(entry.track.distanceToLine(entry.startPoint) < 0.005);
  }
}

TEST("Simulación completa: el Gran Premio a PWM 220 termina en menos de 40 s") {
  const std::vector<TrackEntry> library = buildTrackLibrary();
  std::size_t index = 0;
  for (std::size_t i = 0; i < library.size(); i++) {
    if (library[i].track.name() == "Gran Premio") index = i;
  }
  ScenarioSpec spec;
  spec.trackIndex = index;
  spec.cruisePwm = 220;
  const ScenarioResult result = ScenarioRunner().run(library[index], spec);
  EXPECT_TRUE(result.passed());
  EXPECT_TRUE(result.raceTime < 40.0);
  EXPECT_EQ(result.calibratedSensors, 16);
}

TEST("Pista de competencia: completa desde el INICIO y desde el hueco en menos de 40 s") {
  const std::vector<TrackEntry> library = buildTrackLibrary();
  const TrackEntry& competition = library.front();
  EXPECT_TRUE(competition.track.name() == "Pista de competencia");
  EXPECT_TRUE(competition.track.hasGaps());
  EXPECT_TRUE(competition.track.distanceToLine(competitionGapCenter()) > 0.02);  // sin cinta en el hueco

  for (bool fromGap : {false, true}) {
    ScenarioSpec spec;
    spec.cruisePwm = 190;
    spec.customStart = fromGap;
    spec.startPoint = competitionGapCenter();
    const ScenarioResult result = ScenarioRunner().run(competition, spec);
    EXPECT_TRUE(result.passed());
    EXPECT_TRUE(result.raceTime < 40.0);
  }
}
