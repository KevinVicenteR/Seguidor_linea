#include "TrackLibrary.h"

#include <cmath>
#include <utility>

#include "TrackBuilder.h"

namespace sim {

namespace {

int lapsFor(const Track& track) { return track.length() < 15.0 ? 2 : 1; }

TrackEntry makeEntry(Track track, Vec2 start) {
  const int laps = lapsFor(track);
  return TrackEntry{std::move(track), start, laps};
}

TrackEntry speedOval() {
  FilletedLoopBuilder b;
  b.corner(0, 0, 0.7).corner(4.0, 0, 0.7).corner(4.0, 1.6, 0.7).corner(0, 1.6, 0.7);
  return makeEntry(Track("Óvalo de velocidad", "Referencia: rectas de 2.6 m y curvas de 0.7 m de radio.", b.build()),
                   {2.0, 0.0});
}

TrackEntry longChicanes() {
  FilletedLoopBuilder b;
  b.corner(0, 0, 0.5);
  b.corner(1.2, 0, 0.18).corner(1.7, 0.35, 0.18).corner(2.2, 0, 0.18).corner(2.7, 0.35, 0.18).corner(3.2, 0, 0.18);
  b.corner(4.4, 0, 0.12).corner(4.8, 0.3, 0.12).corner(5.2, 0, 0.12).corner(5.6, 0.3, 0.12).corner(6.0, 0, 0.12);
  b.corner(7.2, 0, 0.5).corner(7.2, 2.4, 0.5);
  b.corner(5.8, 2.4, 0.3).corner(4.9, 1.7, 0.3).corner(4.0, 2.4, 0.3).corner(3.1, 1.7, 0.3).corner(2.2, 2.4, 0.3);
  b.corner(0, 2.4, 0.5);
  return makeEntry(Track("Chicanas largas", "Chicanas suaves y cerradas con eses amplias; obliga a cambiar de lado sin parar.",
                         b.build()),
                   {0.6, 0.0});
}

TrackEntry figureHexagon() {
  using namespace features;
  constexpr double kRadius = 2.2;
  FilletedLoopBuilder b;
  const EdgeFeaturePtr sides[] = {sineWave(0.12, 2), zigzag(0.18, 4, 0.1), squareSteps(0.25, 2),
                                  loops(2, 0.22), semicircles(3), nullptr};
  for (int k = 0; k < 6; k++) {
    const double angle = kPi / 6.0 + k * kPi / 3.0 - kPi / 2.0;
    b.corner(kRadius * std::cos(angle), kRadius * std::sin(angle), 0.35);
    if (sides[k]) b.with(sides[k]);
  }
  return makeEntry(Track("Hexágono de figuras",
                         "Hexágono donde cada lado es una figura: senoide, zigzag, escalones, rizos, semicírculos y recta.",
                         b.build()),
                   {0.0, -kRadius * std::cos(kPi / 6.0)});
}

TrackEntry fivePointStar() {
  constexpr double kRadius = 2.4;
  FilletedLoopBuilder b;
  for (int k = 0; k < 5; k++) {
    const double angle = kPi / 2.0 + (2 * k % 5) * 2.0 * kPi / 5.0;
    b.corner(kRadius * std::cos(angle), kRadius * std::sin(angle), 0.25);
  }
  return makeEntry(Track("Estrella de cinco puntas", "Pentagrama: puntas agudas de 36° y cinco cruces en el pentágono interior.",
                         b.build()),
                   {-0.35, 1.315});
}

TrackEntry loopTriangle() {
  using namespace features;
  constexpr double kSide = 4.5;
  FilletedLoopBuilder b;
  b.corner(0, 0, 0.4).with(loops(3, 0.2));
  b.corner(kSide, 0, 0.4).with(zigzag(0.15, 6, 0.06));
  b.corner(kSide / 2.0, kSide * std::sqrt(3.0) / 2.0, 0.4).with(sineWave(0.12, 3));
  return makeEntry(Track("Triángulo de rizos", "Triángulo equilátero con tres rizos de 360°, zigzag fino y senoide.", b.build()),
                   {0.75, 0.0});
}

TrackEntry hairpinSerpentine() {
  constexpr double kTop = 2.4;
  constexpr double kLow = 0.9;
  constexpr double kGap = 0.4;
  constexpr double kHairpin = 0.15;

  FilletedLoopBuilder b;
  b.corner(0, 0, 0.4).corner(6.4, 0, 0.4);
  double x = 6.4;
  for (int k = 0; k < 6; k++) {
    b.corner(x, kTop, kHairpin).corner(x - kGap, kTop, kHairpin);
    b.corner(x - kGap, kLow, kHairpin).corner(x - 2 * kGap, kLow, kHairpin);
    x -= 2 * kGap;
  }
  b.corner(x, kTop, 0.4).corner(0, kTop, 0.4);
  return makeEntry(Track("Serpentina de horquillas", "12 horquillas de 180° con radio de 15 cm separadas 40 cm.", b.build()),
                   {3.0, 0.0});
}

TrackEntry rightAngleMaze() {
  FilletedLoopBuilder b;
  b.sharp(0, 0).sharp(2.5, 0).sharp(2.5, 0.6).sharp(3.1, 0.6).sharp(3.1, 0);
  b.sharp(4.5, 0).sharp(4.5, 0.5).sharp(5.0, 0.5).sharp(5.0, 0).sharp(6.0, 0);
  b.sharp(6.0, 2.6).sharp(5.2, 2.6).sharp(5.2, 1.4).sharp(4.8, 1.4).sharp(4.8, 2.6);
  b.sharp(3.6, 2.6).sharp(3.6, 1.8).sharp(3.0, 1.8).sharp(3.0, 2.6);
  b.sharp(1.8, 2.6).sharp(1.8, 1.2).sharp(1.2, 1.2).sharp(1.2, 2.6);
  b.sharp(0, 2.6).sharp(0.6, 1.3);
  return makeEntry(Track("Laberinto de esquinas vivas",
                         "Esquinas de 90° sin radio, escalones, giros en U de 40 cm y dos ángulos agudos de 115°.",
                         b.build()),
                   {1.2, 0.0});
}

TrackEntry crossingEight() {
  FilletedLoopBuilder b;
  b.corner(-2.4, 1.2, 0.45).corner(2.4, -1.2, 0.45).corner(2.4, 1.2, 0.45).corner(-2.4, -1.2, 0.45);
  return makeEntry(Track("Ocho con cruce", "Dos lóbulos en sentidos opuestos unidos por diagonales que se cruzan a 53°.",
                         b.build()),
                   {-1.2, 0.6});
}

TrackEntry wavyFlower() {
  auto curve = [](double t) {
    const double theta = 2.0 * kPi * t;
    const double radius = 1.6 + 0.25 * std::sin(7.0 * theta);
    return Vec2{radius * std::cos(theta), radius * std::sin(theta)};
  };
  return makeEntry(Track("Flor ondulada", "Curvatura variable continua con 14 inflexiones por vuelta.",
                         buildParametricLoop(curve)),
                   {1.6, 0.0});
}

TrackEntry loopClover() {
  auto curve = [](double t) {
    const double theta = 2.0 * kPi * t;
    constexpr double kA = 1.6;
    constexpr double kB = 0.55;
    return Vec2{kA * std::cos(theta) + kB * std::cos(5.0 * theta), kA * std::sin(theta) + kB * std::sin(5.0 * theta)};
  };
  return makeEntry(Track("Trébol de lazos", "Cuatro lazos de 11 cm de radio, cada uno con su propio cruce.",
                         buildParametricLoop(curve)),
                   {2.15, 0.0});
}

TrackEntry grandPrix() {
  FilletedLoopBuilder b;
  b.corner(1.5, -1.2, 0.3).with(features::loops(1, 0.22)).corner(6.0, -1.2, 0.35);
  b.corner(6.8, -0.6, 0.2).corner(6.2, 0.0, 0.2).corner(6.8, 0.6, 0.2).corner(6.0, 1.2, 0.35);
  b.sharp(4.5, 1.2).sharp(4.5, 1.6).sharp(3.0, 1.6).sharp(3.0, 1.2);
  b.corner(1.5, 1.2, 0.3);
  b.corner(-1.5, -1.2, 0.3);
  b.corner(-2.6, -1.2, 0.15).corner(-2.6, -0.2, 0.15).corner(-3.0, -0.2, 0.15).corner(-3.0, -1.2, 0.15);
  b.corner(-3.4, -1.2, 0.15).corner(-3.4, -0.2, 0.15).corner(-3.8, -0.2, 0.15).corner(-3.8, -1.2, 0.15);
  b.corner(-5.5, -1.2, 0.4).corner(-5.5, 1.2, 0.4);
  b.corner(-4.0, 1.7, 0.5).corner(-2.8, 1.2, 0.5).corner(-1.5, 1.2, 0.3);
  return makeEntry(Track("Gran Premio",
                         "Circuito mixto: rizo, chicana, escalón de 90°, cruce, horquillas y curvas rápidas.",
                         b.build()),
                   {2.0, -1.2});
}

TrackEntry competitionTrack() {
  constexpr double kMetersPerPixel = 3.66 / 995.0;
  constexpr double kBoardBottomPx = 678.0;
  constexpr double kSymmetryX = 1025.0;
  constexpr double kSymmetryY = 706.0;

  struct PixelCorner {
    double x;
    double y;
    double radius;
  };
  const PixelCorner half[] = {
      {40, 58, 30},   {40, 135, 25},  {340, 340, 40}, {300, 428, 40}, {40, 285, 35},  {40, 470, 35},
      {382, 470, 25}, {382, 357, 40}, {475, 357, 40}, {475, 525, 40}, {40, 525, 35},  {40, 648, 35},
      {628, 648, 30}, {628, 585, 20}, {558, 528, 0},  {558, 408, 0},  {715, 512, 40}, {718, 648, 35},
  };

  auto toMeters = [&](double x, double y) { return Vec2{x * kMetersPerPixel, (kBoardBottomPx - y) * kMetersPerPixel}; };

  FilletedLoopBuilder b;
  for (int pass = 0; pass < 2; pass++) {
    for (const PixelCorner& c : half) {
      const double x = pass == 0 ? c.x : kSymmetryX - c.x;
      const double y = pass == 0 ? c.y : kSymmetryY - c.y;
      const Vec2 p = toMeters(x, y);
      b.corner(p.x, p.y, c.radius * kMetersPerPixel);
    }
  }

  const std::vector<TrackGap> gaps = {
      {toMeters(325, 648), toMeters(350, 648)},
      {toMeters(kSymmetryX - 325, kSymmetryY - 648), toMeters(kSymmetryX - 350, kSymmetryY - 648)},
  };
  Track track("Pista de competencia",
              "Reglamento: trovicel 3.66 x 2.44 m, cinta negra de 2 cm, curvas, dos picos agudos y dos huecos de ~9 cm.",
              b.build(), 0.020, gaps);
  return makeEntry(std::move(track), toMeters(240, 58));
}

TrackEntry practiceTrack() {
  FilletedLoopBuilder b;
  b.corner(0.0, 0.0, 0.25);
  b.corner(2.6, 0.0, 0.30);
  b.corner(2.6, 1.7, 0.30);
  b.corner(2.15, 1.7, 0.10).corner(2.00, 1.86, 0.09).corner(1.82, 1.86, 0.09);
  b.corner(1.72, 1.48, 0.09).corner(1.52, 1.48, 0.09).corner(1.44, 1.7, 0.10);
  b.corner(1.05, 1.7, 0.06).sharp(1.10, 1.30).sharp(0.85, 1.52).corner(0.45, 1.25, 0.05).corner(0.45, 1.00, 0.10);
  b.corner(0.10, 1.00, 0.09).corner(0.10, 0.78, 0.09).corner(0.35, 0.78, 0.09);
  b.sharp(0.35, 0.50);

  std::vector<TrackGap> gaps;
  const Vec2 corner{0.35, 0.50};
  const Vec2 diagonal = (Vec2{0.0, 0.0} - corner).normalized();
  for (int k = 0; k < 3; k++) {
    const double start = 0.15 + k * 0.115;
    gaps.push_back({corner + diagonal * start, corner + diagonal * (start + 0.045)});
  }
  for (int k = 0; k < 4; k++) gaps.push_back({{1.0 + k * 0.08, 0.0}, {1.03 + k * 0.08, 0.0}});
  for (int k = 0; k < 3; k++) gaps.push_back({{2.6, 0.7 + k * 0.08}, {2.6, 0.73 + k * 0.08}});
  Track track("Pista de práctica",
              "Zonas donde se salía el robot real: casa con pico agudo, gancho en U, ondas con U profunda, punteados y esquina seguida de punteado.",
              b.build(), 0.020, gaps);
  return makeEntry(std::move(track), {2.0, 0.0});
}

}

Vec2 competitionGapCenter() { return Vec2{337.5 * 3.66 / 995.0, (678.0 - 648.0) * 3.66 / 995.0}; }

TrackEntry extremeHairpins() {
  constexpr double kTop = 2.0, kLow = 0.6, kGap = 0.25, kHairpin = 0.11;
  FilletedLoopBuilder b;
  b.corner(0, 0, 0.3).corner(5.0, 0, 0.3);
  double x = 5.0;
  for (int k = 0; k < 6; k++) {
    b.corner(x, kTop, kHairpin).corner(x - kGap, kTop, kHairpin);
    b.corner(x - kGap, kLow, kHairpin).corner(x - 2 * kGap, kLow, kHairpin);
    x -= 2 * kGap;
  }
  b.corner(x, kTop, 0.3).corner(0, kTop, 0.3);
  return makeEntry(Track("Extrema: horquillas de 11 cm", "12 horquillas de 180° con radio de 11 cm separadas 25 cm.", b.build()),
                   {1.0, 0.0});
}

TrackEntry extremeRightAngles() {
  FilletedLoopBuilder b;
  b.sharp(0, 0).sharp(4.0, 0);
  for (int k = 0; k < 6; k++) {
    b.sharp(4.0 - 0.3 * k, 0.3 * (k + 1)).sharp(4.0 - 0.3 * (k + 1), 0.3 * (k + 1));
  }
  b.sharp(0, 1.8);
  return makeEntry(Track("Extrema: escalera de 90°", "Doce esquinas de 90° sin radio separadas 30 cm.", b.build()), {1.5, 0.0});
}

TrackEntry extremePeaks() {
  FilletedLoopBuilder b;
  b.corner(0, 0, 0.25).corner(4.0, 0, 0.25);
  double x = 4.0;
  b.sharp(x, 1.1);
  for (int k = 0; k < 4; k++) {
    b.sharp(x - 0.19, 1.8).sharp(x - 0.38, 1.1);
    x -= 0.38;
  }
  b.corner(0, 1.1, 0.25);
  return makeEntry(Track("Extrema: picos de 30°", "Cuatro picos y valles de 30° seguidos (la barra ve ambas ramas).", b.build()),
                   {2.0, 0.0});
}

TrackEntry extremeCornerDashes() {
  const Vec2 corners[] = {{0, 0}, {3.5, 0}, {3.5, 2.0}, {0, 2.0}};
  FilletedLoopBuilder b;
  for (const Vec2& c : corners) b.sharp(c.x, c.y);
  std::vector<TrackGap> gaps;
  for (int i = 0; i < 4; i++) {
    const Vec2 c = corners[i];
    const Vec2 dir = (corners[(i + 1) % 4] - c).normalized();
    for (int k = 0; k < 3; k++) {
      const double start = 0.15 + k * 0.115;
      gaps.push_back({c + dir * start, c + dir * (start + 0.045)});
    }
  }
  Track track("Extrema: esquinas con punteado", "Cuatro esquinas de 90°, cada una seguida de tres huecos de 4.5 cm.",
              b.build(), Track::kStandardLineWidth, gaps);
  return makeEntry(std::move(track), {1.5, 0.0});
}

TrackEntry extremeShallowCrossing() {
  FilletedLoopBuilder b;
  b.corner(-1.5, 0.4, 0.3).corner(1.5, -0.4, 0.3).corner(2.3, -0.4, 0.35).corner(2.3, 0.4, 0.35);
  b.corner(1.5, 0.4, 0.3).corner(-1.5, -0.4, 0.3).corner(-2.3, -0.4, 0.35).corner(-2.3, 0.4, 0.35);
  return makeEntry(Track("Extrema: cruce a 30°", "Ocho alargado: las diagonales se cruzan a solo 30°.", b.build()), {-0.75, 0.2});
}

TrackEntry extremeFastEsses() {
  FilletedLoopBuilder b;
  b.corner(0, 0, 0.5).sharp(1.0, 0);
  double x = 1.0;
  for (int k = 0; k < 6; k++) {
    x += 0.35;
    b.corner(x, k % 2 == 0 ? 0.18 : -0.18, 0.12);
  }
  b.sharp(x + 0.35, 0).corner(4.5, 0, 0.5).corner(4.5, 1.6, 0.5).corner(0, 1.6, 0.5);
  return makeEntry(Track("Extrema: eses rápidas", "Seis eses de radio 12 cm después de una recta larga.", b.build()), {0.5, 0.0});
}

std::vector<TrackEntry> buildExtremeTrackLibrary() {
  std::vector<TrackEntry> library;
  library.push_back(extremeHairpins());
  library.push_back(extremeRightAngles());
  library.push_back(extremePeaks());
  library.push_back(extremeCornerDashes());
  library.push_back(extremeShallowCrossing());
  library.push_back(extremeFastEsses());
  return library;
}

std::vector<TrackEntry> buildTrackLibrary() {
  std::vector<TrackEntry> library;
  library.push_back(competitionTrack());
  library.push_back(practiceTrack());
  library.push_back(speedOval());
  library.push_back(longChicanes());
  library.push_back(figureHexagon());
  library.push_back(fivePointStar());
  library.push_back(loopTriangle());
  library.push_back(hairpinSerpentine());
  library.push_back(rightAngleMaze());
  library.push_back(crossingEight());
  library.push_back(wavyFlower());
  library.push_back(loopClover());
  library.push_back(grandPrix());
  return library;
}

}
