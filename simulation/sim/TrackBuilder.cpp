#include "TrackBuilder.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace sim {

namespace {

struct Fillet {
  Vec2 entry;
  Vec2 exit;
  Vec2 center;
  double radius = 0.0;
  double turn = 0.0;  // ángulo de giro con signo
  double tangentLength = 0.0;
};

Fillet makeFillet(const Vec2& previous, const Vec2& current, const Vec2& next, double radius) {
  const Vec2 incoming = (current - previous).normalized();
  const Vec2 outgoing = (next - current).normalized();

  Fillet fillet;
  fillet.turn = std::atan2(incoming.cross(outgoing), incoming.dot(outgoing));
  fillet.radius = radius;
  fillet.entry = fillet.exit = current;
  if (radius <= 0.0 || std::fabs(fillet.turn) < 1e-9) return fillet;

  fillet.tangentLength = radius * std::tan(std::fabs(fillet.turn) / 2.0);
  fillet.entry = current - incoming * fillet.tangentLength;
  fillet.exit = current + outgoing * fillet.tangentLength;
  const double side = fillet.turn > 0.0 ? 1.0 : -1.0;
  fillet.center = fillet.entry + incoming.perpendicularLeft() * (radius * side);
  return fillet;
}

// Agrega puntos desde `from` (incluido) hasta `to` (excluido).
void appendLine(std::vector<Vec2>& out, const Vec2& from, const Vec2& to, double step) {
  const double length = (to - from).length();
  const int samples = std::max(1, static_cast<int>(std::ceil(length / step)));
  for (int k = 0; k < samples; k++) out.push_back(from + (to - from) * (static_cast<double>(k) / samples));
}

void appendArc(std::vector<Vec2>& out, const Fillet& fillet, double step) {
  if (fillet.tangentLength <= 0.0) return;
  const double startAngle = (fillet.entry - fillet.center).angle();
  const double arcLength = fillet.radius * std::fabs(fillet.turn);
  const int samples = std::max(1, static_cast<int>(std::ceil(arcLength / step)));
  for (int k = 0; k < samples; k++) {
    const double angle = startAngle + fillet.turn * (static_cast<double>(k) / samples);
    out.push_back(fillet.center + Vec2::fromAngle(angle) * fillet.radius);
  }
}

void requireFits(double used, double available, std::size_t from, std::size_t to) {
  if (used <= available + 1e-9) return;
  std::ostringstream message;
  message << "Los arcos de los vértices " << from << " y " << to << " no caben en su lado";
  throw std::invalid_argument(message.str());
}

// Re-muestrea una polilínea densa a paso uniforme; excluye el punto final.
std::vector<Vec2> resampleOpen(const std::vector<Vec2>& dense, double step) {
  std::vector<Vec2> points{dense.front()};
  double carried = 0.0;
  for (std::size_t k = 1; k < dense.size(); k++) {
    Vec2 a = dense[k - 1];
    const Vec2 b = dense[k];
    double segment = (b - a).length();
    while (carried + segment >= step) {
      a = a + (b - a) * ((step - carried) / segment);
      points.push_back(a);
      segment = (b - a).length();
      carried = 0.0;
    }
    carried += segment;
  }
  if ((points.back() - dense.back()).length() < step * 0.5) points.pop_back();
  return points;
}

//  ---------------------------------------------------------------------------
//  Figuras sobre los lados
//  ---------------------------------------------------------------------------

constexpr double kFeatureLead = 0.12;  // tramo recto antes y después de la figura

double patternSpan(double length, const char* featureName) {
  const double span = length - 2.0 * kFeatureLead;
  if (span <= 0.0) throw std::invalid_argument(std::string("Lado demasiado corto para la figura ") + featureName);
  return span;
}

// Figura definida por una función v(x) suave sobre el tramo central.
class FunctionFeature : public EdgeFeature {
 public:
  explicit FunctionFeature(std::function<double(double x, double span)> profile) : profile_(std::move(profile)) {}

  std::vector<Vec2> localPath(double length, double step) const override {
    const double span = patternSpan(length, "senoidal");
    std::vector<Vec2> dense{{0, 0}};
    const int samples = static_cast<int>(span / (step / 8.0));
    for (int k = 0; k <= samples; k++) {
      const double x = span * k / samples;
      dense.push_back({kFeatureLead + x, profile_(x, span)});
    }
    dense.push_back({length, 0});
    return resampleOpen(dense, step);
  }

 private:
  std::function<double(double, double)> profile_;
};

// Figura definida por una polilínea con esquinas redondeadas.
class PolylineFeature : public EdgeFeature {
 public:
  using Pattern = std::function<std::vector<Vec2>(double span)>;
  PolylineFeature(Pattern pattern, double filletRadius) : pattern_(std::move(pattern)), radius_(filletRadius) {}

  std::vector<Vec2> localPath(double length, double step) const override {
    const double span = patternSpan(length, "poligonal");
    std::vector<Vec2> corners{{0, 0}, {kFeatureLead, 0}};
    for (const Vec2& p : pattern_(span)) corners.push_back({kFeatureLead + p.x, p.y});
    corners.push_back({kFeatureLead + span, 0});
    corners.push_back({length, 0});
    return sampleFilletedPolyline(corners, radius_, step);
  }

 private:
  Pattern pattern_;
  double radius_;
};

// Rizos de 360°: cicloide alargada x = R·t − b·sin t, v = ±b(1 + cos t),
// t ∈ [π, 3π]. Con b > R la curva se cruza a sí misma.
class LoopFeature : public EdgeFeature {
 public:
  LoopFeature(int count, double size, bool toLeft) : count_(count), size_(size), side_(toLeft ? 1.0 : -1.0) {}

  std::vector<Vec2> localPath(double length, double step) const override {
    const double span = patternSpan(length, "rizos");
    const double rollRadius = 0.35 * size_;
    const double loopSpan = 2.0 * kPi * rollRadius;
    const double cell = span / count_;
    if (loopSpan > cell) throw std::invalid_argument("Los rizos no caben en el lado");

    std::vector<Vec2> dense{{0, 0}};
    for (int k = 0; k < count_; k++) {
      const double loopStart = kFeatureLead + cell * k + (cell - loopSpan) / 2.0;
      dense.push_back({loopStart, 0});
      constexpr int kSamples = 600;
      for (int i = 0; i <= kSamples; i++) {
        const double t = kPi + 2.0 * kPi * i / kSamples;
        const double x = rollRadius * (t - kPi) - size_ * std::sin(t);
        dense.push_back({loopStart + x, side_ * size_ * (1.0 + std::cos(t))});
      }
    }
    dense.push_back({length, 0});
    return resampleOpen(dense, step);
  }

 private:
  int count_;
  double size_;
  double side_;
};

// Semicírculos alternados; entran y salen perpendiculares al lado.
class SemicircleFeature : public EdgeFeature {
 public:
  explicit SemicircleFeature(int count) : count_(count) {}

  std::vector<Vec2> localPath(double length, double step) const override {
    const double span = patternSpan(length, "semicírculos");
    const double radius = span / (2.0 * count_);
    std::vector<Vec2> dense{{0, 0}};
    for (int k = 0; k < count_; k++) {
      const Vec2 center{kFeatureLead + radius * (2 * k + 1), 0};
      const double sweep = (k % 2 == 0) ? -kPi : kPi;  // izquierda, luego derecha
      constexpr int kSamples = 400;
      for (int i = 0; i <= kSamples; i++) {
        dense.push_back(center + Vec2::fromAngle(kPi + sweep * i / kSamples) * radius);
      }
    }
    dense.push_back({length, 0});
    return resampleOpen(dense, step);
  }

 private:
  int count_;
};

}  // namespace

namespace features {

EdgeFeaturePtr sineWave(double amplitude, int waves) {
  return std::make_shared<FunctionFeature>([=](double x, double span) {
    return amplitude * std::sin(2.0 * kPi * waves * x / span) * std::sin(kPi * x / span);
  });
}

EdgeFeaturePtr zigzag(double amplitude, int teeth, double filletRadius) {
  return std::make_shared<PolylineFeature>(
      [=](double span) {
        std::vector<Vec2> pattern;
        for (int k = 0; k < teeth; k++) pattern.push_back({span * (k + 0.5) / teeth, (k % 2 == 0) ? amplitude : -amplitude});
        return pattern;
      },
      filletRadius);
}

EdgeFeaturePtr squareSteps(double height, int steps) {
  return std::make_shared<PolylineFeature>(
      [=](double span) {
        const double cell = span / (2.0 * steps + 1.0);
        std::vector<Vec2> pattern;
        for (int k = 0; k < steps; k++) {
          const double x0 = cell * (2 * k + 1);
          pattern.push_back({x0, 0});
          pattern.push_back({x0, height});
          pattern.push_back({x0 + cell, height});
          pattern.push_back({x0 + cell, 0});
        }
        return pattern;
      },
      0.0);
}

EdgeFeaturePtr loops(int count, double size, bool toLeft) { return std::make_shared<LoopFeature>(count, size, toLeft); }

EdgeFeaturePtr semicircles(int count) { return std::make_shared<SemicircleFeature>(count); }

}  // namespace features

//  ---------------------------------------------------------------------------
//  Constructores
//  ---------------------------------------------------------------------------

FilletedLoopBuilder& FilletedLoopBuilder::corner(double x, double y, double filletRadius) {
  waypoints_.push_back({{x, y}, filletRadius, nullptr});
  return *this;
}

FilletedLoopBuilder& FilletedLoopBuilder::with(EdgeFeaturePtr feature) {
  if (waypoints_.empty()) throw std::logic_error("with() requiere un waypoint previo");
  waypoints_.back().feature = std::move(feature);
  return *this;
}

std::vector<Vec2> FilletedLoopBuilder::build() const {
  const std::size_t n = waypoints_.size();
  if (n < 3) throw std::invalid_argument("Se necesitan al menos 3 waypoints");

  std::vector<Fillet> fillets;
  for (std::size_t i = 0; i < n; i++) {
    fillets.push_back(makeFillet(waypoints_[(i + n - 1) % n].position, waypoints_[i].position,
                                 waypoints_[(i + 1) % n].position, waypoints_[i].radius));
  }

  std::vector<Vec2> points;
  for (std::size_t i = 0; i < n; i++) {
    const std::size_t j = (i + 1) % n;
    requireFits(fillets[i].tangentLength + fillets[j].tangentLength,
                (waypoints_[j].position - waypoints_[i].position).length(), i, j);

    appendArc(points, fillets[i], step_);
    const Vec2 from = fillets[i].exit;
    const Vec2 to = fillets[j].entry;
    if (!waypoints_[i].feature) {
      appendLine(points, from, to, step_);
      continue;
    }
    const Vec2 along = (to - from).normalized();
    const Vec2 left = along.perpendicularLeft();
    for (const Vec2& local : waypoints_[i].feature->localPath((to - from).length(), step_)) {
      points.push_back(from + along * local.x + left * local.y);
    }
  }
  return points;
}

std::vector<Vec2> sampleFilletedPolyline(const std::vector<Vec2>& corners, double radius, double step) {
  const std::size_t n = corners.size();
  std::vector<Fillet> fillets(n);
  fillets.front().entry = fillets.front().exit = corners.front();
  fillets.back().entry = fillets.back().exit = corners.back();
  for (std::size_t i = 1; i + 1 < n; i++) fillets[i] = makeFillet(corners[i - 1], corners[i], corners[i + 1], radius);

  std::vector<Vec2> points;
  for (std::size_t i = 0; i + 1 < n; i++) {
    requireFits(fillets[i].tangentLength + fillets[i + 1].tangentLength, (corners[i + 1] - corners[i]).length(), i,
                i + 1);
    appendArc(points, fillets[i], step);
    appendLine(points, fillets[i].exit, fillets[i + 1].entry, step);
  }
  return points;
}

std::vector<Vec2> buildParametricLoop(const std::function<Vec2(double)>& curve, double sampleStep) {
  constexpr int kDenseSamples = 40000;
  std::vector<Vec2> dense;
  dense.reserve(kDenseSamples + 1);
  for (int k = 0; k <= kDenseSamples; k++) dense.push_back(curve(static_cast<double>(k) / kDenseSamples));
  return resampleOpen(dense, sampleStep);
}

}  // namespace sim
