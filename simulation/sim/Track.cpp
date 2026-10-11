#include "Track.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace sim {

constexpr double Track::kStandardLineWidth;
constexpr double Track::kGridCell;
constexpr double Track::kGridMargin;
constexpr double Track::kFarDistance;

Track::Track(std::string name, std::string description, std::vector<Vec2> centerline, double lineWidth)
    : name_(std::move(name)),
      description_(std::move(description)),
      points_(std::move(centerline)),
      lineWidth_(lineWidth) {
  if (points_.size() < 3) throw std::invalid_argument("Una pista necesita al menos 3 puntos");
  computeArcLengths();
  computeBounds();
  buildGrid();
}

void Track::computeArcLengths() {
  arcLengths_.resize(points_.size());
  totalLength_ = 0.0;
  for (std::size_t i = 0; i < segmentCount(); i++) {
    arcLengths_[i] = totalLength_;
    totalLength_ += (segmentEnd(i) - segmentStart(i)).length();
  }
}

void Track::computeBounds() {
  bounds_.min = bounds_.max = points_.front();
  for (const Vec2& p : points_) {
    bounds_.min = {std::min(bounds_.min.x, p.x), std::min(bounds_.min.y, p.y)};
    bounds_.max = {std::max(bounds_.max.x, p.x), std::max(bounds_.max.y, p.y)};
  }
}

long Track::cellIndex(int col, int row) const {
  if (col < 0 || row < 0 || col >= gridCols_ || row >= gridRows_) return -1;
  return static_cast<long>(row) * gridCols_ + col;
}

// Cada segmento se registra en todas las celdas que toca su caja envolvente
// ampliada por kGridMargin; así una consulta solo revisa su propia celda.
void Track::buildGrid() {
  gridOrigin_ = bounds_.min - Vec2{2 * kGridMargin, 2 * kGridMargin};
  gridCols_ = static_cast<int>((bounds_.max.x - gridOrigin_.x + 2 * kGridMargin) / kGridCell) + 1;
  gridRows_ = static_cast<int>((bounds_.max.y - gridOrigin_.y + 2 * kGridMargin) / kGridCell) + 1;
  gridCells_.assign(static_cast<std::size_t>(gridCols_) * gridRows_, {});

  for (std::size_t i = 0; i < segmentCount(); i++) {
    const Vec2& a = segmentStart(i);
    const Vec2& b = segmentEnd(i);
    const int col0 = static_cast<int>((std::min(a.x, b.x) - kGridMargin - gridOrigin_.x) / kGridCell);
    const int col1 = static_cast<int>((std::max(a.x, b.x) + kGridMargin - gridOrigin_.x) / kGridCell);
    const int row0 = static_cast<int>((std::min(a.y, b.y) - kGridMargin - gridOrigin_.y) / kGridCell);
    const int row1 = static_cast<int>((std::max(a.y, b.y) + kGridMargin - gridOrigin_.y) / kGridCell);
    for (int row = row0; row <= row1; row++) {
      for (int col = col0; col <= col1; col++) {
        const long cell = cellIndex(col, row);
        if (cell >= 0) gridCells_[static_cast<std::size_t>(cell)].push_back(static_cast<uint32_t>(i));
      }
    }
  }
}

double Track::distanceToLine(const Vec2& p) const {
  const int col = static_cast<int>(std::floor((p.x - gridOrigin_.x) / kGridCell));
  const int row = static_cast<int>(std::floor((p.y - gridOrigin_.y) / kGridCell));
  const long cell = cellIndex(col, row);
  if (cell < 0) return kFarDistance;

  double best = kFarDistance;
  for (uint32_t segment : gridCells_[static_cast<std::size_t>(cell)]) {
    best = std::min(best, projectOntoSegment(p, segmentStart(segment), segmentEnd(segment)).distance);
  }
  return best;
}

double Track::wrapArcLength(double arcLength) const {
  double s = std::fmod(arcLength, totalLength_);
  return s < 0.0 ? s + totalLength_ : s;
}

std::size_t Track::segmentAt(double arcLength) const {
  const double s = wrapArcLength(arcLength);
  auto it = std::upper_bound(arcLengths_.begin(), arcLengths_.end(), s);
  return static_cast<std::size_t>(std::distance(arcLengths_.begin(), it)) - 1;
}

TrackProjection Track::projectNear(const Vec2& p, double hintArcLength, double backWindow,
                                   double forwardWindow) const {
  const double averageSegment = totalLength_ / static_cast<double>(segmentCount());
  const long back = static_cast<long>(backWindow / averageSegment) + 1;
  const long forward = static_cast<long>(forwardWindow / averageSegment) + 1;
  const long center = static_cast<long>(segmentAt(hintArcLength));
  const long count = static_cast<long>(segmentCount());

  TrackProjection best;
  best.distance = std::numeric_limits<double>::max();
  for (long k = center - back; k <= center + forward; k++) {
    const std::size_t i = static_cast<std::size_t>(((k % count) + count) % count);
    const Vec2& a = segmentStart(i);
    const Vec2& b = segmentEnd(i);
    const SegmentProjection projection = projectOntoSegment(p, a, b);
    if (projection.distance >= best.distance) continue;

    const Vec2 direction = b - a;
    best.distance = projection.distance;
    best.arcLength = arcLengths_[i] + direction.length() * projection.t;
    const double side = direction.cross(p - a);
    best.signedOffset = side >= 0.0 ? projection.distance : -projection.distance;
  }
  return best;
}

double Track::nearestArcLength(const Vec2& p) const {
  return projectNear(p, 0.0, 0.0, totalLength_).arcLength;
}

Pose Track::poseAt(double arcLength) const {
  const std::size_t i = segmentAt(arcLength);
  const Vec2& a = segmentStart(i);
  const Vec2 direction = segmentEnd(i) - a;
  const double t = (wrapArcLength(arcLength) - arcLengths_[i]) / direction.length();
  Pose pose;
  pose.position = a + direction * t;
  pose.heading = direction.angle();
  return pose;
}

// Radio de giro mínimo medido sobre ventanas de ~5 cm (las esquinas vivas
// de 90° dan un radio casi nulo, que es justamente lo que se quiere mostrar).
double Track::minimumTurnRadius() const {
  const double window = 0.05;
  const std::size_t step = std::max<std::size_t>(1, static_cast<std::size_t>(window / (totalLength_ / segmentCount())));
  double minimum = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < segmentCount(); i++) {
    const Vec2 d1 = segmentEnd(i) - segmentStart(i);
    const std::size_t j = (i + step) % segmentCount();
    const Vec2 d2 = segmentEnd(j) - segmentStart(j);
    const double turn = std::fabs(wrapAngle(d2.angle() - d1.angle()));
    if (turn > 1e-6) minimum = std::min(minimum, window / turn);
  }
  return minimum;
}

}  // namespace sim
