/*
  Track.h - Pista cerrada representada por su eje central muestreado.

  Ofrece dos consultas:
    - distanceToLine(): distancia mínima a CUALQUIER tramo (lo que "ve" un
      sensor; incluye los cruces). Acelerada con una rejilla espacial.
    - projectNear(): proyección restringida a una ventana alrededor de la
      posición anterior, para medir el avance sin saltar de rama en un cruce.
*/
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "Geometry.h"

namespace sim {

struct Bounds {
  Vec2 min;
  Vec2 max;
};

struct TrackProjection {
  double arcLength = 0.0;
  double distance = 0.0;
  double signedOffset = 0.0;  // positivo = el punto está a la izquierda de la línea
};

class Track {
 public:
  static constexpr double kStandardLineWidth = 0.019;

  Track(std::string name, std::string description, std::vector<Vec2> centerline,
        double lineWidth = kStandardLineWidth);

  const std::string& name() const { return name_; }
  const std::string& description() const { return description_; }
  double lineWidth() const { return lineWidth_; }
  double length() const { return totalLength_; }
  const std::vector<Vec2>& points() const { return points_; }
  Bounds bounds() const { return bounds_; }

  double distanceToLine(const Vec2& p) const;
  TrackProjection projectNear(const Vec2& p, double hintArcLength, double backWindow,
                              double forwardWindow) const;
  double nearestArcLength(const Vec2& p) const;
  Pose poseAt(double arcLength) const;
  double minimumTurnRadius() const;

 private:
  static constexpr double kGridCell = 0.03;
  static constexpr double kGridMargin = 0.03;
  static constexpr double kFarDistance = 1.0;

  std::size_t segmentCount() const { return points_.size(); }
  const Vec2& segmentStart(std::size_t i) const { return points_[i]; }
  const Vec2& segmentEnd(std::size_t i) const { return points_[(i + 1) % points_.size()]; }
  std::size_t segmentAt(double arcLength) const;
  double wrapArcLength(double arcLength) const;

  void computeArcLengths();
  void computeBounds();
  void buildGrid();
  long cellIndex(int col, int row) const;

  std::string name_;
  std::string description_;
  std::vector<Vec2> points_;
  std::vector<double> arcLengths_;  // arcLengths_[i] = s al inicio del segmento i
  double totalLength_ = 0.0;
  double lineWidth_;
  Bounds bounds_;

  Vec2 gridOrigin_;
  int gridCols_ = 0;
  int gridRows_ = 0;
  std::vector<std::vector<uint32_t>> gridCells_;
};

}  // namespace sim
