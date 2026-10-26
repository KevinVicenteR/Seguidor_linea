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

struct TrackGap {
  Vec2 from;
  Vec2 to;
};

struct TrackProjection {
  double arcLength = 0.0;
  double distance = 0.0;
  double signedOffset = 0.0;
};

class Track {
 public:
  static constexpr double kStandardLineWidth = 0.019;

  Track(std::string name, std::string description, std::vector<Vec2> centerline,
        double lineWidth = kStandardLineWidth, const std::vector<TrackGap>& gaps = {});

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
  bool isPainted(std::size_t segment) const { return painted_[segment]; }
  bool hasGaps() const;

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
  void markGaps(const std::vector<TrackGap>& gaps);
  void buildGrid();
  long cellIndex(int col, int row) const;

  std::string name_;
  std::string description_;
  std::vector<Vec2> points_;
  std::vector<double> arcLengths_;
  std::vector<bool> painted_;
  double totalLength_ = 0.0;
  double lineWidth_;
  Bounds bounds_;

  Vec2 gridOrigin_;
  int gridCols_ = 0;
  int gridRows_ = 0;
  std::vector<std::vector<uint32_t>> gridCells_;
};

}
