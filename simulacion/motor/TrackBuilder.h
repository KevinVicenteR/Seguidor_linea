#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "Geometry.h"

namespace sim {

class EdgeFeature {
 public:
  virtual ~EdgeFeature() = default;
  virtual std::vector<Vec2> localPath(double length, double step) const = 0;
};

using EdgeFeaturePtr = std::shared_ptr<const EdgeFeature>;

namespace features {
EdgeFeaturePtr sineWave(double amplitude, int waves);
EdgeFeaturePtr zigzag(double amplitude, int teeth, double filletRadius = 0.0);
EdgeFeaturePtr squareSteps(double height, int steps);
EdgeFeaturePtr loops(int count, double size, bool toLeft = true);
EdgeFeaturePtr semicircles(int count);
}

class FilletedLoopBuilder {
 public:
  explicit FilletedLoopBuilder(double sampleStep = 0.005) : step_(sampleStep) {}

  FilletedLoopBuilder& corner(double x, double y, double filletRadius);
  FilletedLoopBuilder& sharp(double x, double y) { return corner(x, y, 0.0); }
  FilletedLoopBuilder& with(EdgeFeaturePtr feature);

  std::vector<Vec2> build() const;

 private:
  struct Waypoint {
    Vec2 position;
    double radius;
    EdgeFeaturePtr feature;
  };

  double step_;
  std::vector<Waypoint> waypoints_;
};

std::vector<Vec2> sampleFilletedPolyline(const std::vector<Vec2>& points, double radius, double step);

std::vector<Vec2> buildParametricLoop(const std::function<Vec2(double)>& curve, double sampleStep = 0.005);

}
