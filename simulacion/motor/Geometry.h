#pragma once

#include <cmath>

namespace sim {

constexpr double kPi = 3.14159265358979323846;

struct Vec2 {
  double x = 0.0;
  double y = 0.0;

  Vec2() = default;
  Vec2(double xValue, double yValue) : x(xValue), y(yValue) {}

  Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  Vec2 operator*(double k) const { return {x * k, y * k}; }

  double dot(const Vec2& o) const { return x * o.x + y * o.y; }
  double cross(const Vec2& o) const { return x * o.y - y * o.x; }
  double length() const { return std::hypot(x, y); }
  double angle() const { return std::atan2(y, x); }

  Vec2 normalized() const {
    const double len = length();
    return len > 0.0 ? Vec2{x / len, y / len} : Vec2{};
  }
  Vec2 perpendicularLeft() const { return {-y, x}; }

  static Vec2 fromAngle(double radians) { return {std::cos(radians), std::sin(radians)}; }
};

struct Pose {
  Vec2 position;
  double heading = 0.0;

  Vec2 forward() const { return Vec2::fromAngle(heading); }
  Vec2 left() const { return forward().perpendicularLeft(); }

  Vec2 toWorld(double ahead, double leftOffset) const {
    return position + forward() * ahead + left() * leftOffset;
  }
};

struct SegmentProjection {
  double t = 0.0;
  double distance = 0.0;
};

inline SegmentProjection projectOntoSegment(const Vec2& p, const Vec2& a, const Vec2& b) {
  const Vec2 ab = b - a;
  const double lengthSq = ab.dot(ab);
  double t = lengthSq > 0.0 ? (p - a).dot(ab) / lengthSq : 0.0;
  t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
  return {t, (p - (a + ab * t)).length()};
}

inline double wrapAngle(double radians) {
  while (radians > kPi) radians -= 2.0 * kPi;
  while (radians < -kPi) radians += 2.0 * kPi;
  return radians;
}

}
