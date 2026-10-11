/*
  TrackBuilder.h - Construcción de pistas cerradas combinando figuras.

  FilletedLoopBuilder recibe un polígono de waypoints (triángulo, hexágono,
  estrella...). Cada vértice se redondea con un arco (radio 0 = esquina viva)
  y cada lado puede decorarse con una figura (EdgeFeature): senoide, zigzag,
  escalones, rizos o semicírculos. Las figuras empiezan y terminan sobre el
  lado, así el cierre de la pista siempre es exacto. Si el polígono se
  auto-intersecta (estrella, ocho) se obtienen cruces reales.
*/
#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "Geometry.h"

namespace sim {

// Figura sobre un lado recto. Coordenadas locales: u a lo largo del lado
// (0 ... length), v a la izquierda. Debe empezar en (0,0) y terminar en
// (length,0) con la dirección del lado; devuelve puntos sin el punto final.
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
}  // namespace features

class FilletedLoopBuilder {
 public:
  explicit FilletedLoopBuilder(double sampleStep = 0.005) : step_(sampleStep) {}

  FilletedLoopBuilder& corner(double x, double y, double filletRadius);
  FilletedLoopBuilder& sharp(double x, double y) { return corner(x, y, 0.0); }
  // Decora el lado que sale del último waypoint agregado.
  FilletedLoopBuilder& with(EdgeFeaturePtr feature);

  // Lanza std::invalid_argument si los arcos no caben en sus lados.
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

// Polilínea abierta con esquinas redondeadas (radio constante, 0 = vivas).
std::vector<Vec2> sampleFilletedPolyline(const std::vector<Vec2>& points, double radius, double step);

std::vector<Vec2> buildParametricLoop(const std::function<Vec2(double)>& curve, double sampleStep = 0.005);

}  // namespace sim
