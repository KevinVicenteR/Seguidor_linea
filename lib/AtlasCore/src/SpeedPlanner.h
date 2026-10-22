/*
  SpeedPlanner.h - Velocidad por tramo para las vueltas guiadas por el mapa.

  1. Límite de cada tramo según su curvatura: más cerrada = más lenta
     (v ∝ 1/√curvatura, como la adherencia lateral).
  2. Anticipación: la velocidad en un tramo nunca supera la que permite
     frenar a tiempo para las curvas de los siguientes tramos.
  3. Confianza: si en el reconocimiento el robot perdió mucho la línea, todo
     el plan se escala hacia abajo.
*/
#pragma once

#include <stdint.h>

#include "parametros/Config.h"
#include "TrackMap.h"

namespace atlas {

class SpeedPlanner {
 public:
  explicit SpeedPlanner(const PlannerConfig& config) : config_(&config) {}
  void configure(const PlannerConfig& config) { config_ = &config; }

  // curveSpeedPercent escala solo las curvas (aprendido de salidas anteriores).
  int16_t limitFor(uint8_t curvature, uint8_t curveSpeedPercent = 100) const;
  // % de velocidad según qué tan difícil fue el reconocimiento.
  uint8_t confidencePercent(const TrackMap& map) const;
  // Velocidad planeada para el tramo `bin` (fuera del mapa: velocidad segura).
  // El mapa es una vuelta cerrada: cerca del final mira el inicio.
  int16_t speedAt(const TrackMap& map, uint16_t bin) const;

 private:
  const PlannerConfig* config_;
};

}  // namespace atlas
