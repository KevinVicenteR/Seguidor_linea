/*
  TrackLibrary.h - Catálogo de pistas de prueba (largas y difíciles).
*/
#pragma once

#include <vector>

#include "Track.h"

namespace sim {

struct TrackEntry {
  Track track;
  Vec2 startPoint;  // la barra de sensores arranca centrada aquí
  int laps;
};

std::vector<TrackEntry> buildTrackLibrary();

// Centro del hueco inferior de la pista de competencia (para salir desde ahí).
Vec2 competitionGapCenter();

}  // namespace sim
