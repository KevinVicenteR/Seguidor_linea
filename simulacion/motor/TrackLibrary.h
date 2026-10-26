#pragma once

#include <vector>

#include "Track.h"

namespace sim {

struct TrackEntry {
  Track track;
  Vec2 startPoint;
  int laps;
};

std::vector<TrackEntry> buildTrackLibrary();
std::vector<TrackEntry> buildExtremeTrackLibrary();

Vec2 competitionGapCenter();

}
