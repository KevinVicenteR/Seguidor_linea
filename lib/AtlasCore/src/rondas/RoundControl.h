#pragma once
#include "../LineFollower.h"

namespace atlas {
enum class Round : uint8_t { First = 1, Second = 2, Third = 3 };

class RoundControl {
 public:
  // Sin mapa siempre se reconoce. Un mapa cargado al encender permite ir a la segunda.
  Round next(bool hasMap) const {
    if (!hasMap) return Round::First;
    return guidedCompleted_ ? Round::Third : Round::Second;
  }
  const RobotConfig& config(bool hasMap, const RobotConfig& first,
                            const RobotConfig* second, const RobotConfig* third) const {
    if (!hasMap) return first;
    if (next(hasMap) == Round::Third && third != nullptr) return *third;
    return second != nullptr ? *second : first;
  }
  RunMode mode(bool mapAttached, bool hasMap) const {
    return !mapAttached ? RunMode::Plain : (hasMap ? RunMode::MapGuided : RunMode::Mapping);
  }
  void finish(bool guided) { if (guided) guidedCompleted_ = true; }
  void reset() { guidedCompleted_ = false; }

 private:
  bool guidedCompleted_ = false;
};
}  // namespace atlas
