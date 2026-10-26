#include "NeuralHierarchy.h"

namespace atlas {

namespace {
int32_t clampPercent(int32_t value) {
  return value < kHierarchyMinPercent ? kHierarchyMinPercent : (value > kHierarchyMaxPercent ? kHierarchyMaxPercent : value);
}
}

void evaluateHierarchy(const NeuralHierarchy& hierarchy, const int8_t* input, uint8_t* kpPercent,
                       uint8_t* kdPercent, uint8_t* weights) {
  int32_t gate[kSituations];
  TinyMlp(hierarchy.gate).evaluate(input, gate);
  int32_t total = 0, kp = 0, kd = 0;
  for (uint8_t k = 0; k < kSituations; k++) {
    const int32_t weight = gate[k] < 0 ? 0 : (gate[k] > 255 ? 255 : gate[k]);
    if (weights != nullptr) weights[k] = static_cast<uint8_t>(weight);
    if (weight == 0) continue;
    int32_t expert[2];
    TinyMlp(hierarchy.experts[k]).evaluate(input, expert);
    kp += weight * clampPercent(expert[0]);
    kd += weight * clampPercent(expert[1]);
    total += weight;
  }
  *kpPercent = static_cast<uint8_t>(total ? kp / total : 100);
  *kdPercent = static_cast<uint8_t>(total ? kd / total : 100);
}

}
