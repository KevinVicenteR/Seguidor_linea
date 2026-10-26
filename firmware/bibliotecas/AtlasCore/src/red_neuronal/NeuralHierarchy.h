#pragma once

#include <stdint.h>

#include "TinyMlp.h"

namespace atlas {

constexpr uint8_t kSituations = 4;
constexpr uint8_t kHierarchyInputs = 8;
constexpr uint8_t kHierarchyMinPercent = 70;
constexpr uint8_t kHierarchyMaxPercent = 130;

struct NeuralHierarchy {
  TinyMlpWeights gate;
  TinyMlpWeights experts[kSituations];
};

void evaluateHierarchy(const NeuralHierarchy& hierarchy, const int8_t* input, uint8_t* kpPercent,
                       uint8_t* kdPercent, uint8_t* weights = nullptr);

}
