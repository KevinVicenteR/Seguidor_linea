#pragma once

#include <stdint.h>

#include "NeuralHierarchy.h"
#include "Progmem.h"

namespace atlas {
namespace neural {

constexpr uint8_t kGateHidden = 6;
constexpr uint8_t kExpertHidden = 4;

ATLAS_PROGMEM const int8_t kGateW1[kGateHidden * kHierarchyInputs] = {};
ATLAS_PROGMEM const int32_t kGateB1[kGateHidden] = {};
ATLAS_PROGMEM const int16_t kGateW2[kSituations * kGateHidden] = {};
ATLAS_PROGMEM const int32_t kGateB2[kSituations] = {64L << 8, 64L << 8, 64L << 8, 64L << 8};

ATLAS_PROGMEM const int8_t kExpertW1[kSituations][kExpertHidden * kHierarchyInputs] = {};
ATLAS_PROGMEM const int32_t kExpertB1[kSituations][kExpertHidden] = {};
ATLAS_PROGMEM const int16_t kExpertW2[kSituations][2 * kExpertHidden] = {};
ATLAS_PROGMEM const int32_t kExpertB2[kSituations][2] = {
    {100L << 8, 100L << 8}, {100L << 8, 100L << 8}, {100L << 8, 100L << 8}, {100L << 8, 100L << 8}};

const NeuralHierarchy kHierarchy = {
    {kHierarchyInputs, kGateHidden, kSituations, kGateW1, kGateB1, 7, kGateW2, kGateB2, 8},
    {{kHierarchyInputs, kExpertHidden, 2, kExpertW1[0], kExpertB1[0], 7, kExpertW2[0], kExpertB2[0], 8},
     {kHierarchyInputs, kExpertHidden, 2, kExpertW1[1], kExpertB1[1], 7, kExpertW2[1], kExpertB2[1], 8},
     {kHierarchyInputs, kExpertHidden, 2, kExpertW1[2], kExpertB1[2], 7, kExpertW2[2], kExpertB2[2], 8},
     {kHierarchyInputs, kExpertHidden, 2, kExpertW1[3], kExpertB1[3], 7, kExpertW2[3], kExpertB2[3], 8}}};

}
}
