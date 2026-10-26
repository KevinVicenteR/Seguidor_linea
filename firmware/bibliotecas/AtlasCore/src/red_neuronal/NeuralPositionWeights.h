#pragma once

#include <stdint.h>

#include "Progmem.h"
#include "TinyMlp.h"

namespace atlas {
namespace neural {

constexpr uint8_t kPositionInputs = 7;
constexpr uint8_t kPositionHidden = 8;

ATLAS_PROGMEM const int8_t kPositionWeights1[kPositionHidden * kPositionInputs] = {
    -42, -30, 38, 50, -3, -66, -31,
    -13, -4, 24, -22, -45, 47, 41,
    -8, 71, -71, 7, -44, -22, -21,
    27, -3, -50, -63, -18, 88, 16,
    -66, -16, 18, -72, -8, 16, -61,
    -39, -32, -30, 76, 26, 52, -37,
    -12, 50, 15, 51, -44, 78, -37,
    6, 4, -20, -57, 16, 127, 47};
ATLAS_PROGMEM const int32_t kPositionBias1[kPositionHidden] = {
    828, -54, 5980, 458, 910,
    217, -2464, 5957};
ATLAS_PROGMEM const int16_t kPositionWeights2[kPositionHidden] = {
    -2918, 1928, 3530, -120, -4918, 3027, -3687, 1770};
ATLAS_PROGMEM const int32_t kPositionBias2[1] = {
    3410412};

const TinyMlpWeights kPositionNet = {kPositionInputs, kPositionHidden, 1,
                                     kPositionWeights1, kPositionBias1, 1,
                                     kPositionWeights2, kPositionBias2, 15};

}
}
