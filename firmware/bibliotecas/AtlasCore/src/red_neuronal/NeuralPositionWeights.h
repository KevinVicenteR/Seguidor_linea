#pragma once

#include <stdint.h>

#include "Progmem.h"
#include "TinyMlp.h"

namespace atlas {
namespace neural {

constexpr uint8_t kPositionInputs = 7;
constexpr uint8_t kPositionHidden = 8;

ATLAS_PROGMEM const int8_t kPositionWeights1[kPositionHidden * kPositionInputs] = {
    -60, -3, 47, 81, -5, -70, -45,
    -19, -9, 35, -30, -65, 62, 59,
    -11, 103, -87, 0, -51, -7, -30,
    39, -5, -71, -90, -25, 127, 23,
    -94, -23, 26, -103, -12, 23, -87,
    -56, -77, -21, 108, 32, 13, -53,
    -17, 22, 53, 34, -90, 57, -54,
    9, 44, -64, -91, 57, 122, 68};
ATLAS_PROGMEM const int32_t kPositionBias1[kPositionHidden] = {
    -825, 45, 7980, 710, 1306,
    1042, 2067, 8705};
ATLAS_PROGMEM const int16_t kPositionWeights2[kPositionHidden] = {
    -4378, 2738, 4785, -177, -6854, 3730, -4220, 3141};
ATLAS_PROGMEM const int32_t kPositionBias2[1] = {
    6342512};

const TinyMlpWeights kPositionNet = {kPositionInputs, kPositionHidden, 1,
                                     kPositionWeights1, kPositionBias1, 1,
                                     kPositionWeights2, kPositionBias2, 16};

}
}
