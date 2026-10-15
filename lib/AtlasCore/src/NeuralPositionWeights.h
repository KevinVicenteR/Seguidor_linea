/*
  NeuralPositionWeights.h - GENERADO por simulation/tools/atlas_train_nn.cpp
  (atlas_train_nn posicion). No editar a mano.

  Red 7 -> 8 ReLU -> 1: posición fina de la línea.
  Entrenada el 2026-10-15 con 25000 lecturas simuladas (parada temprana).
  Validación (posición, RMS en milésimas de sensor; 1000 = 9 mm):
    promedio ponderado 147, red 59 (máx 765)
*/
#pragma once

#include <stdint.h>

#include "Progmem.h"
#include "TinyMlp.h"

namespace atlas {
namespace neural {

constexpr uint8_t kPositionInputs = 7;
constexpr uint8_t kPositionHidden = 8;

ATLAS_PROGMEM const int8_t kPositionWeights1[kPositionHidden * kPositionInputs] = {
    -45, -3, 37, 54, 0, -72, -34, 
    -15, -10, 26, -23, -49, 51, 45, 
    -8, -69, -53, -16, -36, -2, -23, 
    29, -3, -54, -68, -19, 96, 18, 
    -71, -17, 19, -78, -9, 17, -66, 
    -42, -58, -24, 83, 21, 32, -40, 
    -13, 51, 15, 33, -46, 109, -41, 
    7, 35, -40, -50, 45, 127, 51};
ATLAS_PROGMEM const int32_t kPositionBias1[kPositionHidden] = {
    698, -82, 7454, 535, 988, 
    27, -494, 6555};
ATLAS_PROGMEM const int16_t kPositionWeights2[kPositionHidden] = {
    -2672, 1518, 3891, 22, -4527, 2530, -2411, 1985};
ATLAS_PROGMEM const int32_t kPositionBias2[1] = {
    1588102};

const TinyMlpWeights kPositionNet = {kPositionInputs, kPositionHidden, 1,
                                     kPositionWeights1, kPositionBias1, 1,
                                     kPositionWeights2, kPositionBias2, 15};

}  // namespace neural
}  // namespace atlas
