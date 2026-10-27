/*
  NeuralPositionWeights.h - GENERADO por entrenamiento/algoritmos/atlas_train_nn.cpp
  (atlas_train_nn posicion). No editar a mano.

  Red 7 -> 8 ReLU -> 1: posición fina de la línea.
  Entrenada el 2026-10-27 con 100000 lecturas simuladas (parada temprana).
  Validación (posición, RMS en milésimas de sensor; 1000 = 9 mm):
    promedio ponderado 150, red 56 (máx 802)
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
    -36, -28, 31, 44, -10, -49, -27, 
    -12, -4, 23, -18, -38, 42, 36, 
    -7, 75, -72, 15, -49, -17, -18, 
    24, -62, -1, -57, -29, 60, 14, 
    -57, -14, 16, -63, -7, 14, -53, 
    -34, -29, -40, 54, 27, 66, -32, 
    -10, 64, 21, 16, -48, 114, -33, 
    6, 13, -15, -46, 16, 127, 41};
ATLAS_PROGMEM const int32_t kPositionBias1[kPositionHidden] = {
    934, 173, 5616, 8488, 794, 
    -483, 1719, 4875};
ATLAS_PROGMEM const int16_t kPositionWeights2[kPositionHidden] = {
    -2569, 2885, 3623, 2134, -5637, 2896, -3028, 1994};
ATLAS_PROGMEM const int32_t kPositionBias2[1] = {
    5254056};

const TinyMlpWeights kPositionNet = {kPositionInputs, kPositionHidden, 1,
                                     kPositionWeights1, kPositionBias1, 1,
                                     kPositionWeights2, kPositionBias2, 15};

}  // namespace neural
}  // namespace atlas
