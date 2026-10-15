/*
  NeuralPositionWeights.h - GENERADO por simulation/tools/atlas_train_nn.cpp
  (atlas_train_nn posicion). No editar a mano.

  Red 7 -> 8 ReLU -> 1: posición fina de la línea.
  Entrenada el 2026-10-15 con 2836989 lecturas simuladas.
  Validación (posición, RMS en milésimas de sensor; 1000 = 9 mm):
    promedio ponderado 146, red 59 (máx 649)
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
    -38, -9, 34, 42, -20, -29, -29, 
    -12, -9, 22, -20, -42, 44, 38, 
    -7, 80, -88, 30, -71, -23, -19, 
    25, -3, -46, -58, -16, 82, 15, 
    -61, -15, 17, -67, -8, 15, -56, 
    -36, 9, -52, 45, 39, 125, -34, 
    -11, 112, 20, 50, -55, 127, -35, 
    6, 3, -21, -72, -5, 60, 44};
ATLAS_PROGMEM const int32_t kPositionBias1[kPositionHidden] = {
    1565, -62, 6971, 465, 843, 
    -1563, -1751, 10059};
ATLAS_PROGMEM const int16_t kPositionWeights2[kPositionHidden] = {
    -1382, 2277, 2547, -90, -5311, 3074, -2145, 2442};
ATLAS_PROGMEM const int32_t kPositionBias2[1] = {
    2822771};

const TinyMlpWeights kPositionNet = {kPositionInputs, kPositionHidden, 1,
                                     kPositionWeights1, kPositionBias1, 1,
                                     kPositionWeights2, kPositionBias2, 15};

}  // namespace neural
}  // namespace atlas
