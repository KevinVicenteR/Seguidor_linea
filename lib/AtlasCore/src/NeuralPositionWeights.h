/*
  NeuralPositionWeights.h - GENERADO por simulation/tools/atlas_train_nn.cpp
  (atlas_train_nn posicion). No editar a mano.

  Red 7 -> 8 ReLU -> 1: posición fina de la línea.
  Entrenada el 2026-10-15 con 40000 lecturas simuladas (parada temprana).
  Validación (posición, RMS en milésimas de sensor; 1000 = 9 mm):
    promedio ponderado 147, red 59 (máx 784)
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
    -45, -36, 38, 53, -11, -60, -34, 
    -15, -10, 26, -23, -50, 54, 45, 
    -9, 127, -112, 20, -63, -29, -23, 
    29, -3, -54, -68, -19, 97, 18, 
    -72, -17, 20, -78, -9, 17, -66, 
    -42, -36, -57, 66, 33, 97, -40, 
    -13, 64, 21, 61, -61, 106, -41, 
    7, 2, -22, -68, 4, 117, 52};
ATLAS_PROGMEM const int32_t kPositionBias1[kPositionHidden] = {
    1277, -83, 7449, 565, 992, 
    -1049, -2404, 8669};
ATLAS_PROGMEM const int16_t kPositionWeights2[kPositionHidden] = {
    -1774, 1729, 2301, -64, -4511, 2449, -2570, 1773};
ATLAS_PROGMEM const int32_t kPositionBias2[1] = {
    5107748};

const TinyMlpWeights kPositionNet = {kPositionInputs, kPositionHidden, 1,
                                     kPositionWeights1, kPositionBias1, 1,
                                     kPositionWeights2, kPositionBias2, 15};

}  // namespace neural
}  // namespace atlas
