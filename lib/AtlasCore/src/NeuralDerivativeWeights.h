/*
  NeuralDerivativeWeights.h - GENERADO por simulation/tools/atlas_train_nn.cpp
  (atlas_train_nn derivada). No editar a mano.

  Red 7 -> 8 ReLU -> 1: cambio del error por ciclo de control
  (adelanto 0 ciclos), desde las pendientes del historial del error.
  Entrenada el 2026-10-17 con 400000 ciclos simulados (parada temprana).
  Validación (RMS en unidades de error por ciclo):
    D filtrado clásico 58.2745, red 10.0034 (máx 600.113)
*/
#pragma once

#include <stdint.h>

#include "Progmem.h"
#include "TinyMlp.h"

namespace atlas {
namespace neural {

constexpr bool kDerivativeTrained = true;
constexpr uint8_t kDerivativeInputs = 7;
constexpr uint8_t kDerivativeHidden = 8;

ATLAS_PROGMEM const int8_t kDerivativeWeights1[kDerivativeHidden * kDerivativeInputs] = {
    -7, -11, -48, -108, 16, -93, -28, 
    5, 16, 46, 22, 30, 79, 30, 
    3, -2, -20, -85, 10, 1, -39, 
    9, 18, 35, -93, -14, 99, 64, 
    5, 0, 17, 52, -89, 12, -33, 
    -23, -28, 6, 52, 2, -76, -52, 
    -4, 9, 18, 68, 19, 0, -40, 
    1, 25, 31, 72, -127, -59, 65};
ATLAS_PROGMEM const int32_t kDerivativeBias1[kDerivativeHidden] = {
    -2308, 1095, 2783, 22, 2394, 
    -2602, 2921, -4380};
ATLAS_PROGMEM const int16_t kDerivativeWeights2[kDerivativeHidden] = {
    -2828, 4093, 1557, -3047, -2171, -446, -1545, 3764};
ATLAS_PROGMEM const int32_t kDerivativeBias2[1] = {
    314346};

const TinyMlpWeights kDerivativeNet = {kDerivativeInputs, kDerivativeHidden, 1,
                                       kDerivativeWeights1, kDerivativeBias1, 2,
                                       kDerivativeWeights2, kDerivativeBias2, 16};

}  // namespace neural
}  // namespace atlas
