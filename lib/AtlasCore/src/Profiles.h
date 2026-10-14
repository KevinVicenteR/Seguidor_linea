/*
  Profiles.h - Perfiles de manejo.

  - Seguro: ajustado para no fallar aunque el robot real sea más lento, tenga
    más inercia o más retraso que el modelo (optimización robusta).
  - Rápido: más agresivo; para cuando el robot ya se probó estable.

  Ambos se obtuvieron con simulation/tools/atlas_tune (--robusto / --rapido).
*/
#pragma once

#include "Config.h"

namespace atlas {

RobotConfig safeProfile();
RobotConfig fastProfile();

}  // namespace atlas
