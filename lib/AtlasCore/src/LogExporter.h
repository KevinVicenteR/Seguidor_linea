/*
  LogExporter.h - Exporta el registro de carreras y el mapa como texto
  (líneas separadas por comas) para analizarlos en la PC con
  simulation_output/analizador.html.

    ATLAS-REGISTRO,1
    CARRERA,n,modo,resultado,tiempo_cs,tramo_falla,largo_mapa
    MAPA,largo,sin_linea_por_mil,huecos,velocidad_curvas_%
    HUECOS,t1,t2,...
    CURVAS,inicio,c,c,c,...
    FIN
*/
#pragma once

#include <stdint.h>

#include "RunLog.h"
#include "TrackMap.h"

namespace atlas {

class ITextSink {
 public:
  virtual ~ITextSink() {}
  virtual void text(const char* value) = 0;
  virtual void number(int32_t value) = 0;
  virtual void endLine() = 0;
};

void exportRunLog(ITextSink& out, RunLog& log, const TrackMap& map);

}  // namespace atlas
