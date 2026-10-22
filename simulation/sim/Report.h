/*
  Report.h - Exporta los resultados: visor HTML interactivo (con los datos
  embebidos en JSON) y un resumen CSV.
*/
#pragma once

#include <string>
#include <vector>

#include "parametros/Config.h"
#include "RobotModel.h"
#include "Scenario.h"
#include "TrackLibrary.h"

namespace sim {

struct ReportContext {
  const std::vector<TrackEntry>& tracks;
  const std::vector<ScenarioResult>& results;
  const atlas::RobotConfig& firmware;
  const RobotParams& robot;
};

std::string buildReportJson(const ReportContext& context);
void writeViewer(const std::string& templatePath, const std::string& outputPath, const ReportContext& context);
void writeSummaryCsv(const std::string& outputPath, const ReportContext& context);
// Analizador de registros del robot real (pegar la salida del modo diagnóstico).
void writeAnalyzer(const std::string& templatePath, const std::string& outputPath,
                   const std::vector<TrackEntry>& tracks, const std::string& exampleLog);

}  // namespace sim
