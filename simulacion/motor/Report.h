#pragma once

#include <string>
#include <vector>

#include "Config.h"
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
void writeAnalyzer(const std::string& templatePath, const std::string& outputPath,
                   const std::vector<TrackEntry>& tracks, const std::string& exampleLog);

}
