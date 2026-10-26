#include "Report.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace sim {

namespace {

constexpr const char* kDataPlaceholder = "/*__SIM_DATA__*/null";
constexpr std::size_t kTrackPointStride = 2;

std::string escapeJson(const std::string& text) {
  std::string out;
  for (char c : text) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      default:
        out += c;
    }
  }
  return out;
}

std::string quoted(const std::string& text) { return "\"" + escapeJson(text) + "\""; }

std::string number(double value, int decimals = 3) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(decimals) << value;
  return out.str();
}

long rounded(double value) { return std::lround(value); }

template <typename T, typename F>
std::string jsonArray(const std::vector<T>& items, F toJson) {
  std::string out = "[";
  for (std::size_t i = 0; i < items.size(); i++) {
    if (i) out += ",";
    out += toJson(items[i]);
  }
  return out + "]";
}

std::string trackJson(const TrackEntry& entry) {
  const Track& track = entry.track;
  std::ostringstream points;
  std::ostringstream painted;
  for (std::size_t i = 0; i < track.points().size(); i += kTrackPointStride) {
    if (i) {
      points << ",";
      painted << ",";
    }
    points << rounded(track.points()[i].x * 1000) << "," << rounded(track.points()[i].y * 1000);
    painted << (track.isPainted(i) ? 1 : 0);
  }
  std::ostringstream out;
  out << "{\"name\":" << quoted(track.name()) << ",\"description\":" << quoted(track.description())
      << ",\"length\":" << number(track.length(), 2) << ",\"minRadius\":" << number(track.minimumTurnRadius(), 3)
      << ",\"laps\":" << entry.laps << ",\"lineWidth\":" << number(track.lineWidth(), 3)
      << ",\"startArc\":" << number(track.nearestArcLength(entry.startPoint), 3) << ",\"points\":["
      << points.str() << "],\"painted\":[" << painted.str() << "]}";
  return out.str();
}

std::string telemetryJson(const std::vector<TelemetrySample>& samples) {
  std::ostringstream t, x, y, h, v, o, f;
  for (std::size_t i = 0; i < samples.size(); i++) {
    const TelemetrySample& s = samples[i];
    const char* sep = i ? "," : "";
    t << sep << rounded(s.time * 1000);
    x << sep << rounded(s.x * 1000);
    y << sep << rounded(s.y * 1000);
    h << sep << rounded(s.heading * 1000);
    v << sep << rounded(s.speed * 100);
    o << sep << rounded(s.lateralOffset * 1000);
    f << sep << ((s.trackable ? 1 : 0) | (s.sliding ? 2 : 0));
  }
  return "\"t\":[" + t.str() + "],\"x\":[" + x.str() + "],\"y\":[" + y.str() + "],\"h\":[" + h.str() +
         "],\"v\":[" + v.str() + "],\"o\":[" + o.str() + "],\"f\":[" + f.str() + "]";
}

std::string resultJson(const ScenarioResult& r) {
  std::ostringstream out;
  out << "{\"label\":" << quoted(r.spec.label) << ",\"track\":" << r.spec.trackIndex << ",\"pwm\":" << r.spec.cruisePwm
      << ",\"lineColor\":" << quoted(r.spec.lineColor == atlas::LineColor::White ? "blanca" : "negra")
      << ",\"impeller\":" << r.spec.impellerPwm << ",\"required\":" << (r.spec.required ? "true" : "false")
      << ",\"passed\":" << (r.passed() ? "true" : "false") << ",\"completed\":" << (r.completed ? "true" : "false")
      << ",\"failure\":" << quoted(r.failureReason)
      << ",\"lapTimes\":" << jsonArray(r.lapTimes, [](double lap) { return number(lap, 3); })
      << ",\"raceTime\":" << number(r.raceTime, 3) << ",\"distance\":" << number(r.distance, 2)
      << ",\"avgSpeed\":" << number(r.averageSpeed, 3) << ",\"maxSpeed\":" << number(r.maxSpeed, 3)
      << ",\"rmsOffsetMm\":" << number(r.rmsOffsetMm, 1) << ",\"maxOffsetMm\":" << number(r.maxOffsetMm, 1) << ",\"p95OffsetMm\":" << number(r.p95OffsetMm, 1)
      << ",\"lostLinePct\":" << number(r.lostLinePercent, 1) << ",\"slidingPct\":" << number(r.slidingPercent, 1)
      << ",\"zigzagPerMeter\":" << number(r.zigzagPerMeter, 2) << ",\"steeringRms\":" << number(r.steeringRmsPwm, 1)
      << ",\"calibratedSensors\":" << r.calibratedSensors
      << ",\"stoppedCleanly\":" << (r.stoppedCleanly ? "true" : "false") << "," << telemetryJson(r.telemetry) << "}";
  return out.str();
}

std::string configJson(const atlas::RobotConfig& firmware, const RobotParams& robot) {
  std::ostringstream out;
  out << "{\"kp\":" << firmware.control.kp << ",\"kd\":" << firmware.control.kd
      << ",\"loopPeriodUs\":" << firmware.control.loopPeriodUs << ",\"maxForwardPwm\":" << firmware.control.maxForwardPwm
      << ",\"maxBackwardPwm\":" << firmware.control.maxBackwardPwm << ",\"wheelBase\":" << robot.wheelBase
      << ",\"sensorBarAhead\":" << robot.sensorBarAhead << ",\"sensorPitch\":" << robot.sensorPitch
      << ",\"maxWheelSpeed\":" << robot.maxWheelSpeed << ",\"sensorCount\":" << static_cast<int>(atlas::kSensorCount)
      << "}";
  return out.str();
}

}

std::string buildReportJson(const ReportContext& context) {
  return "{\"config\":" + configJson(context.firmware, context.robot) +
         ",\"tracks\":" + jsonArray(context.tracks, trackJson) + ",\"scenarios\":" + jsonArray(context.results, resultJson) +
         "}";
}

void fillTemplate(const std::string& templatePath, const std::string& outputPath, const std::string& json) {
  std::ifstream in(templatePath);
  if (!in) throw std::runtime_error("No se encontró la plantilla: " + templatePath);
  std::stringstream buffer;
  buffer << in.rdbuf();
  std::string html = buffer.str();

  const std::size_t at = html.find(kDataPlaceholder);
  if (at == std::string::npos) throw std::runtime_error("La plantilla no contiene el marcador de datos");
  html.replace(at, std::string(kDataPlaceholder).size(), json);

  std::ofstream out(outputPath);
  if (!out) throw std::runtime_error("No se pudo escribir " + outputPath);
  out << html;
}

void writeViewer(const std::string& templatePath, const std::string& outputPath, const ReportContext& context) {
  fillTemplate(templatePath, outputPath, buildReportJson(context));
}

void writeAnalyzer(const std::string& templatePath, const std::string& outputPath,
                   const std::vector<TrackEntry>& tracks, const std::string& exampleLog) {
  fillTemplate(templatePath, outputPath,
               "{\"tracks\":" + jsonArray(tracks, trackJson) + ",\"exampleLog\":" + quoted(exampleLog) + "}");
}

void writeSummaryCsv(const std::string& outputPath, const ReportContext& context) {
  std::ofstream out(outputPath);
  if (!out) throw std::runtime_error("No se pudo escribir " + outputPath);
  out << "escenario,pista,pwm,linea,impeller,obligatorio,aprobado,tiempo_s,vel_media_mps,vel_max_mps,"
         "error_rms_mm,error_max_mm,linea_perdida_pct,derrape_pct,falla\n";
  for (const ScenarioResult& r : context.results) {
    out << quoted(r.spec.label) << "," << quoted(r.trackName) << "," << r.spec.cruisePwm << ","
        << (r.spec.lineColor == atlas::LineColor::White ? "blanca" : "negra") << "," << r.spec.impellerPwm << ","
        << (r.spec.required ? "si" : "no") << "," << (r.passed() ? "si" : "no") << "," << number(r.raceTime, 3) << ","
        << number(r.averageSpeed, 3) << "," << number(r.maxSpeed, 3) << "," << number(r.rmsOffsetMm, 1) << ","
        << number(r.maxOffsetMm, 1) << "," << number(r.lostLinePercent, 1) << "," << number(r.slidingPercent, 1)
        << "," << quoted(r.failureReason) << "\n";
  }
}

}
