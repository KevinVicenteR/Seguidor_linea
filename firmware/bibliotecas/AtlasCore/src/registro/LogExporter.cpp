#include "LogExporter.h"

namespace atlas {

namespace {

constexpr uint8_t kCurvaturesPerLine = 32;

const char* modeName(uint8_t mode) {
  switch (mode) {
    case 1:
      return "reconocimiento";
    case 2:
      return "mapa";
    default:
      return "normal";
  }
}

void exportRuns(ITextSink& out, RunLog& log) {
  const uint8_t count = log.count();
  for (uint8_t i = 0; i < count; i++) {
    const RunRecord run = log.at(i);
    out.text("CARRERA,");
    out.number(i + 1);
    out.text(",");
    out.text(modeName(run.mode));
    out.text(run.failed ? ",falla," : ",ok,");
    out.number(run.timeCentiseconds);
    out.text(",");
    out.number(run.failed ? run.failureBin : -1);
    out.text(",");
    out.number(run.mapLength);
    out.endLine();
  }
}

void exportMap(ITextSink& out, const TrackMap& map) {
  out.text("MAPA,");
  out.number(map.isValid() ? map.length() : 0);
  out.text(",");
  out.number(map.lostPerMille());
  out.text(",");
  out.number(map.gapCount());
  out.text(",");
  out.number(map.curveSpeedPercent());
  out.endLine();
  if (!map.isValid()) return;

  out.text("HUECOS");
  for (uint8_t i = 0; i < map.gapCount(); i++) {
    out.text(",");
    out.number(map.gap(i));
  }
  out.endLine();

  for (uint16_t start = 0; start < map.length(); start += kCurvaturesPerLine) {
    out.text("CURVAS,");
    out.number(start);
    for (uint16_t i = start; i < start + kCurvaturesPerLine && i < map.length(); i++) {
      out.text(",");
      out.number(map.curvature(i));
    }
    out.endLine();
  }
}

}

void exportRunLog(ITextSink& out, RunLog& log, const TrackMap& map) {
  out.text("ATLAS-REGISTRO,1");
  out.endLine();
  exportRuns(out, log);
  exportMap(out, map);
  out.text("FIN");
  out.endLine();
}

}
