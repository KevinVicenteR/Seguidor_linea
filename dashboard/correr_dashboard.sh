#!/usr/bin/env bash
set -euo pipefail

RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PUERTO="${PUERTO:-8092}"
SIMULAR=1
SERVIR=1

uso() {
  echo "Uso: dashboard/correr_dashboard.sh [--sin-simular] [--sin-servidor] [--puerto N]"
  echo "  --sin-simular   usa los resultados que ya existen (no compila ni simula)"
  echo "  --sin-servidor  solo regenera los datos y abre el archivo directamente"
  echo "  --puerto N      puerto del servidor local (por defecto 8092)"
}

while [ $# -gt 0 ]; do
  case "$1" in
    --sin-simular) SIMULAR=0 ;;
    --sin-servidor) SERVIR=0 ;;
    --puerto) PUERTO="$2"; shift ;;
    -h|--help) uso; exit 0 ;;
    *) echo "Opción desconocida: $1"; uso; exit 1 ;;
  esac
  shift
done

cd "$RAIZ"

if [ "$SIMULAR" -eq 1 ]; then
  echo "==> Compilando el simulador"
  cmake -S . -B build >/dev/null
  cmake --build build -j
  echo "==> Simulando el firmware con red neuronal"
  ./build/simulacion/atlas_sim --firmware --out resultados_simulacion/firmware | grep -E "Aprobados" || true
  echo "==> Simulando el firmware sin red neuronal"
  ./build/simulacion/atlas_sim --firmware --sin-red --out resultados_simulacion/firmware_sin_red | grep -E "Aprobados" || true
fi

echo "==> Generando los datos del dashboard"
python3 dashboard/generar_datos.py

abrir() {
  if command -v open >/dev/null 2>&1; then open "$1"
  elif command -v xdg-open >/dev/null 2>&1; then xdg-open "$1"
  else echo "Abre en tu navegador: $1"
  fi
}

if [ "$SERVIR" -eq 0 ]; then
  abrir "$RAIZ/dashboard/index.html"
  exit 0
fi

if lsof -iTCP:"$PUERTO" -sTCP:LISTEN >/dev/null 2>&1; then
  echo "El puerto $PUERTO está ocupado. Usa: dashboard/correr_dashboard.sh --puerto 8093"
  exit 1
fi

URL="http://127.0.0.1:$PUERTO/dashboard/index.html"
echo "==> Dashboard:        $URL"
echo "==> Visor animado:    http://127.0.0.1:$PUERTO/resultados_simulacion/firmware/index.html"
echo "    Ctrl+C para detener el servidor."
python3 -m http.server "$PUERTO" --bind 127.0.0.1 >/dev/null 2>&1 &
SERVIDOR=$!
trap 'kill "$SERVIDOR" 2>/dev/null; echo; echo "Servidor detenido."' EXIT INT TERM
sleep 1
abrir "$URL"
wait "$SERVIDOR"
