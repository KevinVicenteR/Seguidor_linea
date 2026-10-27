#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

echo "==> Compilando simulador y pruebas"
cmake -S . -B build >/dev/null
cmake --build build -j

echo "==> Pruebas"
ctest --test-dir build --output-on-failure

PIO="$(command -v pio || echo "$HOME/.platformio/penv/bin/pio")"
if [ -x "$PIO" ]; then
  echo "==> Firmware del robot"
  "$PIO" run -e atlas | grep -E "Flash|RAM|SUCCESS|FAILED|rror"
else
  echo "==> PlatformIO no encontrado: se omite la compilación del firmware"
fi

exec dashboard/correr_dashboard.sh "$@"
