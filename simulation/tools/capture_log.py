#!/usr/bin/env python3
"""capture_log.py - Guarda el registro serial del firmware atlas_log.

Uso: python3 simulation/tools/capture_log.py PUERTO salida.bin
     (p. ej. /dev/cu.usbserial-1410; termina con Ctrl+C)

Escribe los bytes tal cual llegan (tramas de AvrRaceLogger.h) y muestra
cuántas tramas válidas van. Después: atlas_fit_robot salida.bin
"""
import sys

import serial  # pyserial (incluido en el entorno de PlatformIO)


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 1
    port, path = sys.argv[1], sys.argv[2]
    frames = 0
    with serial.Serial(port, 115200, timeout=0.2) as link, open(path, "wb") as out:
        print(f"Capturando {port} -> {path} (Ctrl+C para terminar)")
        try:
            while True:
                chunk = link.read(512)
                if not chunk:
                    continue
                out.write(chunk)
                frames += chunk.count(b"\xa5\x5a")
                print(f"\r  ~{frames} tramas", end="", flush=True)
        except KeyboardInterrupt:
            print(f"\nListo: ~{frames} tramas en {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
