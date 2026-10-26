# Diagnóstico

Ejecuta los comandos desde la raíz del repositorio.

- `atlas_extremes.cpp`: compara el firmware original y el actual ante imperfecciones del robot y del entorno. Compila con `cmake -S . -B build && cmake --build build -j` y ejecuta `./build/simulacion/atlas_extremes --out resultados_simulacion`. Sin `--out` utiliza esa misma carpeta; genera el reporte y visor HTML.
- `capture_log.py`: captura las tramas binarias del firmware `atlas_log` por serial a 115200 baudios. Carga con `pio run -e atlas_log -t upload` y ejecuta `~/.platformio/penv/bin/python diagnostico/capture_log.py /dev/cu.usbserial-XXXX vuelta.bin` (requiere pyserial, incluido en el entorno PlatformIO). Termina con Ctrl+C. Ajusta el modelo con `./build/simulacion/atlas_fit_robot vuelta.bin --pista 0 --pwm 130`.

## Modo diagnóstico del robot

Mantén SW1 o SW2 al encender. Conecta un USB-TTL y abre el monitor serial a 115200 baudios. SW2 cambia la opción y SW1 confirma; los LED muestran la selección. RESET permite salir de las pruebas continuas.

| Opción | Uso |
|---|---|
| [0] MOTORES | Con las ruedas en el aire, pulsa un botón: prueba cada rueda hacia delante y atrás (PWM 60). |
| [1] IMPELLER | SW2 alterna encendido y apagado con la rampa configurada. En esta rama sin turbina el PWM configurado es 0. |
| [2] SENSORES | SW1 selecciona lectura calibrada (sigue las instrucciones de calibración); SW2, lectura cruda. Mantener un botón muestra valores crudos. |
| [3] INTERFAZ | Muestra SW1, SW2, READY y GO por serial; LED0 indica SW1, LED1 SW2 y LED2 ambos. |
| [4] REGISTRO | Exporta mapa y últimas 8 carreras: copia desde `ATLAS-REGISTRO` hasta `FIN` y pega en `resultados_simulacion/analizador.html`. SW2 borra el registro; SW1 vuelve al menú. |

Genera el analizador con `cmake --build build --target simulate`. El registro de texto de [4] y la captura binaria para ajustar el modelo tienen formatos y usos distintos. Consulta también el [uso en competencia](../README.md#uso-en-competencia-3-intentos).
