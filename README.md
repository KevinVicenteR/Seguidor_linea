# ATLAS Rev. 1.4 — firmware POO + simulador

Refactorización orientada a objetos del firmware ATLAS 1.4.3 (EXOTIC TEAM MX) para el seguidor de línea ATLAS Rev. 1.4 (ATmega328P, 16 sensores QRE1113, drivers BTN9960LV, control PD), con un banco de simulación que corre **el mismo código de control** sobre 11 pistas largas y difíciles a alta velocidad.

> Criterio de aprobación: cada pista debe completarse **en menos de 40 s** a PWM 160, 190 y 220, sin salirse y deteniéndose correctamente al quitar GO.
>
> Modo único: **línea negra sobre fondo blanco**.

## Encendido y calibración

1. Conecta el impeller a la salida 2S de la batería y luego la batería principal. El LED2 parpadea al iniciar (mantén SW1/SW2 en ese momento solo si quieres el modo diagnóstico).
2. Coloca el robot sobre la pista y presiona **SW1 o SW2**: empieza la calibración (parpadean LED0 y LED1).
3. Pasa la barra **lentamente** de lado a lado para que los 16 sensores vean la línea negra y el fondo blanco; presiona cualquier botón para terminar.
4. Indicación del resultado:
   - **Calibrado:** todos los LEDs parpadean dos veces y luego **LED0 + LED1 quedan encendidos fijos**.
   - **Incompleta:** el **LED2 parpadea rápido**; algún sensor no vio línea y fondo. Presiona un botón y repite la pasada.
5. Ajusta la velocidad (SW2 +10, SW1 −10, base 100), activa READY (LED2 fijo = impeller listo) y luego GO. Quitar GO frena y detiene el robot.

## Estructura

```
platformio.ini            Firmware (botón "Upload" de PlatformIO, USBasp por ICSP)
src/main.cpp              Parámetros de usuario y cableado de objetos
lib/AtlasCore/            Lógica independiente del hardware (robot + simulador)
  Config.h                Configuración agrupada por responsabilidad
  Hal.h                   Interfaces: IClock, ILineSensorBus, IMotor, IDrivetrain, IOperatorInput, IIndicators
  LineSensorArray         Lectura, calibración y posición de la línea (con seguimiento en cruces)
  PdController            Control proporcional-derivativo saturado
  LineFollower            Carrera: rampas, PD, recuperación de línea, frenado
  AtlasApp                Máquina de estados del manual (color → calibración → velocidad → READY → GO → fin)
  StatusLeds, Ramp        Utilidades
lib/AtlasAvr/             Implementación AVR (registros, timers, ADC, diagnóstico serial)
CMakeLists.txt            Simulador, pruebas y targets firmware/upload
simulation/sim/           Física, pistas, hardware simulado, operador virtual, árbitro, reporte
simulation/tests/         40 pruebas unitarias y de integración
simulation/viewer/        Plantilla del visor HTML animado
```

Principios aplicados: inversión de dependencias (el núcleo solo conoce interfaces), responsabilidad única por clase, configuración inyectada, sin estado global en el núcleo, funciones cortas con nombres descriptivos.

## Firmware (PlatformIO)

Abre la carpeta en VS Code con la extensión **PlatformIO IDE**: aparecen los botones *Build* y *Upload* (entorno `atlas`, programador USBasp como indica el manual). Desde terminal:

```bash
pio run -e atlas -t upload
```

Para programar por la terminal UART con un convertidor USB-TTL usa el entorno `atlas_uart`. Consumo actual: 651 B de RAM (32 %) y 14.2 KB de Flash (46 %).

## Simulador y pruebas (CMake)

```bash
cmake -S . -B build && cmake --build build -j
```

```bash
ctest --test-dir build --output-on-failure
```

```bash
cmake --build build --target simulate
```

`simulate` corre los 52 escenarios en paralelo y genera `simulation_output/index.html`: un visor con la animación del robot sobre cada pista (reproducir, pausa, velocidad 0.25×–8×, línea de tiempo, cámara que sigue al robot), la gráfica de velocidad con las zonas donde se perdió la línea, métricas y una matriz de resultados. También genera `resumen.csv`. Si CMake encuentra PlatformIO, existen también los targets `firmware` y `upload`.

### Qué se simula

- **Firmware real**: `AtlasApp`, `LineFollower` y `LineSensorArray` sin cambios, sobre un reloj virtual donde cada lectura de ADC consume 20 µs (el lazo de 750 µs tiene la misma temporización que en el ATmega328P).
- **Operador virtual** que sigue el manual: inicia la calibración, barre la barra sobre la línea para calibrar, ajusta la velocidad con SW1/SW2, activa READY (rampa del impeller) y GO, y quita GO al terminar.
- **Robot**: tracción diferencial con motores de primer orden (3.2 m/s a PWM 255), barra a 18 cm del eje, sensores cada 9 mm con ruido, dispersión de ganancia y área de lectura finita; adherencia limitada por μ·(g + succión/m), así que sin impeller derrapa.
- **Árbitro**: avance sobre la pista (sin confundirse en los cruces), vueltas, error lateral, salida de pista (>12 cm durante 0.4 s), atascos y límite de 40 s.

### Pistas (combinan figuras geométricas)

| Pista | Largo | Qué la hace difícil |
|---|---|---|
| **Pista de competencia** | 25.9 m | Reglamento: trovicel 3.66 × 2.44 m, cinta de 2 cm, dos picos agudos de ~56°, dos huecos de ~9 cm, dos INICIO |
| Óvalo de velocidad | 10.0 m ×2 | Referencia de velocidad máxima |
| Chicanas largas | 19.7 m | Chicanas suaves y cerradas, eses amplias |
| Hexágono de figuras | 17.3 m | Cada lado es una figura: senoide, zigzag, escalones, rizos, semicírculos |
| Estrella de cinco puntas | 18.3 m | Puntas de 36° y cinco cruces |
| Triángulo de rizos | 15.1 m | Tres rizos de 360° con cruce, zigzag fino, senoide |
| Serpentina de horquillas | 33.4 m | 12 horquillas de 180° (radio 15 cm) |
| Laberinto de esquinas vivas | 26.5 m | Esquinas de 90° sin radio, giros en U, ángulos agudos de 115° |
| Ocho con cruce | 13.4 m ×2 | Cruce oblicuo a 53° |
| Flor ondulada | 12.6 m ×2 | Curvatura variable con 14 inflexiones |
| Trébol de lazos | 18.8 m | Cuatro lazos de 11 cm de radio con cruce |
| Gran Premio | 35.5 m | Rizo, chicana, escalón, cruce, horquillas y curvas rápidas |

Las pistas se construyen con `FilletedLoopBuilder` (polígono + radio por vértice + figura por lado) o curvas paramétricas; agregar una nueva es una función en `simulation/sim/TrackLibrary.cpp`.

### Pista de competencia

Reconstruida a escala del esquema del reglamento (`competitionTrack()` en `TrackLibrary.cpp`, precisión aproximada ±5 cm). Es un solo circuito con simetría de 180° que pasa por los dos INICIO en el mismo sentido.

| PWM | 100 | 130 | 160 | 190 | 220 | 250 |
|---|---|---|---|---|---|---|
| Tiempo por vuelta | 18.7 s | 14.5 s | 12.0 s | 10.6 s | 9.6 s | 9.3 s |

Pasa saliendo desde el INICIO y saliendo desde un hueco, y no falla en 40 corridas con distintas semillas de ruido (PWM 160–250).

### Resultados actuales

50/52 escenarios aprobados. Todas las pistas que se completan lo hacen **en menos de 40 s**; el Gran Premio (35.5 m) tarda 17.3 s a PWM 160, 13.1 s a 220 y 12.0 s a 250.

- **Trébol de lazos a PWM 160: falla.** Sus lazos de 11 cm exigen más adherencia de la que da la succión a ~2 m/s; el robot derrapa, recupera la línea por la rama equivocada y se sale. En un barrido de 480 corridas (12 pistas × PWM 160–250 × 4 semillas de ruido) falla en 2 de 40 corridas del trébol y 1 de 40 del triángulo de rizos; el resto de pistas, incluida la de competencia, 0.
- **Gran Premio sin impeller: falla a propósito** (muestra el efecto de la succión).

## Cambios respecto al firmware original

1. **Bug corregido** — en `RunControl.ino` el método fuera de línea calculaba `offlineElapsedTime = currentTime - offlineElapsedTime` (se restaba a sí mismo). Debía usar `offlineStartTime`; ahora el freno de la rueda interior dura los 35 ms previstos.
2. **Cruces oblicuos y rizos** — con varias líneas bajo la barra el promedio ponderado original se desviaba hacia la línea transversal (en simulación el robot tomaba el cruce en la estrella, el ocho y los rizos). Ahora se usa el grupo de sensores contiguos más cercano a la posición anterior y, si el grupo es más ancho que una línea o la posición salta más de un sensor por ciclo, se mantiene el rumbo. Se desactiva con `config.sensors.trackNearestCluster = false`.
3. **Alta velocidad** — `maxForwardPwm` pasa de 200 a 255. Con 200, PWM 220 y 250 daban la misma velocidad (~2.5 m/s); con 255 el robot llega a 3.1 m/s.
4. **Frenado en curva** — la velocidad base baja hasta un 40 % en proporción al error (`curveSlowdownPercent`; 0 = original). En el barrido de 440 corridas redujo las fallas de 6 a 3 con un costo de tiempo menor a 1 %.
5. **Solo línea negra sobre fondo blanco** — cualquier botón inicia la calibración (`config.sensors.lineColor`).
6. **Indicación de calibración** — al terminar se verifica que los 16 sensores hayan visto línea y fondo (`minCalibrationSpan`, `requiredCalibratedSensors`): LED0 + LED1 fijos si es válida, LED2 rápido y repetir si no.
7. **Huecos en la línea** — si la línea desaparece justo después de un tramo recto y centrado (≥ 60 ms), el robot sigue recto hasta 200 ms antes de buscarla girando (`gapBridgeMs`, `gapStraightMs`). También permite arrancar sobre un hueco.
8. **Picos agudos** — se recuerda de qué lado apareció otra rama de línea; al terminarse la línea en el pico el robot gira hacia esa rama.
9. El impeller sigue su rampa aunque el robot esté fuera de la línea (antes conservaba el último valor).
10. Sensores sin calibrar (máx ≤ mín) nunca reportan línea, en vez de producir un umbral inválido.

## Limitaciones conocidas

- El modelo físico es simplificado (sin inercia de guiñada ni deriva lateral detallada); sirve para comparar algoritmos y configuraciones, no reemplaza las pruebas en pista.
- En el laberinto, a velocidades bajas (PWM 100–130) la esquina aguda de 115° puede hacer que la recuperación gire hacia el lado equivocado; a partir de PWM 160 pasa.
- El manual recomienda PWM ≤ 160, impeller ≤ 215 y no exceder 8 s de prueba con 3S. Las velocidades superiores son pruebas de simulación: súbelas en el robot real de forma gradual.
