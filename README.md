# ATLAS Rev. 1.4 — firmware POO + simulador

Refactorización orientada a objetos del firmware ATLAS 1.4.3 (EXOTIC TEAM MX) para el seguidor de línea ATLAS Rev. 1.4 (ATmega328P, 16 sensores QRE1113, drivers BTN9960LV, control PD), con un banco de simulación que corre **el mismo código de control** sobre 11 pistas largas y difíciles a alta velocidad.

> Criterio de aprobación: cada pista debe completarse **en menos de 40 s** a PWM 160, 190 y 220, sin salirse y deteniéndose correctamente al quitar GO.

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
simulation/tests/         31 pruebas unitarias y de integración
simulation/viewer/        Plantilla del visor HTML animado
```

Principios aplicados: inversión de dependencias (el núcleo solo conoce interfaces), responsabilidad única por clase, configuración inyectada, sin estado global en el núcleo, funciones cortas con nombres descriptivos.

## Firmware (PlatformIO)

Abre la carpeta en VS Code con la extensión **PlatformIO IDE**: aparecen los botones *Build* y *Upload* (entorno `atlas`, programador USBasp como indica el manual). Desde terminal:

```bash
pio run -e atlas -t upload
```

Para programar por la terminal UART con un convertidor USB-TTL usa el entorno `atlas_uart`. Consumo actual: 624 B de RAM (30 %) y 13.5 KB de Flash (44 %).

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

`simulate` corre los 48 escenarios en paralelo y genera `simulation_output/index.html`: un visor con la animación del robot sobre cada pista (reproducir, pausa, velocidad 0.25×–8×, línea de tiempo, cámara que sigue al robot), la gráfica de velocidad con las zonas donde se perdió la línea, métricas y una matriz de resultados. También genera `resumen.csv`. Si CMake encuentra PlatformIO, existen también los targets `firmware` y `upload`.

### Qué se simula

- **Firmware real**: `AtlasApp`, `LineFollower` y `LineSensorArray` sin cambios, sobre un reloj virtual donde cada lectura de ADC consume 20 µs (el lazo de 750 µs tiene la misma temporización que en el ATmega328P).
- **Operador virtual** que sigue el manual: elige el color de línea, barre la barra sobre la línea para calibrar, ajusta la velocidad con SW1/SW2, activa READY (rampa del impeller) y GO, y quita GO al terminar.
- **Robot**: tracción diferencial con motores de primer orden (3.2 m/s a PWM 255), barra a 18 cm del eje, sensores cada 9 mm con ruido, dispersión de ganancia y área de lectura finita; adherencia limitada por μ·(g + succión/m), así que sin impeller derrapa.
- **Árbitro**: avance sobre la pista (sin confundirse en los cruces), vueltas, error lateral, salida de pista (>12 cm durante 0.4 s), atascos y límite de 40 s.

### Pistas (combinan figuras geométricas)

| Pista | Largo | Qué la hace difícil |
|---|---|---|
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

### Resultados actuales

47/48 escenarios aprobados; **todos los obligatorios pasan en menos de 40 s**. El más lento es el Gran Premio (35.5 m): 17.2 s a PWM 160, 13.1 s a 220 y 12.0 s a 250. La única falla es intencional: el Gran Premio sin impeller a PWM 190 derrapa y se sale (demuestra el efecto de la succión).

## Cambios respecto al firmware original

1. **Bug corregido** — en `RunControl.ino` el método fuera de línea calculaba `offlineElapsedTime = currentTime - offlineElapsedTime` (se restaba a sí mismo). Debía usar `offlineStartTime`; ahora el freno de la rueda interior dura los 35 ms previstos.
2. **Cruces oblicuos y rizos** — con varias líneas bajo la barra el promedio ponderado original se desviaba hacia la línea transversal (en simulación el robot tomaba el cruce en la estrella, el ocho y los rizos). Ahora se usa el grupo de sensores contiguos más cercano a la posición anterior y, si el grupo es más ancho que una línea o la posición salta más de un sensor por ciclo, se mantiene el rumbo. Se desactiva con `config.sensors.trackNearestCluster = false`.
3. **Alta velocidad** — `maxForwardPwm` pasa de 200 a 255. Con 200, PWM 220 y 250 daban la misma velocidad (~2.5 m/s); con 255 el robot llega a 3.1 m/s.
4. El impeller sigue su rampa aunque el robot esté fuera de la línea (antes conservaba el último valor).
5. Sensores sin calibrar (máx ≤ mín) nunca reportan línea, en vez de producir un umbral inválido.

## Limitaciones conocidas

- El modelo físico es simplificado (sin inercia de guiñada ni deriva lateral detallada); sirve para comparar algoritmos y configuraciones, no reemplaza las pruebas en pista.
- En el laberinto, a velocidades bajas (PWM 100–130) la esquina aguda de 115° puede hacer que la recuperación gire hacia el lado equivocado; a partir de PWM 160 pasa.
- El manual recomienda PWM ≤ 160, impeller ≤ 215 y no exceder 8 s de prueba con 3S. Las velocidades superiores son pruebas de simulación: súbelas en el robot real de forma gradual.
