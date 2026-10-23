# ATLAS Rev. 1.4 — firmware POO + simulador

Refactorización orientada a objetos del firmware ATLAS 1.4.3 (EXOTIC TEAM MX) para el seguidor de línea ATLAS Rev. 1.4 (ATmega328P, 16 sensores QRE1113, drivers BTN9960LV, control PD), con un banco de simulación que corre **el mismo código de control** sobre 11 pistas largas y difíciles a alta velocidad.

> Criterio de aprobación: cada pista debe completarse **en menos de 40 s** a PWM 160, 190 y 220, sin salirse y deteniéndose correctamente al quitar GO.
>
> Modo único: **línea negra sobre fondo blanco**.

## Uso en competencia (3 intentos)

1. **Encender:** conecta el impeller a la salida 2S y luego la batería. El LED2 parpadea al iniciar (mantén SW1/SW2 en ese momento solo si quieres el modo diagnóstico).
2. **Calibrar** (solo la primera vez en una pista nueva): coloca el robot sobre la pista, presiona SW1 o SW2 (parpadean LED0 y LED1), pasa la barra **lentamente** de lado a lado sobre la línea negra y presiona cualquier botón para terminar.
   - LED0 + LED1 **fijos**: calibrado (si LED2 da 3 destellos lentos antes, hay 1–2 sensores dañados y se compensan).
   - LED2 parpadeando rápido: calibración incompleta, repítela.
   - La calibración **se guarda en la EEPROM**: al volver a encender, LED2 da 2 destellos y el robot queda listo sin recalibrar.
3. **Intento 1 — reconocimiento:** elige una velocidad baja y segura con SW1 (−10) / SW2 (+10); **la velocidad se guarda** y no hay que volver a ajustarla al encender. READY y GO: el robot corre con el perfil **Seguro** y aprende la pista; al quitar GO guarda el mapa (también en EEPROM). **Reconocer lento no hace más lentos los intentos 2 y 3**: su velocidad la decide el mapa. En simulación (pista de práctica, robot "real"), reconocer a PWM 60 tarda 22 s y los intentos con mapa siguen en ~12 s; recomendado 60–80 si a 100 se sale.
4. **Intentos 2 y 3 — rápidos:** ahora LED0 y LED1 **parpadean alternados** (hay mapa). READY y GO: el robot cambia solo al perfil **Rápido**, acelera en las rectas y frena antes de cada curva que ya conoce. Se re-sincroniza en cada hueco de la línea.
5. **Pista nueva:** SW1 + SW2 juntos recalibran y **borran el mapa**.

**Si un intento falla** (el robot pasa más de 1.5 s sin encontrar la línea, es decir, se salió):

| Falla en… | Qué hace el robot al quitar GO | Siguiente intento |
|---|---|---|
| Intento 1 (reconocimiento) | Descarta el mapa; LED2 da 4 destellos rápidos y LED0 + LED1 quedan **fijos** | Vuelve a ser de reconocimiento (considera bajar a PWM 90 con SW1) |
| Intento 2 (con mapa) | Conserva el mapa, marca la zona donde se salió y baja 10 % la velocidad de las curvas | Frena antes en esa zona (ver abajo) |

**Ver dónde se salió y corrección automática:**

- Cuando un intento con mapa se sale, el robot anota el **tramo** donde perdió la línea y, sin que hagas nada, marca esa zona (≈1 m antes) como curva más cerrada y hace un 10 % más lentas todas las curvas (las rectas no cambian). El siguiente intento frena ahí. Si vuelve a salirse, corrige de nuevo.
- También cuenta como salida quitar GO mientras el robot buscaba la línea, o una vuelta con mapa que no llegó al 95 % del recorrido.
- Para **verlo**: conecta el USB-TTL, entra al modo diagnóstico (mantén SW1 al encender), elige **[4] REGISTRO** y copia el texto desde `ATLAS-REGISTRO` hasta `FIN`. Pégalo en `simulation_output/analizador.html`: dibuja tu pista con el mapa aprendido, los huecos detectados y una ✕ donde se salió cada intento, más la tabla de tiempos. El registro guarda las últimas 8 carreras (SW2 lo borra desde ese menú).

**Cómo reiniciar:**

| Situación | Qué hacer | Qué se conserva |
|---|---|---|
| El robot se salió o hizo algo raro durante la carrera | Quita **GO**: frena y queda listo. Llévalo al INICIO y da READY + GO | Calibración y mapa (el mapa se descarta solo si el reconocimiento falló) |
| No responde a GO / se quedó trabado | Presiona **RESET** (o apágalo y préndelo) | Calibración y mapa: se cargan de la EEPROM (LED2 da 2 destellos) |
| Quieres empezar de cero (otra pista, otra luz, el mapa salió mal) | **SW1 + SW2 juntos** estando listo: borra el mapa y recalibra | Nada: nueva calibración y el siguiente intento es de reconocimiento |

Estrategia con 3 intentos: si el 1 falla, el 2 es reconocimiento y el 3 el rápido; si el 1 sale bien, su tiempo (≈21 s) ya cuenta y el 2 y el 3 buscan mejorarlo.

Coloca el robot en el mismo INICIO en todos los intentos: el mapa se mide desde la salida.

## Pista de práctica (zonas donde se salía el robot real)

`Pista de práctica` reproduce las dos zonas marcadas en la foto: la "casa" (esquina hacia adentro, pico agudo y diagonal) con un gancho en U, y las ondas con una U profunda; curvas de 9–10 cm de radio y tramos punteados. El robot se salía porque al buscar la línea la barra la cruzaba rápido y (1) el filtro de saltos de los cruces descartaba esas lecturas y (2) el derivativo daba un volantazo al recuperarla. Ahora, durante ~30 ms tras recuperar la línea no se filtran saltos y el PD se reinicia.

Con turbina, 10 variantes por modelo de robot: 0 fallas en 90 intentos; reconocimiento 13.8–15.4 s y vueltas con mapa 8.6 s (nominal) a 12.9 s (robot real). En la pista de competencia mejoró también: 13.2 s con mapa (robot real 17.7 s).

**Turbina:** **PROGRAM / READY** enciende la turbina con una rampa de 1 s, con las ruedas quietas. **START / GO** inicia la carrera sin apagar ni reiniciar la turbina. GO directo también funciona, con una rampa de 0.25 s. Quitar READY antes de correr apaga la turbina; quitar GO durante la carrera la apaga inmediatamente. Si READY sigue activo al volver al estado listo, vuelve a iniciar el giro previo.

Las velocidades de turbina de las tres rondas (PWM 235 en cada una) se ajustan en `lib/AtlasCore/src/rondas/PrimeraRonda.h`, `SegundaRonda.h` y `TerceraRonda.h`. Las rampas comunes y `spinWithReady` están en `lib/AtlasCore/src/parametros/ImpellerConfig.h`. Conéctala a la salida 2S de la batería. Para probarla sin correr: modo diagnóstico, opción [1] IMPELLER.

**Motores:** la velocidad inicial del firmware y el impulso de los primeros 0.2 s quedan en PWM 60, el mínimo configurado en el proyecto. No se ha confirmado ese valor con una especificación del fabricante. La compensación de zona muerta (`control.minMotorPwm = 40`) sigue aplicándose a la salida de las ruedas. Si ya hay una velocidad guardada en EEPROM, esta tiene prioridad para el crucero: SW1 permite bajarla hasta 60.

En simulación, con la succión frenando los motores: pista de práctica y de competencia con 0 fallas en los intentos con mapa; reconocimiento a PWM 60 de la pista de práctica con 1 falla en 30 (robot "real").

## Aprendizaje de pista (resultados en simulación)

Sin encoders: la distancia se estima integrando el PWM en el tiempo y se corrige en cada hueco de la línea. El mapa (curvatura promedio cada ~10 cm, huecos y "confianza") se guarda en EEPROM.

Pista de competencia, 10 variantes de ruido por modelo de robot:

| Robot simulado | Intento 1 (reconoce, PWM 100) | Intentos 2 y 3 (mapa) | Fallas |
|---|---|---|---|
| Nominal | 21.4 s | **14.1 s** (−34 %) | 0/30 |
| Moderado (más lento) | 21.5 s | **14.5 s** (−33 %) | 0/30 |
| Real (lento + inercia + retraso) | 23.5 s | **20.8 s** (−12 %) | 0/30 |

**Sin turbina** (como en las pruebas actuales) también funciona: en la pista de competencia, 0 fallas en 90 intentos y 96 reconocimientos; los intentos con mapa quedan en 14.5 s (nominal) y 20.4 s (robot real), casi igual que con turbina, porque el planificador y la confianza se adaptan solos al menor agarre.

El robot mide en el reconocimiento cuánto tiempo pasó buscando la línea; si su dinámica es "difícil" (robot real), las vueltas guiadas reducen la velocidad solas (`easyLostPerMille`, `slowdownPerLostPerMille`). En el reconocimiento a PWM 100, el robot "real" falló 1 de 32 veces; los demás 0.

## Red neuronal (rama `neouronal`)

**Interruptor:** `kUseNeuralNetwork` en `lib/AtlasCore/src/parametros/FirmwareParameters.h` (`true` = red activa; `false` = control clásico de antes). En el simulador: `atlas_sim --sin-red` para comparar.

### Fase 1: posición fina de la línea
- **Qué hace:** una red pequeña (7 entradas → 8 neuronas ReLU → 1 salida, 76 pesos y sesgos en la flash) lee los 7 sensores alrededor de la línea **sin el umbral del 50 %**. Así también ve los sensores apenas cubiertos, que el promedio ponderado descarta, y calcula la posición con más precisión. Solo reemplaza la posición; los cruces, picos, huecos y bordes siguen con la lógica de siempre. En los extremos de la barra y en grupos anchos usa el promedio ponderado, y su resultado nunca se aleja más de ±300 (±2.7 mm) de él.
- **En el AVR:** aritmética entera (pesos int8/int16, `MULS` nativo), ~150 µs por lectura; firmware al 92 % de flash y 77 % de RAM.
- **Entrenamiento:** `atlas_train_nn posicion` corre el firmware en todas las pistas con 3 modelos de robot, 3 velocidades y sensores variados (tamaño del punto de lectura, ganancia, contraste, ruido, luz). La etiqueta es el cruce verdadero de la línea con la barra, que el simulador conoce. Luego cuantiza a enteros y escribe `lib/AtlasCore/src/NeuralPositionWeights.h`.
- **Precisión (corridas de validación no usadas al entrenar):** error de posición de **1.3 mm → 0.53 mm (−60 %)**; la versión en enteros es igual de precisa que la de `double`.

| Todas las pistas (3 modelos × 10 semillas × 3 intentos, turbina 200) | Fallos |
|---|---|
| Sin red | 10 de 990 (vueltas con mapa en Gran Premio) |
| **Con red** | **0 de 990** |
| Con red y lazo más lento (costo de la red + margen) | 0 de 990 |

En tus pistas (práctica y competencia, configuración de `main.cpp`): 0 fallos con y sin red; con red, entre 0.1 y 0.5 s más rápido y error p95 igual o menor.

### Fase 2: control (ajuste de ganancias en marcha)
- **Qué hace:** una segunda red (6 entradas → 8 ReLU → 2 salidas) decide en cada lectura qué **porcentaje de kp y de kd** usar, entre **70 % y 130 %**. Sus entradas son el error, cuánto cambia, si se aleja o se acerca a la línea, la velocidad, la curva que viene según el mapa y la auto-amortiguación. **kp y kd no cambian** (siguen en `Config.h` / `Profiles.cpp`); la red solo los multiplica dentro de ese rango.
- **Entrenamiento:** `atlas_train_nn control` usa evolución (estrategia evolutiva con muestreo antitético). Cada candidato corre los 3 intentos en las pistas de competencia, práctica, Gran Premio y Hexágono, con 3 modelos de robot y la configuración de `main.cpp`. Premia el tiempo, el error p95 y el zigzag, y castiga cada salida de pista. Escribe `lib/AtlasCore/src/NeuralGainWeights.h`.
- **Validación (99 competencias con semillas nuevas, todas las pistas):** zigzag 1.60 → **1.32/m (−17 %)**, error p95 38 → 37 mm, tiempo medio 11.87 → 11.72 s, 0 salidas.

| Robot "real", con las dos redes | Sin red | Con redes |
|---|---|---|
| Competencia, vueltas 2-3: tiempo / p95 / zigzag | 16.3 s / 60 mm / 3.9 | **15.8 s / 52 mm / 3.2** |
| Práctica, vueltas 2-3: tiempo / p95 / zigzag | 11.4 s / 74 mm / 2.7 | 11.6 s / **61 mm** / 2.9 |
| Todas las pistas (990 intentos) | 10 salidas | 1 salida |

**Ojo:** las dos redes se entrenaron en el simulador. En el robot real conviene probar primero con `kUseNeuralNetwork = true` y, si se porta peor, regresar a `false`.

Firmware con las dos redes: flash al 95.5 % y RAM al 79 %.

Al probar la red aparecieron dos defectos de la lógica de cruces, que también afectaban sin red, y quedaron corregidos:
- **Línea en diagonal en curvas cerradas:** ocupa 4 sensores y se confundía con un cruce, así que el robot seguía de frente. Ahora, sin otra línea cerca, 4 sensores es una línea normal.
- **Después de un cruce:** si la línea quedaba lejos de la posición retenida, el robot seguía "congelado" hasta ~0.3 s. Ahora acepta la línea tras ~15 ms de verla sola y limpia.

## Casos extremos

`atlas_extremes` compara el firmware original del fabricante con el nuevo frente a imperfecciones reales (pista de competencia, Gran Premio, laberinto y chicanas; PWM 100, 160 y 220):

| Caso | Fallas original → nuevo |
|---|---|
| Nominal | 10/24 → 0/24 |
| Motores lentos / retraso de 8 ms | 4/24 → 0, 7/24 → 0 |
| Inercia de giro alta / robot real | 11/24 → 1, 10/24 → 1 |
| Zona muerta, motor débil, batería baja/3S, llantas sucias | 6–12/24 → 0–2 |
| Línea gastada, luz ambiental, sensores altos | 10/24 → 0 |
| Sin succión | 8/24 → 4/24 |
| Sensor dañado / sensor pegado | 11/24 → 6, 12/24 → 10 |
| **Total** | **150/384 → 27/384** |

Soluciones en el firmware: **auto-amortiguación** (`OscillationGuard`: baja Kp y Kd si detecta zigzag), **gobernador de velocidad** (`SpeedGovernor`: baja la velocidad al perder la línea o zigzaguear), **perfil Seguro robusto** (optimizado contra robots nominal, lento, con inercia y con retraso), **frenado anticipado en esquinas** y **tolerancia a 1–2 sensores dañados** (se reemplazan por el promedio de sus vecinos). Lo que sigue fallando: sensores dañados cerca del centro de la barra y correr sin succión.

## Estructura

```
platformio.ini            Firmware (botón "Upload" de PlatformIO, USBasp por ICSP)
src/main.cpp              Parámetros de usuario y cableado de objetos
lib/AtlasCore/src/        Lógica independiente del hardware (robot + simulador)
  rondas/                 Velocidades y selección de primera, segunda y tercera ronda
    PrimeraRonda.h        Crucero PWM 60, turbina 235 y perfil Seguro
    SegundaRonda.h        Máximo motor 255, curva 255 y turbina 235
    TerceraRonda.h        Máximo motor 255, curva 255 y turbina 235 (independientes)
    RoundControl.h        Selección de ronda y modo según mapa disponible
    RoundSequence.cpp     Guardado/corrección del mapa y registro de carreras
  turbina/                Rampas de preparación/carrera y coordinación con READY
  arrancador/             PROGRAM/READY, START/GO, arranque, frenado y paro
  parametros/             Config.h, Profiles.h, FirmwareConfig.h y parámetros comunes
  Hal.h                   Interfaces: IClock, ILineSensorBus, IMotor, IDrivetrain, IOperatorInput, IIndicators
  LineSensorArray         Lectura, calibración y posición de la línea (con seguimiento en cruces)
  PdController            Control proporcional-derivativo saturado
  LineFollower            Carrera: rampas, PD, recuperación de línea, huecos, frenado, modos de mapa
  OscillationGuard        Auto-amortiguación (reduce ganancias si hay zigzag)
  SpeedGovernor           Baja la velocidad al perder la línea, la recupera al ir limpio
  TrackMap, SpeedPlanner  Mapa aprendido y velocidad por tramo con anticipación
  CalibrationStore, TrackMapStore  Calibración y mapa en EEPROM
  RunLog, LogExporter     Registro de las últimas carreras (dónde se salió) para el analizador
  AtlasApp                Máquina de estados del manual (color → calibración → velocidad → READY → GO → fin)
  StatusLeds, Ramp        Utilidades
lib/AtlasAvr/             Implementación AVR (registros, timers, ADC, EEPROM, diagnóstico serial)
CMakeLists.txt            Simulador, pruebas y targets firmware/upload
simulation/sim/           Física, pistas, hardware simulado, operador virtual, árbitro, reporte
simulation/tests/         Pruebas unitarias y de integración
simulation/tools/         atlas_tune (perfiles), atlas_tune_learning (planificador), atlas_extremes
simulation/viewer/        Plantilla del visor HTML animado
```

Para cambiar velocidades, edita el archivo de la ronda correspondiente en
`rondas/`. La primera usa crucero PWM 60 (ajustable con SW1/SW2 y guardado en
EEPROM). La segunda y la tercera usan el mapa: sus valores de motor son
**límites**, y el planificador reduce la velocidad según curvas, confianza y
correcciones aprendidas. Las tres mantienen un impulso inicial PWM 60.

Sin mapa válido se repite reconocimiento. Con mapa se usa la segunda ronda;
después del primer intento guiado se usa la tercera, también en intentos
posteriores. Recalibrar reinicia esta secuencia y borra el mapa. Al apagar no se
guarda el número de ronda: si se carga un mapa válido, se retoma la segunda.
Si una carrera guiada falla, se conserva la corrección automática del mapa.

Principios aplicados: inversión de dependencias (el núcleo solo conoce interfaces), responsabilidad única por clase, configuración inyectada, sin estado global en el núcleo, funciones cortas con nombres descriptivos.

## Firmware (PlatformIO)

Abre la carpeta en VS Code con la extensión **PlatformIO IDE**: aparecen los botones *Build* y *Upload* (entorno `atlas`, programador USBasp como indica el manual). Desde terminal:

```bash
pio run -e atlas -t upload
```

Para programar por la terminal UART con un convertidor USB-TTL usa el entorno `atlas_uart`. Consumo actual: 1514 B de RAM (74 %) y 27.0 KB de Flash (88 %).

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
| **Pista de práctica** | 8.6 m | Zonas reales donde se salía: casa con pico agudo, gancho en U, ondas con U profunda (r 9–10 cm), punteados |
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
| Tiempo por vuelta | 18.7 s | 14.5 s | 11.9 s | 10.4 s | 9.5 s | 9.1 s |

Pasa saliendo desde el INICIO y saliendo desde un hueco, y no falla en 40 corridas con distintas semillas de ruido (PWM 160–250).

### Resultados actuales

96/105 escenarios aprobados; ningún obligatorio falla. No pasan: el Triángulo de rizos (lazos más cerrados que el límite geométrico; el perfil Seguro prioriza la pista de competencia con robots lentos o con inercia) y la prueba intencional sin succión. Todas las pistas que se completan lo hacen **en menos de 40 s**; el Gran Premio (35.5 m) tarda 17.3 s a PWM 160, 13.1 s a 220 y 12.0 s a 250.

- En un barrido de 480 corridas (12 pistas × PWM 160–250 × 4 semillas de ruido) solo fallan 2 corridas del triángulo de rizos a PWM 240; el resto, incluida la pista de competencia, 0.
- **Gran Premio sin impeller: falla a propósito** (muestra el efecto de la succión).

## Cambios respecto al firmware original

1. **Menos oscilación** — Kd de 0.35 a 1.2 y filtro exponencial del derivativo (`derivativeFilterShift = 2`): el zigzag baja 55 % sin perder velocidad.
2. **Bug corregido** — en `RunControl.ino` el método fuera de línea calculaba `offlineElapsedTime = currentTime - offlineElapsedTime` (se restaba a sí mismo). Debía usar `offlineStartTime`; ahora el freno de la rueda interior dura los 35 ms previstos.
3. **Cruces oblicuos y rizos** — con varias líneas bajo la barra el promedio ponderado original se desviaba hacia la línea transversal (en simulación el robot tomaba el cruce en la estrella, el ocho y los rizos). Ahora se usa el grupo de sensores contiguos más cercano a la posición anterior y, si el grupo es más ancho que una línea o la posición salta más de un sensor por ciclo, se mantiene el rumbo. Se desactiva con `config.sensors.trackNearestCluster = false`.
4. **Alta velocidad** — `maxForwardPwm` pasa de 200 a 255. Con 200, PWM 220 y 250 daban la misma velocidad (~2.5 m/s); con 255 el robot llega a 3.1 m/s.
5. **Frenado en curva** — la velocidad base baja hasta un 40 % en proporción al error (`curveSlowdownPercent`; 0 = original). En el barrido de 440 corridas redujo las fallas de 6 a 3 con un costo de tiempo menor a 1 %.
6. **Solo línea negra sobre fondo blanco** — cualquier botón inicia la calibración (`config.sensors.lineColor`).
7. **Indicación de calibración** — al terminar se verifica que los 16 sensores hayan visto línea y fondo (`minCalibrationSpan`, `requiredCalibratedSensors`): LED0 + LED1 fijos si es válida, LED2 rápido y repetir si no.
8. **Huecos en la línea** — si la línea desaparece justo después de un tramo recto y centrado (≥ 25 ms), el robot sigue recto hasta 300 ms antes de buscarla girando (`gapBridgeMs`, `gapStraightMs`). También permite arrancar sobre un hueco.
   Entre huecos consecutivos admite una pequeña desviación de la línea (`gapChainCenterTolerance`) sin exigir otra recta de 25 ms. Una rama, un borde o una desviación mayor cancela esa continuidad.
9. **Varias carreras por calibración** — quitar GO regresa al estado listo en lugar de quedarse en espera hasta RESET; SW1 + SW2 juntos recalibran.
10. **Picos agudos** — se recuerda de qué lado apareció otra rama de línea; al terminarse la línea en el pico el robot gira hacia esa rama.
11. El impeller sigue su rampa aunque el robot esté fuera de la línea (antes conservaba el último valor).
12. Sensores sin calibrar (máx ≤ mín) nunca reportan línea, en vez de producir un umbral inválido.

## Precisión y robustez (rama `feature/precision-alta-velocidad`)

Validación con 1 536 corridas que el optimizador no vio (12 pistas × PWM 100–250 × 8 semillas de ruido):

| | Fallas | Error p95 pistas suaves* | Error p95 pista de competencia |
|---|---|---|---|
| Antes (Kp 0.05, Kd 1.2) | 95 | 4–6 mm | 31 mm |
| **Ahora** | **3** | **3.5–4.9 mm** | 30 mm |

\* óvalo, chicanas, estrella, ocho y flor.

- **Cruces a velocidad baja:** cuando otra línea se une a la que se sigue, el grupo de sensores se ensancha o refleja más; antes la posición se desviaba medio sensor y el derivativo daba un tirón hacia la rama equivocada (95 fallas a PWM 100–130). Ahora se detecta por ancho e intensidad y se mantiene el rumbo; el tiempo máximo manteniéndolo se escala con la velocidad.
- **Parámetros** elegidos con `atlas_tune` (búsqueda aleatoria + refinamiento sobre PWM 100–250): Kp 0.09, Kd 1.15, filtro 1/4, frenado en curva 8 %, rueda interior a −210 durante 45 ms al buscar la línea.
- **Frenado anticipado en esquinas** implementado (`CornerConfig`) pero desactivado: el optimizador no encontró mejora.
- **Las 3 fallas** son en rizos con radio menor que el límite geométrico del robot (la barra está 18 cm delante del eje, así que no puede seguir de forma estable curvas de menos de ~18 cm): 2 en el Trébol (lazos de 11 cm, ahora prueba no obligatoria) y 1 en el triángulo de rizos, todas a PWM 100–110.
- **Límite físico:** en esquinas vivas y picos la barra siempre se sale unos centímetros, por eso el p95 de la pista de competencia queda en ~30 mm; en tramos suaves la precisión es de 3–5 mm.

Para volver a optimizar con otros pesos o pistas: `./build/simulation/atlas_tune [aleatorias] [refinamientos] [semilla]`.

## Oscilación

El simulador mide la oscilación como **cruces de un lado a otro de la línea por metro** y el **esfuerzo de dirección** (RMS de la diferencia entre ruedas). Ajuste elegido tras barrer Kp, Kd y el filtro del derivativo:

| | Kp | Kd | Filtro | Cruces/m | Esfuerzo | Error RMS |
|---|---|---|---|---|---|---|
| Original | 0.05 | 0.35 | — | 1.79 | 46 PWM | 13.2 mm |
| Rama de oscilación | 0.05 | 1.2 | 1/4 | 0.81 | 44 PWM | 13.2 mm |

Si en el robot real todavía oscila, en `lib/AtlasCore/src/parametros/Config.h` y `Profiles.h`: sube `kd` de 0.2 en 0.2, o baja `kp` de 0.005 en 0.005; si vibra rápido (zumbido), sube `derivativeFilterShift` a 3. Cambia un parámetro a la vez y vuelve a cargar.

## Limitaciones conocidas

- El modelo físico es simplificado (sin inercia de guiñada ni deriva lateral detallada); sirve para comparar algoritmos y configuraciones, no reemplaza las pruebas en pista.
- En el laberinto, a velocidades bajas (PWM 100–130) la esquina aguda de 115° puede hacer que la recuperación gire hacia el lado equivocado; a partir de PWM 160 pasa.
- El manual recomienda PWM ≤ 160, impeller ≤ 215 y no exceder 8 s de prueba con 3S. Las velocidades superiores son pruebas de simulación: súbelas en el robot real de forma gradual.

### Validar los ajustes de despliegue

`atlas_sim --firmware --out DIR` ejecuta tres intentos en cada pista con los
ajustes compartidos con `src/main.cpp` en `FirmwareConfig.h`: reconocimiento
PWM 60, turbina 235; perfil Rápido con mapa y turbina 235. Añadir `--sin-red`
permite comparar ambas redes apagadas con las mismas semillas. `--track 0`
limita la ejecución a la pista de competencia. Los dos modos de esa pista
forman parte de CTest. Los rizos menores que la distancia barra-eje siguen
siendo exploratorios, no obligatorios.

El banco general conserva su barrido histórico (130/160/190/220/250), turbina
200 y aprendizaje a 100 con turbina 215. Ese barrido no representa por sí solo
los ajustes de despliegue. Las simulaciones no sustituyen pruebas en hardware;
una velocidad guardada en EEPROM también puede cambiar el arranque real.

Con el arranque a PWM 60, el reconocimiento en la pista de competencia supera
el límite de 40 s de CTest; los intentos con mapa completan la pista en unos
11 s. Se conserva el mínimo solicitado sin modificar el criterio de aprobación.
