# Recomendaciones de hardware contra el zigzag (2026-10-17)

## Por qué hardware

Cuatro pruebas en el simulador llegan a la misma conclusión: lo que limita
al ATLAS es el **retraso**, no el ruido.

| Prueba | Resultado |
|---|---|
| Barrido de Kp/Kd (32,406 competencias) | No baja el zigzag sin pasar del +3 % de tiempo |
| Red jerárquica (% de Kp/Kd por situación) | No superó a la referencia |
| Red de la derivada (77 % menos error de estimación) | Más zigzag (0.91 → 1.23/m) |
| D combinado (15–100 % de la red) | Cada punto de mezcla empeora |
| **Predictor de retraso** (activo) | **Zigzag −5 %** (robot "real" −7 %) |

Lo único que ayudó fue compensar el retraso. La siguiente mejora grande
saldría de **reducir el retraso físico** o de **medir el giro directamente**.

Antes de comprar o cambiar nada, conviene medir el robot real con el registro
serial (`pio run -e atlas_log -t upload`, `capture_log.py`,
`atlas_fit_robot`). Así se sabe cuánto retraso tiene de verdad.

## 1. Giroscopio (por ejemplo, MPU-6050 o similar)

**Qué aporta.** Mide la velocidad de giro (yaw rate) directamente, sin
esperar a que la línea se mueva bajo la barra. El término D de los robots de
competencia rápidos suele combinar el error de la barra con la velocidad de
giro. La señal del giroscopio llega al menos un ciclo de control antes que la
derivada del error y casi no tiene el ruido de cuantización de los sensores.

**Restricción del ATLAS Rev 1.4.** Los pines I²C de hardware (A4/SDA y
A5/SCL) **están ocupados**:

| Pin | Uso actual |
|---|---|
| A0–A3 (PC0–PC3) | Selección del multiplexor de los 16 sensores |
| A4 (PC4) | Entrada analógica de la barra de sensores |
| A5 (PC5) | Botón SW1 |
| D0/D1 | UART (libre si no se usa el registro) |
| D2, D8, D13 | LED0, LED1, LED2 |
| D3/D4 | Módulo de arranque READY/GO |
| D5, D6, D9, D10, D12 | Motores (dirección, PWM e inhibición) |
| D11 | Turbina (OC2A) |
| D7 | Botón SW2 |
| A6/A7 | Solo analógicos (no sirven para I²C) |

**Opciones:**
1. **I²C por software en D8/D13**, sacrificando LED1 y LED2. No hace falta
   rediseñar la placa: bastan dos cables y resistencias de pull-up.
2. **Siguiente revisión de la placa:** mover SW1 fuera de A5 y la entrada de
   los sensores a A6 (solo analógica), para liberar A4/A5 y usar el I²C de
   hardware.

**Costo en el firmware.** Leer solo el eje Z del giroscopio son
~0.8–1.2 KB de flash. El firmware normal está al 93.5 % (quedan ~2 KB), así
que cabe, pero justo. Habría que simular primero su efecto (agregar un
sensor de giro al modelo del simulador) antes de soldar.

## 2. Barra de sensores más adelante

**Qué aporta.** Con la barra más lejos del eje de las ruedas
(`sensorBarAhead` = 18 cm en el simulador), el error de la barra crece con el
rumbo del robot antes de que el robot se desplace de lado. Eso da una
anticipación natural, la misma que el PD intenta conseguir con el término D.

**Contras.** Más momento de inercia y peso adelante; en curvas cerradas la
barra se sale antes de la línea; el soporte puede vibrar (ruido).

**Siguiente paso.** Simular 20 y 22 cm (parámetro `sensorBarAhead` de
`RobotParams`) con el modelo ajustado al robot real antes de cambiar el
soporte.

## 3. Turbina en curvas

**Situación actual.** La turbina va a un PWM fijo por perfil (seguro 190,
rápido 235) con rampa de arranque (`runRampMs`). No sube en las curvas.

**Qué aporta.** Más succión significa más agarre y menos deslizamiento, y el
deslizamiento es retraso extra que el modelo "real" representa con
`yawTimeConstant`. Subir la turbina antes de las curvas del mapa podría bajar
el zigzag de salida de curva.

**Contras.** La turbina tarda en responder (`impellerTimeConstant` = 120 ms
en el simulador), así que habría que subirla por adelantado con el mapa.
Además gasta más batería, y más succión pide más PWM mínimo en los motores
(`suctionDeadbandPwm`).

**Siguiente paso.** Ya se puede probar en el simulador sin hardware nuevo:
PWM de la turbina según la curvatura del mapa más adelante. Solo hace falta
la aprobación para cambiar esa lógica.

## Orden sugerido

1. **Medir** el robot real con `atlas_log` y ajustar el simulador.
2. **Turbina en curvas** (solo software; se valida en el simulador).
3. **Barra más adelante** (simular antes de cambiar el soporte).
4. **Giroscopio** por I²C de software en D8/D13 (simular antes de soldar).
