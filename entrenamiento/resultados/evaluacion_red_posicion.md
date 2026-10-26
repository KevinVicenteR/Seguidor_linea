# Red de posición: entrenamiento y evaluación (rama sin turbina)

Entrenada con `./build/simulacion/atlas_train_nn posicion 50000` (50 000 lecturas
simuladas sin turbina; registro completo en `entrenamiento_posicion.log`).

Error de posición en validación (RMS, milésimas de sensor; 1000 = 9 mm):

| Método | RMS | Máximo |
|---|---|---|
| Promedio ponderado (clásico) | 145 | — |
| Red anterior (40 000 lecturas, con turbina) | 59 | 784 |
| **Red nueva (50 000 lecturas, sin turbina)** | **58** | **614** |

Lazo cerrado: 11 pistas obligatorias × robots nominal, moderado y real × 8 semillas × 3 intentos
(792 carreras por fila), firmware de la rama sin turbina.

| Agarre de llantas | Fallas con red | Fallas sin red | Vuelta con mapa (con / sin red) |
|---|---|---|---|
| 100 % | **0** | 7 | 14.24 s / 14.23 s |
| 90 % | 5 | 4 | 14.55 s / 14.58 s |
| 80 % | **7** | 9 | 14.93 s / 14.96 s |
