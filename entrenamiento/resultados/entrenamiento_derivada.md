# Red de la derivada: registro (2026-10-17)

Estimación (pistas de validación separadas), RMS en unidades de error por ciclo:

```
adelanto 0:
  validación       44.65       10.13         10.14            572
  mejora en validación: 77 %
adelanto 2:
  validación       44.83       10.42         10.44            619
  mejora en validación: 77 %
adelanto 5:
  validación       45.10       11.05         11.09            536
  mejora en validación: 75 %
adelanto 10:
  validación       45.61       11.98         11.99            495
  mejora en validación: 74 %
D clásico / verdadera: 0.95 (correlación 0.49)
```

Lazo cerrado (66 competencias por configuración, adelanto 0):

```
Prueba en lazo cerrado: 66 competencias por configuración (3 intentos cada una)
  D clásico (total)       salidas   0  zigzag±10mm 0.907/m  dirección  40.0  mapa  11.33s (+0.0 %)
  red de la derivada       salidas   0  zigzag±10mm 1.225/m  dirección  40.2  mapa  11.51s (+1.6 %)

Por pista y modelo (clásico -> red):
  Pista de competencia       nominal   salidas  0-> 0  zigzag 0.503->0.567  mapa  10.79-> 10.67s (-1.0 %)
  Pista de competencia       moderado  salidas  0-> 0  zigzag 1.161->1.457  mapa  12.57-> 13.13s (+4.5 %)  <- NO CUMPLE
  Pista de competencia       real      salidas  0-> 0  zigzag 2.392->3.662  mapa  15.77-> 16.57s (+5.1 %)  <- NO CUMPLE
  Pista de práctica         nominal   salidas  0-> 0  zigzag 0.970->1.009  mapa   8.83->  8.48s (-4.0 %)
  Pista de práctica         moderado  salidas  0-> 0  zigzag 1.397->1.629  mapa  10.57-> 10.55s (-0.2 %)
  Pista de práctica         real      salidas  0-> 0  zigzag 1.785->2.328  mapa  11.53-> 11.59s (+0.5 %)
  Óvalo de velocidad        nominal   salidas  0-> 0  zigzag 0.000->0.000  mapa   7.69->  7.73s (+0.5 %)
  Óvalo de velocidad        moderado  salidas  0-> 0  zigzag 0.000->0.000  mapa   7.77->  7.78s (+0.2 %)
  Óvalo de velocidad        real      salidas  0-> 0  zigzag 0.000->0.233  mapa   7.81->  7.64s (-2.1 %)
  Chicanas largas            nominal   salidas  0-> 0  zigzag 0.608->0.608  mapa   7.82->  7.83s (+0.1 %)
  Chicanas largas            moderado  salidas  0-> 0  zigzag 0.608->0.676  mapa   8.07->  8.20s (+1.6 %)
  Chicanas largas            real      salidas  0-> 0  zigzag 1.216->2.601  mapa   8.82->  9.43s (+6.8 %)  <- NO CUMPLE
  Hexágono de figuras       nominal   salidas  0-> 0  zigzag 0.923->0.923  mapa   8.58->  8.38s (-2.3 %)
  Hexágono de figuras       moderado  salidas  0-> 0  zigzag 0.981->1.115  mapa  10.81-> 10.95s (+1.4 %)
  Hexágono de figuras       real      salidas  0-> 0  zigzag 2.231->2.481  mapa  11.55-> 11.83s (+2.4 %)
  Estrella de cinco puntas   nominal   salidas  0-> 0  zigzag 0.000->0.000  mapa   7.11->  7.08s (-0.5 %)
  Estrella de cinco puntas   moderado  salidas  0-> 0  zigzag 0.000->0.000  mapa   7.15->  7.12s (-0.4 %)
  Estrella de cinco puntas   real      salidas  0-> 0  zigzag 1.021->2.171  mapa   7.81->  8.39s (+7.5 %)  <- NO CUMPLE
  Serpentina de horquillas   nominal   salidas  0-> 0  zigzag 0.360->0.360  mapa  13.08-> 13.00s (-0.6 %)
  Serpentina de horquillas   moderado  salidas  0-> 0  zigzag 0.839->0.869  mapa  14.08-> 14.11s (+0.2 %)
  Serpentina de horquillas   real      salidas  0-> 0  zigzag 1.798->2.697  mapa  18.90-> 19.58s (+3.6 %)  <- NO CUMPLE
  Laberinto de esquinas vivas nominal   salidas  0-> 0  zigzag 0.529->0.491  mapa  15.28-> 15.26s (-0.1 %)
  Laberinto de esquinas vivas moderado  salidas  0-> 0  zigzag 1.889->2.318  mapa  15.40-> 15.86s (+3.0 %)
  Laberinto de esquinas vivas real      salidas  0-> 0  zigzag 2.368->4.220  mapa  17.41-> 18.20s (+4.5 %)  <- NO CUMPLE
  Ocho con cruce             nominal   salidas  0-> 0  zigzag 0.112->0.112  mapa  10.17-> 10.10s (-0.6 %)
  Ocho con cruce             moderado  salidas  0-> 0  zigzag 0.112->0.112  mapa  10.13-> 10.05s (-0.7 %)
  Ocho con cruce             real      salidas  0-> 0  zigzag 0.112->0.274  mapa  10.16-> 10.21s (+0.5 %)
  Flor ondulada              nominal   salidas  0-> 0  zigzag 1.071->1.071  mapa  10.50-> 10.62s (+1.2 %)
  Flor ondulada              moderado  salidas  0-> 0  zigzag 1.071->1.071  mapa  10.70-> 10.85s (+1.4 %)
  Flor ondulada              real      salidas  0-> 0  zigzag 1.772->1.812  mapa  11.49-> 11.99s (+4.4 %)  <- NO CUMPLE
  Gran Premio                nominal   salidas  0-> 0  zigzag 0.394->0.394  mapa  13.75-> 13.68s (-0.6 %)
  Gran Premio                moderado  salidas  0-> 0  zigzag 0.553->0.703  mapa  14.30-> 14.47s (+1.2 %)
  Gran Premio                real      salidas  0-> 0  zigzag 1.153->2.475  mapa  17.39-> 18.45s (+6.1 %)  <- NO CUMPLE

No cumple en todas las pistas y modelos.

adelanto 2:
  red de la derivada       salidas   0  zigzag±10mm 1.203/m  dirección  40.2  mapa  11.49s (+1.5 %)
adelanto 5:
  red de la derivada       salidas   0  zigzag±10mm 1.221/m  dirección  39.8  mapa  11.57s (+2.2 %)
adelanto 10:
  red de la derivada       salidas   0  zigzag±10mm 1.252/m  dirección  39.2  mapa  11.58s (+2.2 %)
```

Conclusión: la red estima la derivada con 77 % menos error, pero en lazo cerrado aumenta el zigzag visible (0.91 -> 1.2 /m) y no cumple el +3 % en 8 pistas/modelos, sobre todo con los robots de más retraso. Se deja desactivada (kDerivativeTrained = false).

## D combinado (filtro complementario), 2026-10-17

D = D clásico + α·(D red − D clásico). Red reentrenada con el predictor activo
(validación: red 10.0 contra D clásico 58.3 unidades/ciclo). 66 competencias
por variante:

| α | con predictor (zigzag ±10 mm) | sin predictor |
|---|---|---|
| 0 % (clásico) | **0.868/m** | **0.907/m** |
| 15 % | 0.890 | 0.932 |
| 25 % | 0.918 | 0.962 |
| 40 % | 0.949 | 0.997 |
| 60 % | 1.013 (no cumple) | 1.044 (no cumple) |
| 100 % | 1.189 (no cumple) | 1.237 (no cumple) |

Cualquier proporción de la red empeora: el "ruido" del D clásico lleva
información útil y adelantada (la línea pasando de un sensor a otro), y
suavizarlo quita amortiguación. Se despliega α = 0 (kDeployedDerivativeBlend);
los pesos quedan compilados pero la red no se evalúa.
