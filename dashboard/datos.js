window.ATLAS_DATOS = {
  "generado": "2026-10-26T22:27:38.517332+00:00",
  "con_red": [],
  "sin_red": [],
  "entrenamiento": {
    "curva": [
      {
        "epoca": 0,
        "entrenamiento": 0.2356,
        "validacion": 0.0739
      },
      {
        "epoca": 50,
        "entrenamiento": 0.0542,
        "validacion": 0.0585
      },
      {
        "epoca": 100,
        "entrenamiento": 0.0539,
        "validacion": 0.0582
      },
      {
        "epoca": 150,
        "entrenamiento": 0.0534,
        "validacion": 0.0577
      },
      {
        "epoca": 200,
        "entrenamiento": 0.0532,
        "validacion": 0.0577
      },
      {
        "epoca": 250,
        "entrenamiento": 0.0532,
        "validacion": 0.0576
      },
      {
        "epoca": 300,
        "entrenamiento": 0.0531,
        "validacion": 0.0576
      },
      {
        "epoca": 350,
        "entrenamiento": 0.0531,
        "validacion": 0.0576
      },
      {
        "epoca": 400,
        "entrenamiento": 0.0531,
        "validacion": 0.0576
      }
    ],
    "tabla": [
      {
        "grupo": "entrenamiento",
        "clasico": 134,
        "doble": 53,
        "enteros": 53,
        "maximo": 893
      },
      {
        "grupo": "validación",
        "clasico": 145,
        "doble": 58,
        "enteros": 58,
        "maximo": 614
      }
    ],
    "mejor_epoca": 205,
    "mejor_rms": 0.0575
  },
  "evaluacion": {
    "red": {
      "nombre": "Red de posición",
      "arquitectura": [
        7,
        8,
        1
      ],
      "activacion": "ReLU",
      "muestras": 50000,
      "turbina": false,
      "mejor_epoca": 205
    },
    "error_validacion": [
      {
        "metodo": "Promedio ponderado (clásico)",
        "rms": 145,
        "max": null
      },
      {
        "metodo": "Red anterior (40 000 lecturas, con turbina)",
        "rms": 59,
        "max": 784
      },
      {
        "metodo": "Red nueva (50 000 lecturas, sin turbina)",
        "rms": 58,
        "max": 614
      }
    ],
    "lazo_cerrado": {
      "carreras_por_fila": 792,
      "descripcion": "11 pistas obligatorias x robots nominal, moderado y real x 8 semillas x 3 intentos",
      "filas": [
        {
          "agarre_pct": 100,
          "fallas_con_red": 0,
          "fallas_sin_red": 7,
          "vuelta_mapa_con_red_s": 14.24,
          "vuelta_mapa_sin_red_s": 14.23
        },
        {
          "agarre_pct": 90,
          "fallas_con_red": 5,
          "fallas_sin_red": 4,
          "vuelta_mapa_con_red_s": 14.55,
          "vuelta_mapa_sin_red_s": 14.58
        },
        {
          "agarre_pct": 80,
          "fallas_con_red": 7,
          "fallas_sin_red": 9,
          "vuelta_mapa_con_red_s": 14.93,
          "vuelta_mapa_sin_red_s": 14.96
        }
      ]
    },
    "ajuste_sin_turbina": {
      "antes": {
        "fallas": 48,
        "carreras": 297,
        "nota": "ajustes de la rama con turbina corriendo sin turbina"
      },
      "despues": {
        "fallas": 0,
        "carreras": 792
      }
    },
    "limitador_aceleracion": [
      {
        "pwm_por_s": 0,
        "vuelta_mapa_s": 14.15
      },
      {
        "pwm_por_s": 600,
        "vuelta_mapa_s": 14.15
      },
      {
        "pwm_por_s": 400,
        "vuelta_mapa_s": 14.24
      },
      {
        "pwm_por_s": 250,
        "vuelta_mapa_s": 14.58
      }
    ]
  },
  "red": {
    "Weights1": [
      -42,
      -30,
      38,
      50,
      -3,
      -66,
      -31,
      -13,
      -4,
      24,
      -22,
      -45,
      47,
      41,
      -8,
      71,
      -71,
      7,
      -44,
      -22,
      -21,
      27,
      -3,
      -50,
      -63,
      -18,
      88,
      16,
      -66,
      -16,
      18,
      -72,
      -8,
      16,
      -61,
      -39,
      -32,
      -30,
      76,
      26,
      52,
      -37,
      -12,
      50,
      15,
      51,
      -44,
      78,
      -37,
      6,
      4,
      -20,
      -57,
      16,
      127,
      47
    ],
    "Bias1": [
      828,
      -54,
      5980,
      458,
      910,
      217,
      -2464,
      5957
    ],
    "Weights2": [
      -2918,
      1928,
      3530,
      -120,
      -4918,
      3027,
      -3687,
      1770
    ],
    "Bias2": [
      3410412
    ]
  },
  "configuracion": {
    "speed.basePwm": 130.0,
    "speed.minPwm": 60.0,
    "speed.maxPwm": 250.0,
    "speed.stepPwm": 10.0,
    "speed.runRampMs": 150.0,
    "speed.maxAccelPwmPerS": 400.0,
    "speed.startKickMs": 0.0,
    "impeller.pwm": 0.0,
    "lostLine.outerMotorPwm": 110.0,
    "corner.speedPercent": 50.0,
    "planner.brakePwmPerBin": 7.0,
    "planner.maxPwm": 225.0,
    "planner.fullTurnPwm": 140.0
  },
  "visor": false,
  "avisos": [
    "Falta resultados_simulacion/firmware/resumen.csv. Para generarlo: ./build/simulacion/atlas_sim --firmware --out resultados_simulacion/firmware",
    "Falta resultados_simulacion/firmware_sin_red/resumen.csv. Para generarlo: ./build/simulacion/atlas_sim --firmware --sin-red --out resultados_simulacion/firmware_sin_red"
  ]
};
