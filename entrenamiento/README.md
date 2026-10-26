# Entrenamiento y ajuste

Desde la raíz del repositorio, compila con `cmake -S . -B build && cmake --build build -j`. Los ejecutables quedan en `build/simulacion/`.

| Fuente | Qué hace | Ejemplo desde la raíz |
|---|---|---|
| `algoritmos/atlas_train_nn.cpp` | Entrena la posición fina (Adam y cuantización) y el derivativo; también explora ganancias y predictor de retraso. | `./build/simulacion/atlas_train_nn posicion 25000` |
| `algoritmos/atlas_train_hierarchy.cpp` | Entrena la principal supervisada y las subredes de control con sep-CMA-ES. | `./build/simulacion/atlas_train_hierarchy 100 entrenamiento/resultados/entrenamiento_jerarquica.json` |
| `ajuste/atlas_tune.cpp` | Optimiza perfiles de control con búsqueda aleatoria y refinamiento. | `./build/simulacion/atlas_tune 150 150 42 --robusto` |
| `ajuste/atlas_tune_learning.cpp` | Optimiza el perfil Rápido y el planificador del aprendizaje de pista. | `./build/simulacion/atlas_tune_learning 100 150 42` |
| `ajuste/atlas_fit_robot.cpp` | Ajusta el modelo físico a una vuelta real capturada por serial. | `./build/simulacion/atlas_fit_robot vuelta.bin --pista 0 --pwm 130` |

Otros modos: `atlas_train_nn ganancias`, `atlas_train_nn derivada 400000 0 escribir`, `atlas_train_nn derivada-prueba 3` y `atlas_train_nn predictor 2` (con el mismo prefijo `./build/simulacion/`). El modo `derivada` solo guarda pesos si recibe `escribir`; `posicion` y el entrenador jerárquico escriben sus pesos en `firmware/bibliotecas/AtlasCore/src/red_neuronal/`.

`resultados/` conserva informes de derivada y predictor, además del JSON y log jerárquicos. La salida JSON predeterminada del entrenador jerárquico está en esta carpeta. Para guardar su salida de terminal: añade `> entrenamiento/resultados/entrenamiento_jerarquica.log 2>&1` al comando.
