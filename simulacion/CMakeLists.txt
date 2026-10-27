find_package(Threads REQUIRED)

file(GLOB ATLAS_SIM_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/motor/*.cpp)
list(REMOVE_ITEM ATLAS_SIM_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/motor/main.cpp)

add_library(atlas_sim_lib STATIC ${ATLAS_SIM_SOURCES})
target_include_directories(atlas_sim_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/motor)
target_link_libraries(atlas_sim_lib PUBLIC atlas_core)
target_compile_options(atlas_sim_lib PRIVATE -Wall -Wextra)

add_executable(atlas_sim motor/main.cpp)
target_link_libraries(atlas_sim PRIVATE atlas_sim_lib Threads::Threads)
target_compile_definitions(atlas_sim PRIVATE
  ATLAS_VIEWER_TEMPLATE="${CMAKE_CURRENT_SOURCE_DIR}/visor/viewer_template.html"
  ATLAS_ANALYZER_TEMPLATE="${CMAKE_CURRENT_SOURCE_DIR}/visor/analyzer_template.html")

file(GLOB ATLAS_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/pruebas/*.cpp)
add_executable(atlas_tests ${ATLAS_TEST_SOURCES})
target_include_directories(atlas_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/pruebas)
target_link_libraries(atlas_tests PRIVATE atlas_sim_lib)
target_compile_options(atlas_tests PRIVATE -Wall -Wextra)

add_test(NAME unit_tests COMMAND atlas_tests)
add_test(NAME simulation_quick COMMAND atlas_sim --quick --out ${CMAKE_BINARY_DIR}/simulation_quick)

# Tres intentos con los ajustes reales de despliegue en la pista de competencia.
add_test(NAME simulation_firmware COMMAND atlas_sim --firmware --track 0 --out ${CMAKE_BINARY_DIR}/simulation_firmware)
add_test(NAME simulation_firmware_classic COMMAND atlas_sim --firmware --sin-red --track 0 --out ${CMAKE_BINARY_DIR}/simulation_firmware_classic)

add_custom_target(simulate
  COMMAND atlas_sim --out ${CMAKE_SOURCE_DIR}/resultados_simulacion
  DEPENDS atlas_sim
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Simulando todas las pistas" USES_TERMINAL)

# Optimizador de parámetros del firmware (no forma parte de ctest).
add_executable(atlas_tune ${CMAKE_SOURCE_DIR}/entrenamiento/ajuste/atlas_tune.cpp)
target_link_libraries(atlas_tune PRIVATE atlas_sim_lib Threads::Threads)

# Banco de casos extremos (imperfecciones del robot real y del entorno).
add_executable(atlas_extremes ${CMAKE_SOURCE_DIR}/diagnostico/atlas_extremes.cpp)
target_link_libraries(atlas_extremes PRIVATE atlas_sim_lib Threads::Threads)
target_compile_definitions(atlas_extremes PRIVATE
  ATLAS_VIEWER_TEMPLATE="${CMAKE_CURRENT_SOURCE_DIR}/visor/viewer_template.html")

# Optimizador del perfil Rápido + planificador del aprendizaje de pista.
add_executable(atlas_tune_learning ${CMAKE_SOURCE_DIR}/entrenamiento/ajuste/atlas_tune_learning.cpp)
target_link_libraries(atlas_tune_learning PRIVATE atlas_sim_lib Threads::Threads)

# Entrenamiento de las redes neuronales del firmware (escribe NeuralPositionWeights.h).
add_executable(atlas_train_nn ${CMAKE_SOURCE_DIR}/entrenamiento/algoritmos/atlas_train_nn.cpp)
target_link_libraries(atlas_train_nn PRIVATE atlas_sim_lib Threads::Threads)
target_compile_definitions(atlas_train_nn PRIVATE ATLAS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

# Entrenamiento de la red jerárquica de control (principal + 4 subneuronas).
add_executable(atlas_train_hierarchy ${CMAKE_SOURCE_DIR}/entrenamiento/algoritmos/atlas_train_hierarchy.cpp)
target_link_libraries(atlas_train_hierarchy PRIVATE atlas_sim_lib Threads::Threads)
target_compile_definitions(atlas_train_hierarchy PRIVATE ATLAS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

# Ajuste del modelo del robot a una vuelta real registrada por serial (atlas_log).
add_executable(atlas_fit_robot ${CMAKE_SOURCE_DIR}/entrenamiento/ajuste/atlas_fit_robot.cpp)
target_link_libraries(atlas_fit_robot PRIVATE atlas_sim_lib Threads::Threads)

add_executable(atlas_train_dagger ${CMAKE_SOURCE_DIR}/entrenamiento/algoritmos/atlas_train_dagger.cpp)
target_link_libraries(atlas_train_dagger PRIVATE atlas_sim_lib Threads::Threads)
target_compile_definitions(atlas_train_dagger PRIVATE ATLAS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
