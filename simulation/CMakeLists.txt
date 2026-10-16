find_package(Threads REQUIRED)

file(GLOB ATLAS_SIM_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/sim/*.cpp)
list(REMOVE_ITEM ATLAS_SIM_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/sim/main.cpp)

add_library(atlas_sim_lib STATIC ${ATLAS_SIM_SOURCES})
target_include_directories(atlas_sim_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/sim)
target_link_libraries(atlas_sim_lib PUBLIC atlas_core)
target_compile_options(atlas_sim_lib PRIVATE -Wall -Wextra)

add_executable(atlas_sim sim/main.cpp)
target_link_libraries(atlas_sim PRIVATE atlas_sim_lib Threads::Threads)
target_compile_definitions(atlas_sim PRIVATE
  ATLAS_VIEWER_TEMPLATE="${CMAKE_CURRENT_SOURCE_DIR}/viewer/viewer_template.html"
  ATLAS_ANALYZER_TEMPLATE="${CMAKE_CURRENT_SOURCE_DIR}/viewer/analyzer_template.html")

file(GLOB ATLAS_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
add_executable(atlas_tests ${ATLAS_TEST_SOURCES})
target_include_directories(atlas_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(atlas_tests PRIVATE atlas_sim_lib)
target_compile_options(atlas_tests PRIVATE -Wall -Wextra)

add_test(NAME unit_tests COMMAND atlas_tests)
add_test(NAME simulation_quick COMMAND atlas_sim --quick --out ${CMAKE_BINARY_DIR}/simulation_quick)

# Tres intentos con los ajustes reales de despliegue en la pista de competencia.
add_test(NAME simulation_firmware COMMAND atlas_sim --firmware --track 0 --out ${CMAKE_BINARY_DIR}/simulation_firmware)
add_test(NAME simulation_firmware_classic COMMAND atlas_sim --firmware --sin-red --track 0 --out ${CMAKE_BINARY_DIR}/simulation_firmware_classic)

add_custom_target(simulate
  COMMAND atlas_sim --out ${CMAKE_SOURCE_DIR}/simulation_output
  DEPENDS atlas_sim
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Simulando todas las pistas" USES_TERMINAL)

# Optimizador de parámetros del firmware (no forma parte de ctest).
add_executable(atlas_tune tools/atlas_tune.cpp)
target_link_libraries(atlas_tune PRIVATE atlas_sim_lib Threads::Threads)

# Banco de casos extremos (imperfecciones del robot real y del entorno).
add_executable(atlas_extremes tools/atlas_extremes.cpp)
target_link_libraries(atlas_extremes PRIVATE atlas_sim_lib Threads::Threads)
target_compile_definitions(atlas_extremes PRIVATE
  ATLAS_VIEWER_TEMPLATE="${CMAKE_CURRENT_SOURCE_DIR}/viewer/viewer_template.html")

# Optimizador del perfil Rápido + planificador del aprendizaje de pista.
add_executable(atlas_tune_learning tools/atlas_tune_learning.cpp)
target_link_libraries(atlas_tune_learning PRIVATE atlas_sim_lib Threads::Threads)

# Entrenamiento de las redes neuronales del firmware (escribe NeuralPositionWeights.h).
add_executable(atlas_train_nn tools/atlas_train_nn.cpp)
target_link_libraries(atlas_train_nn PRIVATE atlas_sim_lib Threads::Threads)
target_compile_definitions(atlas_train_nn PRIVATE ATLAS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
