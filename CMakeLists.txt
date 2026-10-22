# ATLAS Rev. 1.4 - firmware orientado a objetos + banco de simulación
#
#   cmake -S . -B build && cmake --build build -j
#   ctest --test-dir build --output-on-failure     # pruebas unitarias + simulación rápida
#   cmake --build build --target simulate          # todas las pistas -> simulation_output/index.html
#   cmake --build build --target firmware          # compila con PlatformIO (ATmega328P)
#   cmake --build build --target upload            # sube con PlatformIO + USBasp

cmake_minimum_required(VERSION 3.16)
project(AtlasLineFollower LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

enable_testing()

# --- Núcleo de control (el mismo código que corre en el robot) -------------
file(GLOB_RECURSE ATLAS_CORE_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/lib/AtlasCore/src/*.cpp)
add_library(atlas_core STATIC ${ATLAS_CORE_SOURCES})
target_include_directories(atlas_core PUBLIC ${CMAKE_SOURCE_DIR}/lib/AtlasCore/src)
target_compile_options(atlas_core PRIVATE -Wall -Wextra -Wpedantic)

# El firmware AVR se compila con gnu++14 (platformio.ini): se verifica que el
# núcleo no use nada más moderno.
add_library(atlas_core_cxx14_check OBJECT ${ATLAS_CORE_SOURCES})
set_target_properties(atlas_core_cxx14_check PROPERTIES CXX_STANDARD 14)
target_compile_options(atlas_core_cxx14_check PRIVATE -Wall -Wextra -Wpedantic)

add_subdirectory(simulation)

# --- Firmware con PlatformIO -----------------------------------------------
find_program(PLATFORMIO_EXECUTABLE NAMES pio platformio PATHS $ENV{HOME}/.platformio/penv/bin)
if(PLATFORMIO_EXECUTABLE)
  add_custom_target(firmware
    COMMAND ${PLATFORMIO_EXECUTABLE} run -e atlas
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Compilando firmware ATLAS con PlatformIO" USES_TERMINAL)
  add_custom_target(upload
    COMMAND ${PLATFORMIO_EXECUTABLE} run -e atlas -t upload
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Subiendo firmware ATLAS (USBasp)" USES_TERMINAL)
else()
  message(STATUS "PlatformIO no encontrado: los targets 'firmware' y 'upload' no estarán disponibles")
endif()
