cmake_minimum_required(VERSION 3.16)
project(AtlasLineFollower LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

enable_testing()

file(GLOB_RECURSE ATLAS_CORE_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/firmware/bibliotecas/AtlasCore/src/*.cpp)
add_library(atlas_core STATIC ${ATLAS_CORE_SOURCES})
set(ATLAS_CORE_INCLUDE_DIRS
  ${CMAKE_SOURCE_DIR}/firmware/bibliotecas/AtlasCore/src
  ${CMAKE_SOURCE_DIR}/firmware/bibliotecas/AtlasCore/src/app
  ${CMAKE_SOURCE_DIR}/firmware/bibliotecas/AtlasCore/src/configuracion
  ${CMAKE_SOURCE_DIR}/firmware/bibliotecas/AtlasCore/src/sensores
  ${CMAKE_SOURCE_DIR}/firmware/bibliotecas/AtlasCore/src/control
  ${CMAKE_SOURCE_DIR}/firmware/bibliotecas/AtlasCore/src/planificador
  ${CMAKE_SOURCE_DIR}/firmware/bibliotecas/AtlasCore/src/memoria
  ${CMAKE_SOURCE_DIR}/firmware/bibliotecas/AtlasCore/src/registro
  ${CMAKE_SOURCE_DIR}/firmware/bibliotecas/AtlasCore/src/red_neuronal
)
target_include_directories(atlas_core PUBLIC ${ATLAS_CORE_INCLUDE_DIRS})
target_compile_options(atlas_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(atlas_core_cxx14_check OBJECT ${ATLAS_CORE_SOURCES})
target_include_directories(atlas_core_cxx14_check PRIVATE ${ATLAS_CORE_INCLUDE_DIRS})
set_target_properties(atlas_core_cxx14_check PROPERTIES CXX_STANDARD 14)
target_compile_options(atlas_core_cxx14_check PRIVATE -Wall -Wextra -Wpedantic)

add_subdirectory(simulacion)

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
