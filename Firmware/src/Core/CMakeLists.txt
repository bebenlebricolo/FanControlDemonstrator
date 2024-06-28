cmake_minimum_required(VERSION 3.20)
project(NanoThermostat_CoreLib C CXX)

add_library(core STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/bridge.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bridge.h
    ${CMAKE_CURRENT_SOURCE_DIR}/interpolation.c
    ${CMAKE_CURRENT_SOURCE_DIR}/interpolation.h
    ${CMAKE_CURRENT_SOURCE_DIR}/buffers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/buffers.h
    ${CMAKE_CURRENT_SOURCE_DIR}/buttons.c
    ${CMAKE_CURRENT_SOURCE_DIR}/buttons.h
    ${CMAKE_CURRENT_SOURCE_DIR}/current.c
    ${CMAKE_CURRENT_SOURCE_DIR}/current.h
    ${CMAKE_CURRENT_SOURCE_DIR}/led.h
    ${CMAKE_CURRENT_SOURCE_DIR}/led.c
    ${CMAKE_CURRENT_SOURCE_DIR}/spanner.c
    ${CMAKE_CURRENT_SOURCE_DIR}/spanner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/mcu_time.h
    ${CMAKE_CURRENT_SOURCE_DIR}/mcu_time.c
    ${CMAKE_CURRENT_SOURCE_DIR}/thermistor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/thermistor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/ntc_10k_3977.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ntc_10k_3977.h
)

add_subdirectory(Tests
    ${CMAKE_BINARY_DIR}/Tests
)

set_target_properties(core
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)