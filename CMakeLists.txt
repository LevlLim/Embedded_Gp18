cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Target board")

include(pico_sdk_import.cmake)

project(car_vsc C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

set(BUDDY3_SOURCES
    src/buddy3/adc_sampler.c
    src/buddy3/barcode_decoder.c
    src/buddy3/buddy3_config.c
    src/buddy3/buddy3_task.c
    src/buddy3/line_sensor.c
)

add_library(buddy3 ${BUDDY3_SOURCES})

set_source_files_properties(${BUDDY3_SOURCES}
    PROPERTIES
        COMPILE_OPTIONS "-Wall;-Wextra;-Werror;-Wpedantic"
)

target_include_directories(buddy3
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/src/buddy3
)

target_link_libraries(buddy3
    PUBLIC
        hardware_adc
        pico_stdlib
        pico_sync
)

add_executable(car_vsc
    src/buddy3_main.c
)

set_source_files_properties(src/buddy3_main.c
    PROPERTIES
        COMPILE_OPTIONS "-Wall;-Wextra;-Werror;-Wpedantic"
)

target_link_libraries(car_vsc
    PRIVATE
        buddy3
        pico_stdlib
)

pico_enable_stdio_usb(car_vsc 1)
pico_enable_stdio_uart(car_vsc 0)
pico_add_extra_outputs(car_vsc)
