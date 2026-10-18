cmake_minimum_required(VERSION 3.16)

project(buddy3_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_executable(buddy3_host_tests
    test_buddy3_main.c
    ../src/buddy3/barcode_decoder.c
    ../src/buddy3/buddy3_config.c
    ../src/buddy3/line_sensor.c
)

target_include_directories(buddy3_host_tests
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../src/buddy3
)

if(MSVC)
    target_compile_options(buddy3_host_tests PRIVATE /W4 /WX)
else()
    target_compile_options(
        buddy3_host_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
            -Wpedantic
    )
endif()
