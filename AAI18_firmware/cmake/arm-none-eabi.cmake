# arm-none-eabi.cmake -- CMake toolchain file for the RP2040 (Cortex-M0+).
#
# Group AAI18.  Used by the top-level CMakeLists.txt; it finds
# arm-none-eabi-gcc on PATH, which the "Pico - Visual Studio Code" program
# and the Pico VS Code extension both set up.

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

# A bare-metal compiler cannot link a test program without our startup code.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# The template's own flags (-O2 -g3) are set in CMakeLists.txt; stop CMake
# adding its per-build-type defaults on top of them.
foreach(lang C ASM CXX)
    foreach(cfg DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
        set(CMAKE_${lang}_FLAGS_${cfg}_INIT "")
    endforeach()
endforeach()

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
