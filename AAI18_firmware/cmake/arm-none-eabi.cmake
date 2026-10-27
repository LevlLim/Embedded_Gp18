# arm-none-eabi.cmake -- CMake toolchain file for the RP2040 (Cortex-M0+).
#
# Group AAI18.  Used by the top-level CMakeLists.txt.

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# ---------------------------------------------------------------------------
# Find arm-none-eabi-gcc even when it is NOT on PATH (plain VS Code on
# Windows: "CMAKE_C_COMPILER arm-none-eabi-gcc ... was not found in the
# PATH").  Searched in this order:
#   1. PATH
#   2. $ENV{AAI18_TOOLCHAIN_BIN} / $ENV{PICO_TOOLCHAIN_PATH}[/bin]
#   3. the Raspberry Pi Pico VS Code extension:  ~/.pico-sdk/toolchain/*/bin
#   4. the Pico Windows installer:    C:/Program Files/Raspberry Pi/*/gcc-arm-none-eabi/bin
#   5. the Arm GNU Toolchain installer: C:/Program Files*/Arm GNU Toolchain arm-none-eabi/*/bin
# Ninja is looked for the same way (~/.pico-sdk/ninja/*).
# ---------------------------------------------------------------------------
set(_aai18_home "$ENV{USERPROFILE}")
if(NOT _aai18_home)
    set(_aai18_home "$ENV{HOME}")
endif()
file(TO_CMAKE_PATH "${_aai18_home}" _aai18_home)

set(_aai18_hints)
foreach(_v AAI18_TOOLCHAIN_BIN PICO_TOOLCHAIN_PATH)
    if(DEFINED ENV{${_v}})
        file(TO_CMAKE_PATH "$ENV{${_v}}" _p)
        list(APPEND _aai18_hints "${_p}" "${_p}/bin")
    endif()
endforeach()
file(GLOB _aai18_found LIST_DIRECTORIES true
     "${_aai18_home}/.pico-sdk/toolchain/*/bin"
     "C:/Program Files/Raspberry Pi/*/gcc-arm-none-eabi/bin"
     "C:/Program Files/Arm GNU Toolchain arm-none-eabi/*/bin"
     "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/*/bin"
     "C:/Program Files (x86)/GNU Arm Embedded Toolchain/*/bin")
list(SORT _aai18_found ORDER DESCENDING)       # newest version first
list(APPEND _aai18_hints ${_aai18_found})

find_program(AAI18_ARM_GCC arm-none-eabi-gcc HINTS ${_aai18_hints})
if(NOT AAI18_ARM_GCC)
    message(FATAL_ERROR
        "arm-none-eabi-gcc not found.  Either open VS Code with the "
        "\"Pico - Visual Studio Code\" shortcut, install the Arm GNU "
        "Toolchain (tick \"Add path to environment variable\"), or set "
        "the environment variable AAI18_TOOLCHAIN_BIN to its bin folder.")
endif()
get_filename_component(AAI18_TOOLCHAIN_DIR "${AAI18_ARM_GCC}" DIRECTORY)

set(CMAKE_C_COMPILER   "${AAI18_ARM_GCC}")
set(CMAKE_ASM_COMPILER "${AAI18_ARM_GCC}")
find_program(AAI18_ARM_GXX arm-none-eabi-g++ HINTS "${AAI18_TOOLCHAIN_DIR}")
if(AAI18_ARM_GXX)
    set(CMAKE_CXX_COMPILER "${AAI18_ARM_GXX}")
endif()
find_program(AAI18_ARM_SIZE arm-none-eabi-size HINTS "${AAI18_TOOLCHAIN_DIR}")
if(NOT AAI18_ARM_SIZE)
    set(AAI18_ARM_SIZE arm-none-eabi-size)
endif()

if(NOT CMAKE_MAKE_PROGRAM)
    file(GLOB _aai18_ninja LIST_DIRECTORIES true
         "${_aai18_home}/.pico-sdk/ninja/*"
         "C:/Program Files/Raspberry Pi/*/ninja")
    find_program(AAI18_NINJA ninja HINTS ${_aai18_ninja})
    if(AAI18_NINJA)
        set(CMAKE_MAKE_PROGRAM "${AAI18_NINJA}" CACHE FILEPATH "Ninja")
    endif()
endif()

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
