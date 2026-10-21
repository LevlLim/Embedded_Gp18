# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/Users/tzhik/.pico-sdk/sdk/2.3.1/tools/pioasm")
  file(MAKE_DIRECTORY "C:/Users/tzhik/.pico-sdk/sdk/2.3.1/tools/pioasm")
endif()
file(MAKE_DIRECTORY
  "C:/Users/tzhik/OneDrive/Documents/SIT/Y2T1/INF2004 Embedded Systems Programming/Group Project/Car_VSC/build-debug/pioasm"
  "C:/Users/tzhik/OneDrive/Documents/SIT/Y2T1/INF2004 Embedded Systems Programming/Group Project/Car_VSC/build-debug/pioasm-install"
  "C:/Users/tzhik/OneDrive/Documents/SIT/Y2T1/INF2004 Embedded Systems Programming/Group Project/Car_VSC/build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/tmp"
  "C:/Users/tzhik/OneDrive/Documents/SIT/Y2T1/INF2004 Embedded Systems Programming/Group Project/Car_VSC/build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/pioasmBuild-stamp"
  "C:/Users/tzhik/OneDrive/Documents/SIT/Y2T1/INF2004 Embedded Systems Programming/Group Project/Car_VSC/build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src"
  "C:/Users/tzhik/OneDrive/Documents/SIT/Y2T1/INF2004 Embedded Systems Programming/Group Project/Car_VSC/build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/pioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/tzhik/OneDrive/Documents/SIT/Y2T1/INF2004 Embedded Systems Programming/Group Project/Car_VSC/build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/pioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/tzhik/OneDrive/Documents/SIT/Y2T1/INF2004 Embedded Systems Programming/Group Project/Car_VSC/build-debug/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/pioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
