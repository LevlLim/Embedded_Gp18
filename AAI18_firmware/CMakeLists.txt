# CMakeLists.txt -- CMake build for INF2004_ProjectTemplate + Group AAI18.
#
# A CMake/Ninja equivalent of build_make/makefile + pico_rp2040.mk, for
# building on Windows WITHOUT WSL: it needs only the tools the "Pico -
# Visual Studio Code" program already provides (CMake, Ninja,
# arm-none-eabi-gcc and Python).  It compiles the same files with the same
# flags and produces the same .elf/.uf2 names as the makefile.
#
# OPTIONS (set in VS Code's CMake settings, or with -D on the command line)
#   AAI18_CONSOLE    AUTO | usb_cdc | uart   (AUTO: usb_cdc if a Pico SDK
#                                              2.x is found, else uart.
#                                              AAI18: uart is REFUSED because
#                                              the ultrasonic uses GP0/GP1)
#   AAI18_WIFI       AUTO | cyw43 | none     (AUTO: same rule)
#   AAI18_NET        none | lwip
#   AAI18_MQTT       OFF | ON                (needs AAI18_NET=lwip)
#   AAI18_TEST       OFF | ON                (subsystem self-tests)
#   AAI18_TEST_BUDDY 0 | 1..5                (one buddy's self-test only)
#   AAI18_CRASH_REPORT OFF | ON              (USB console mirrored to UART0)
#   AAI18_PICO_SDK_PATH  path to Pico SDK 2.2.0 or newer, with submodules
#
# The uart/none build needs NO Pico SDK, but THIS project cannot use it:
# the ultrasonic sensor is wired to GP0/GP1, the UART0 console pins.  The USB console and Wi-Fi build
# need Pico SDK 2.x: the SDK 1.5 bundled with the Pico Windows installer is
# too old (the template uses headers added in SDK 2.0).
#
# Only SMP=1 (dual core, the template default) is provided.

cmake_minimum_required(VERSION 3.13)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/cmake/arm-none-eabi.cmake)
endif()

project(aai18_robot C ASM)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
set(AAI18_CONSOLE AUTO CACHE STRING "Console: AUTO, usb_cdc or uart")
set_property(CACHE AAI18_CONSOLE PROPERTY STRINGS AUTO usb_cdc uart)
set(AAI18_WIFI AUTO CACHE STRING "Wi-Fi: AUTO, cyw43 or none")
set_property(CACHE AAI18_WIFI PROPERTY STRINGS AUTO cyw43 none)
set(AAI18_NET none CACHE STRING "Network stack: none or lwip")
set_property(CACHE AAI18_NET PROPERTY STRINGS none lwip)
option(AAI18_MQTT "MQTT client (needs AAI18_NET=lwip)" OFF)
option(AAI18_TEST "Build the subsystem self-tests (APP_TEST=1)" OFF)
set(AAI18_TEST_BUDDY 0 CACHE STRING
    "Self-test of ONE buddy only: 1..5 (implies AAI18_TEST), 0 = all")
set_property(CACHE AAI18_TEST_BUDDY PROPERTY STRINGS 0 1 2 3 4 5)
option(AAI18_CRASH_REPORT "Mirror the USB console to UART0 on GP0/GP1" OFF)

if(DEFINED ENV{AAI18_PICO_SDK_PATH})
    set(_sdk_default $ENV{AAI18_PICO_SDK_PATH})
elseif(DEFINED ENV{PICO_SDK_PATH})
    set(_sdk_default $ENV{PICO_SDK_PATH})
else()
    set(_sdk_default "")
endif()
set(AAI18_PICO_SDK_PATH "${_sdk_default}" CACHE PATH
    "Pico SDK 2.x (only for usb_cdc / cyw43 builds)")

# A usable SDK is 2.x with its submodules checked out.
set(SDK ${AAI18_PICO_SDK_PATH})
set(_sdk_ok FALSE)
if(SDK AND EXISTS "${SDK}/src/rp2_common/pico_platform_compiler"
   AND EXISTS "${SDK}/lib/tinyusb/src/tusb.c"
   AND EXISTS "${SDK}/lib/cyw43-driver/src/cyw43_ll.c")
    set(_sdk_ok TRUE)
endif()

set(CONSOLE ${AAI18_CONSOLE})
set(WIFI ${AAI18_WIFI})
if(CONSOLE STREQUAL "AUTO")
    if(_sdk_ok)
        set(CONSOLE usb_cdc)
    else()
        set(CONSOLE uart)
    endif()
endif()
if(WIFI STREQUAL "AUTO")
    if(_sdk_ok)
        set(WIFI cyw43)
    else()
        set(WIFI none)
    endif()
endif()

# GP0/GP1 carry the ultrasonic sensor (app.h), so UART0 can never be the
# console or the crash mirror in this project.
if(CONSOLE STREQUAL "uart" OR AAI18_CRASH_REPORT)
    message(FATAL_ERROR
        "GP0/GP1 are wired to the ultrasonic sensor, so the UART console "
        "and CRASH_REPORT cannot be used. Build with AAI18_CONSOLE=usb_cdc "
        "and AAI18_CRASH_REPORT=OFF.")
endif()

if((CONSOLE STREQUAL "usb_cdc" OR WIFI STREQUAL "cyw43") AND NOT _sdk_ok)
    message(FATAL_ERROR
        "CONSOLE=${CONSOLE} WIFI=${WIFI} needs Pico SDK 2.x with submodules.\n"
        "AAI18_PICO_SDK_PATH='${SDK}' is missing or too old (SDK 1.5 from the "
        "Pico Windows installer will not work).\n"
        "Either set AAI18_PICO_SDK_PATH to an SDK 2.2.0 checkout, or set "
        "AAI18_CONSOLE=uart and AAI18_WIFI=none.")
endif()
if(AAI18_NET STREQUAL "lwip" AND NOT WIFI STREQUAL "cyw43")
    message(FATAL_ERROR "AAI18_NET=lwip requires WIFI=cyw43")
endif()
if(AAI18_MQTT AND NOT AAI18_NET STREQUAL "lwip")
    message(FATAL_ERROR "AAI18_MQTT=ON requires AAI18_NET=lwip")
endif()

# CONSOLE=uart always uses UART0, so it forces the crash-report mirror.
if(CONSOLE STREQUAL "uart")
    set(CRASH_REPORT 1)
elseif(AAI18_CRASH_REPORT)
    set(CRASH_REPORT 1)
else()
    set(CRASH_REPORT 0)
endif()
if(NOT AAI18_TEST_BUDDY MATCHES "^[0-5]$")
    message(FATAL_ERROR "AAI18_TEST_BUDDY must be 0 (all) or 1..5")
endif()
if(NOT AAI18_TEST_BUDDY EQUAL 0)
    set(AAI18_TEST ON)
endif()
if(AAI18_TEST)
    set(APP_TEST 1)
else()
    set(APP_TEST 0)
endif()

# Image name, exactly as the makefile names it.
set(EXE_NAME "mtk3pico_smp1_${CONSOLE}")
if(WIFI STREQUAL "cyw43")
    string(APPEND EXE_NAME "_wifi")
endif()
if(AAI18_NET STREQUAL "lwip")
    string(APPEND EXE_NAME "_lwip")
endif()
if(AAI18_MQTT)
    string(APPEND EXE_NAME "_mqtt")
endif()
if(CONSOLE STREQUAL "usb_cdc" AND CRASH_REPORT)
    string(APPEND EXE_NAME "_crash")
endif()
if(AAI18_TEST AND NOT AAI18_TEST_BUDDY EQUAL 0)
    string(APPEND EXE_NAME "_test_b${AAI18_TEST_BUDDY}")
elseif(AAI18_TEST)
    string(APPEND EXE_NAME "_test")
endif()

message(STATUS "AAI18 build: CONSOLE=${CONSOLE} WIFI=${WIFI} NET=${AAI18_NET} "
               "MQTT=${AAI18_MQTT} TEST=${APP_TEST} -> ${EXE_NAME}.uf2")

# ---------------------------------------------------------------------------
# Sources, in the makefile's link order
# ---------------------------------------------------------------------------
set(T ${CMAKE_CURRENT_SOURCE_DIR})

set(KERNEL_SRCS_A
    lib/libtm/libtm.c lib/libtm/libtm_printf.c
    lib/libtk/fastlock.c lib/libtk/fastmlock.c lib/libtk/kmalloc.c
    kernel/usermain/usermain.c
    kernel/tstdlib/bitop.c kernel/tstdlib/string.c
    kernel/tkernel/cpuctl.c kernel/tkernel/device.c kernel/tkernel/deviceio.c
    kernel/tkernel/eventflag.c kernel/tkernel/int.c kernel/tkernel/klock.c
    kernel/tkernel/mailbox.c kernel/tkernel/memory.c kernel/tkernel/mempfix.c
    kernel/tkernel/mempool.c kernel/tkernel/messagebuf.c
    kernel/tkernel/misc_calls.c kernel/tkernel/mutex.c kernel/tkernel/objname.c
    kernel/tkernel/power.c kernel/tkernel/rendezvous.c
    kernel/tkernel/semaphore.c kernel/tkernel/smp_lock.c
    kernel/tkernel/smp_dispatch.c kernel/tkernel/ready_queue_smp.c
    kernel/tkernel/task.c kernel/tkernel/task_manage.c
    kernel/tkernel/task_sync.c kernel/tkernel/time_calls.c
    kernel/tkernel/timer.c kernel/tkernel/tkinit.c kernel/tkernel/wait.c
    kernel/sysinit/sysinit.c kernel/inittask/inittask.c
    device/common/drvif/msdrvif.c
    device/adc/adc.c device/adc/sysdepend/rp2040/adc_rp2040.c
    device/i2c/i2c.c device/i2c/sysdepend/rp2040/i2c_rp2040.c
    device/ser/ser.c device/ser/sysdepend/rp2040/ser_rp2040.c)

set(KERNEL_SRCS_B
    lib/libtm/sysdepend/pico_rp2040/tm_com.c
    lib/libtm/sysdepend/no_device/tm_com.c
    lib/libtk/sysdepend/cpu/rp2040/int_rp2040.c
    lib/libtk/sysdepend/cpu/rp2040/ptimer_rp2040.c
    lib/libtk/sysdepend/cpu/core/armv6m/int_armv6m.c
    lib/libtk/sysdepend/cpu/core/armv6m/wusec_armv6m.c
    lib/libbsp/sysdepend/cpu/rp2040/libbsp_pwm.c
    kernel/sysdepend/pico_rp2040/cpu_clock.c
    kernel/sysdepend/pico_rp2040/devinit.c
    kernel/sysdepend/pico_rp2040/hw_setting.c
    kernel/sysdepend/pico_rp2040/power_save.c
    kernel/sysdepend/cpu/rp2040/boot_2nd.c
    kernel/sysdepend/cpu/rp2040/vector_tbl.c
    kernel/sysdepend/cpu/rp2040/smp_rp2040.c
    kernel/sysdepend/cpu/rp2040/smp_atomic.c
    kernel/sysdepend/cpu/core/armv6m/cpu_cntl.c
    kernel/sysdepend/cpu/core/armv6m/dispatch.S
    kernel/sysdepend/cpu/core/armv6m/exc_hdr.c
    kernel/sysdepend/cpu/core/armv6m/interrupt.c
    kernel/sysdepend/cpu/core/armv6m/reset_hdl.c)

set(USB_SRCS
    lib/libtm/sysdepend/pico_rp2040/usb/usb_console.c
    lib/libtm/sysdepend/pico_rp2040/usb/usb_descriptors.c
    lib/libtm/sysdepend/pico_rp2040/usb/usb_sdk_compat.c
    lib/libtm/sysdepend/pico_rp2040/usb/usb_tinyusb_glue.c
    ${SDK}/lib/tinyusb/src/tusb.c
    ${SDK}/lib/tinyusb/src/common/tusb_fifo.c
    ${SDK}/lib/tinyusb/src/device/usbd.c
    ${SDK}/lib/tinyusb/src/device/usbd_control.c
    ${SDK}/lib/tinyusb/src/class/cdc/cdc_device.c
    ${SDK}/lib/tinyusb/src/portable/raspberrypi/rp2040/rp2040_usb.c
    ${SDK}/lib/tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.c)

set(WIFI_SRCS
    lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c
    lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c
    lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_compat.c
    ${SDK}/lib/cyw43-driver/src/cyw43_ll.c
    ${SDK}/lib/cyw43-driver/src/cyw43_ctrl.c
    ${SDK}/lib/cyw43-driver/src/cyw43_stats.c
    ${SDK}/src/rp2_common/pico_cyw43_driver/cyw43_bus_pio_spi.c
    ${SDK}/src/rp2_common/hardware_pio/pio.c
    ${SDK}/src/rp2_common/hardware_dma/dma.c
    ${SDK}/src/rp2_common/hardware_gpio/gpio.c)

set(LWIP_SRCS)
foreach(f core/init.c core/def.c core/dns.c core/inet_chksum.c core/ip.c
          core/mem.c core/memp.c core/netif.c core/pbuf.c core/raw.c
          core/stats.c core/sys.c core/altcp.c core/altcp_alloc.c
          core/altcp_tcp.c core/tcp.c core/tcp_in.c core/tcp_out.c
          core/timeouts.c core/udp.c core/ipv4/acd.c core/ipv4/autoip.c
          core/ipv4/dhcp.c core/ipv4/etharp.c core/ipv4/icmp.c
          core/ipv4/igmp.c core/ipv4/ip4_frag.c core/ipv4/ip4.c
          core/ipv4/ip4_addr.c netif/ethernet.c)
    list(APPEND LWIP_SRCS ${SDK}/lib/lwip/src/${f})
endforeach()
list(APPEND LWIP_SRCS
    lib/liblwip/sysdepend/pico_rp2040/sys_arch.c
    ${SDK}/lib/cyw43-driver/src/cyw43_lwip.c)

set(MQTT_SRCS
    ${SDK}/lib/lwip/src/apps/mqtt/mqtt.c
    lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c)

# The application: every .c one and two levels below app_program/, as the
# template's subdir.mk globs them.
file(GLOB APP_SRCS_TOP CONFIGURE_DEPENDS ${T}/app_program/*.c)
file(GLOB APP_SRCS_SUB CONFIGURE_DEPENDS ${T}/app_program/*/*.c)
list(SORT APP_SRCS_TOP)
list(SORT APP_SRCS_SUB)

set(SRCS ${KERNEL_SRCS_A})
if(CONSOLE STREQUAL "usb_cdc")
    list(APPEND SRCS ${USB_SRCS})
endif()
if(WIFI STREQUAL "cyw43")
    list(APPEND SRCS ${WIFI_SRCS})
    if(AAI18_NET STREQUAL "lwip")
        list(APPEND SRCS ${LWIP_SRCS})
        if(AAI18_MQTT)
            list(APPEND SRCS ${MQTT_SRCS})
        endif()
    endif()
endif()
list(APPEND SRCS ${KERNEL_SRCS_B} ${APP_SRCS_TOP} ${APP_SRCS_SUB})

# ---------------------------------------------------------------------------
# Flags, in the makefile's order
# ---------------------------------------------------------------------------
set(CPU_FLAGS -mcpu=cortex-m0plus -mthumb -ffreestanding)
set(C_ONLY_FLAGS -std=gnu11 -O2 -g3 -mfloat-abi=soft)
set(ASM_ONLY_FLAGS -O2 -g3)
set(DEFS CNF_SMP=1)
set(C_DEFS APP_TEST=${APP_TEST} APP_TEST_BUDDY=${AAI18_TEST_BUDDY}
           TM_CONSOLE_UART=${CRASH_REPORT})
set(C_INCS)
set(C_EXTRA)
if(NOT CRASH_REPORT)
    list(APPEND C_DEFS BOARD_UART0_FREE=1)
endif()

set(SDK_PLATFORM_INCS
    ${SDK}/src/common/pico_base_headers/include
    ${SDK}/src/common/pico_binary_info/include
    ${SDK}/src/rp2040/pico_platform/include
    ${SDK}/src/rp2_common/pico_platform_compiler/include
    ${SDK}/src/rp2_common/pico_platform_sections/include
    ${SDK}/src/rp2_common/pico_platform_panic/include
    ${SDK}/src/rp2_common/pico_platform_common/include
    ${SDK}/src/rp2_common/hardware_base/include
    ${SDK}/src/rp2_common/hardware_irq/include
    ${SDK}/src/rp2_common/hardware_resets/include
    ${SDK}/src/rp2_common/hardware_sync/include
    ${SDK}/src/rp2_common/hardware_sync_spin_lock/include
    ${SDK}/src/rp2_common/hardware_timer/include
    ${SDK}/src/rp2040/hardware_regs/include
    ${SDK}/src/rp2040/hardware_structs/include)

if(CONSOLE STREQUAL "usb_cdc")
    list(APPEND C_DEFS TM_CONSOLE_USB_CDC=1 NDEBUG)
    list(APPEND C_INCS ${T}/lib/libtm/sysdepend/pico_rp2040/usb
                       ${SDK}/lib/tinyusb/src ${SDK_PLATFORM_INCS})
endif()

if(WIFI STREQUAL "cyw43")
    if(AAI18_NET STREQUAL "lwip")
        set(_cyw43_lwip 1)
    else()
        set(_cyw43_lwip 0)
    endif()
    list(APPEND C_DEFS TM_WIFI_CYW43=1 CYW43_LWIP=${_cyw43_lwip}
         CYW43_ENABLE_BLUETOOTH=0 CYW43_USE_OTP_MAC=1
         PICO_CYW43_LOGGING_ENABLED=0 PICO_RP2040=1 PICO_32BIT=1
         PICO_ON_DEVICE=1 PICO_BUILD=1 PICO_NO_HARDWARE=0
         RASPBERRYPI_PICO_W=1
         CYW43_DEFAULT_PIN_WL_REG_ON=23u CYW43_DEFAULT_PIN_WL_DATA_OUT=24u
         CYW43_DEFAULT_PIN_WL_DATA_IN=24u CYW43_DEFAULT_PIN_WL_HOST_WAKE=24u
         CYW43_DEFAULT_PIN_WL_CLOCK=29u CYW43_DEFAULT_PIN_WL_CS=25u NDEBUG)
    list(APPEND C_EXTRA -ffunction-sections -fdata-sections)
    list(APPEND C_INCS
         ${T}/lib/libwifi/sysdepend/pico_rp2040
         ${T}/lib/libtm/sysdepend/pico_rp2040/usb
         ${SDK}/lib/cyw43-driver/src
         ${SDK}/lib/cyw43-driver/firmware
         ${SDK}/src/rp2_common/pico_cyw43_driver/include
         ${SDK}/src/boards/include
         ${SDK}/src/common/hardware_claim/include
         ${SDK}/src/common/pico_util/include
         ${SDK_PLATFORM_INCS}
         ${SDK}/src/rp2_common/hardware_pio/include
         ${SDK}/src/rp2_common/hardware_gpio/include
         ${SDK}/src/rp2_common/hardware_dma/include
         ${SDK}/src/rp2_common/hardware_clocks/include)
    if(AAI18_NET STREQUAL "lwip")
        list(APPEND C_DEFS TM_NET_LWIP=1)
        list(APPEND C_INCS ${T}/lib/liblwip/sysdepend/pico_rp2040/include
                           ${T}/lib/liblwip/sysdepend/pico_rp2040
                           ${SDK}/lib/lwip/src/include)
        if(AAI18_MQTT)
            list(APPEND C_DEFS TM_MQTT=1)
        endif()
    endif()
endif()

list(APPEND DEFS _PICO_RP2040_)
set(BASE_INCS ${T}/include ${T}/config ${T}/kernel/knlinc)

# ---------------------------------------------------------------------------
# The firmware image
# ---------------------------------------------------------------------------
add_executable(${EXE_NAME} ${SRCS})
set_target_properties(${EXE_NAME} PROPERTIES SUFFIX ".elf")

target_compile_options(${EXE_NAME} PRIVATE
    ${CPU_FLAGS}
    $<$<COMPILE_LANGUAGE:C>:${C_ONLY_FLAGS}>
    $<$<COMPILE_LANGUAGE:C>:${C_EXTRA}>
    $<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>
    $<$<COMPILE_LANGUAGE:ASM>:${ASM_ONLY_FLAGS}>)
target_compile_definitions(${EXE_NAME} PRIVATE
    ${DEFS} $<$<COMPILE_LANGUAGE:C>:${C_DEFS}>)
target_include_directories(${EXE_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:C>:${C_INCS}> ${BASE_INCS})

set(LINK_FLAGS ${CPU_FLAGS} -nostartfiles -O2 -g3 -mfloat-abi=soft)
if(WIFI STREQUAL "cyw43")
    list(APPEND LINK_FLAGS -Wl,--gc-sections)
endif()
target_link_options(${EXE_NAME} PRIVATE ${LINK_FLAGS}
    -T ${T}/etc/linker/pico_rp2040/tkernel_map.ld
    -Wl,-Map,${EXE_NAME}.map)
set_target_properties(${EXE_NAME} PROPERTIES
    LINK_DEPENDS ${T}/etc/linker/pico_rp2040/tkernel_map.ld)

# .uf2 for drag-and-drop flashing, and the size report the makefile prints.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_custom_command(TARGET ${EXE_NAME} POST_BUILD
    COMMAND arm-none-eabi-size ${EXE_NAME}.elf
    COMMAND ${Python3_EXECUTABLE} ${T}/cmake/elf2uf2.py
            ${EXE_NAME}.elf ${EXE_NAME}.uf2
    COMMENT "Writing ${EXE_NAME}.uf2"
    VERBATIM)
