################################################################################
# micro T-Kernel 3.0 BSP makefile
#     Target Board: Raspberry Pi Pico
################################################################################

################################################################################
# Execution model
#
#   SMP=0  (default)  single core.  TK_MAX_CORE is 1, the per-core arrays
#                     collapse and the current-core index folds to a constant
#                     zero, so the generated code is equivalent to the
#                     pre-SMP port.
#   SMP=1             dual core.  TK_MAX_CORE is 2.
#
# Passed to the assembler as well: the dispatcher indexes the per-core arrays
# by SIO CPUID and needs the same profile.
################################################################################

SMP ?= 0

ifneq ($(SMP),0)
ifneq ($(SMP),1)
$(error Unknown SMP '$(SMP)'; use 0 or 1)
endif
endif

# Console default is needed here because the image name includes it.
#
# Defaults to uart because it remains the early/panic and core qualification
# profile.  The optional Phase 11 SMP USB-CDC profile is also qualified; it
# retains UART as a mirror and adds a core-0-owned service task plus an SMP-safe
# producer ring.
CONSOLE ?= uart

# Optional Pico W radio profile.  The default remains byte-compatible with
# the qualified release; WIFI=cyw43 adds the core-0-owned polling service and
# its Pico SDK 2.2.0 driver subset.
WIFI ?= none
ifneq ($(filter $(WIFI),none cyw43),$(WIFI))
$(error Unknown WIFI '$(WIFI)'; use none or cyw43)
endif

# Phase-1 IP roadmap gate.  Association is opt-in because it needs local
# credentials and must not change the already-qualified scan-only image.
WIFI_JOIN ?= 0
ifneq ($(filter $(WIFI_JOIN),0 1),$(WIFI_JOIN))
$(error Unknown WIFI_JOIN '$(WIFI_JOIN)'; use 0 or 1)
endif

# Phase-2 networking gate.  This connects the CYW43439 Ethernet callbacks to
# an lwIP netif, but deliberately leaves the interface at 0.0.0.0: static IP,
# ARP/ICMP qualification and DHCP belong to later phases.
WIFI_NETIF ?= 0
ifneq ($(filter $(WIFI_NETIF),0 1),$(WIFI_NETIF))
$(error Unknown WIFI_NETIF '$(WIFI_NETIF)'; use 0 or 1)
endif

# Phase-3 gate: static IPv4, next-hop ARP resolution and an ICMP echo.  Network
# values are local configuration because choosing an address without knowing
# the user's LAN can create a duplicate-IP conflict.
WIFI_STATIC ?= 0
ifneq ($(filter $(WIFI_STATIC),0 1),$(WIFI_STATIC))
$(error Unknown WIFI_STATIC '$(WIFI_STATIC)'; use 0 or 1)
endif
ifeq ($(WIFI_STATIC),1)
ifneq ($(WIFI_NETIF),1)
$(error WIFI_STATIC=1 requires WIFI_NETIF=1)
endif
ifeq ($(wildcard ../config/network_config.h),)
$(error WIFI_STATIC=1 needs config/network_config.h; copy it from config/network_config.example.h and choose an unused LAN address)
endif
endif

# Phase-4 gate: acquire an IPv4 address, netmask, gateway and DNS server by
# DHCP, then re-run the Phase-3 ARP and ICMP traffic checks with that lease.
WIFI_DHCP ?= 0
ifneq ($(filter $(WIFI_DHCP),0 1),$(WIFI_DHCP))
$(error Unknown WIFI_DHCP '$(WIFI_DHCP)'; use 0 or 1)
endif
ifeq ($(WIFI_DHCP),1)
ifneq ($(WIFI_NETIF),1)
$(error WIFI_DHCP=1 requires WIFI_NETIF=1)
endif
ifeq ($(WIFI_STATIC),1)
$(error WIFI_DHCP=1 and WIFI_STATIC=1 are mutually exclusive)
endif
endif

# Phase-5 gate: resolve a known hostname using the DNS server learned through
# DHCP.  DNS stays on the same NO_SYS radio-owner task as all other lwIP work.
WIFI_DNS ?= 0
ifneq ($(filter $(WIFI_DNS),0 1),$(WIFI_DNS))
$(error Unknown WIFI_DNS '$(WIFI_DNS)'; use 0 or 1)
endif
ifeq ($(WIFI_DNS),1)
ifneq ($(WIFI_DHCP),1)
$(error WIFI_DNS=1 requires WIFI_DHCP=1)
endif
endif

# Phase-6 gate: send a deterministic series through lwIP's raw UDP API to a
# LAN echo peer and require every payload to return intact. The endpoint is a
# local ignored file because the Windows host address varies by network.
WIFI_UDP ?= 0
ifneq ($(filter $(WIFI_UDP),0 1),$(WIFI_UDP))
$(error Unknown WIFI_UDP '$(WIFI_UDP)'; use 0 or 1)
endif
ifeq ($(WIFI_UDP),1)
ifneq ($(WIFI_DNS),1)
$(error WIFI_UDP=1 requires WIFI_DNS=1)
endif
ifeq ($(wildcard ../config/udp_test_config.h),)
$(error WIFI_UDP=1 needs config/udp_test_config.h; copy it from config/udp_test_config.example.h and set the Windows echo-server address)
endif
endif

# Phase-7 gate: connect an lwIP raw-TCP client to a LAN echo peer, exchange a
# deterministic record series over the byte stream, and prove an orderly
# bidirectional close with the pcb released.
WIFI_TCP ?= 0
ifneq ($(filter $(WIFI_TCP),0 1),$(WIFI_TCP))
$(error Unknown WIFI_TCP '$(WIFI_TCP)'; use 0 or 1)
endif
ifeq ($(WIFI_TCP),1)
ifneq ($(WIFI_UDP),1)
$(error WIFI_TCP=1 requires WIFI_UDP=1)
endif
ifeq ($(wildcard ../config/tcp_test_config.h),)
$(error WIFI_TCP=1 needs config/tcp_test_config.h; copy it from config/tcp_test_config.example.h and set the Windows echo-server address)
endif
endif

# Phase-8 gate: pipelined bulk transfer over the same TCP peer, proving the
# receive path reassembles a byte stream across arbitrary segment boundaries.
# Phase 7 could not: its stop-and-wait records each arrived as one segment.
WIFI_TCPBULK ?= 0
ifneq ($(filter $(WIFI_TCPBULK),0 1),$(WIFI_TCPBULK))
$(error Unknown WIFI_TCPBULK '$(WIFI_TCPBULK)'; use 0 or 1)
endif
ifeq ($(WIFI_TCPBULK),1)
ifneq ($(WIFI_TCP),1)
$(error WIFI_TCPBULK=1 requires WIFI_TCP=1)
endif
endif

# Group 18 MQTT profile. Avoid the port's sequential UDP/TCP echo tests.
WIFI_MQTT ?= 0
ifneq ($(filter $(WIFI_MQTT),0 1),$(WIFI_MQTT))
$(error Unknown WIFI_MQTT '$(WIFI_MQTT)'; use 0 or 1)
endif
ifeq ($(WIFI_MQTT),1)
ifneq ($(WIFI),cyw43)
$(error WIFI_MQTT=1 requires WIFI=cyw43)
endif
ifneq ($(WIFI_JOIN),1)
$(error WIFI_MQTT=1 requires WIFI_JOIN=1)
endif
ifneq ($(WIFI_NETIF),1)
$(error WIFI_MQTT=1 requires WIFI_NETIF=1)
endif
ifneq ($(WIFI_DHCP),1)
$(error WIFI_MQTT=1 requires WIFI_DHCP=1)
endif
ifneq ($(PROJECT_APP),1)
$(error WIFI_MQTT=1 requires PROJECT_APP=1)
endif
ifeq ($(wildcard ../config/mqtt_config.h),)
$(error WIFI_MQTT=1 needs config/mqtt_config.h; copy mqtt_config.example.h and set broker details)
endif
endif

ifeq ($(WIFI_NETIF),1)
ifneq ($(WIFI_JOIN),1)
$(error WIFI_NETIF=1 requires WIFI_JOIN=1)
endif
endif
ifeq ($(WIFI_JOIN),1)
ifneq ($(WIFI),cyw43)
$(error WIFI_JOIN=1 requires WIFI=cyw43)
endif
ifeq ($(wildcard ../config/wifi_credentials.h),)
$(error WIFI_JOIN=1 needs config/wifi_credentials.h; copy it from config/wifi_credentials.example.h and add local credentials)
endif
endif

ifeq ($(WIFI),cyw43)
WIFI_SUFFIX := _wifi
ifeq ($(WIFI_JOIN),1)
WIFI_SUFFIX := _wifi_join
endif
ifeq ($(WIFI_NETIF),1)
WIFI_SUFFIX := _wifi_netif
endif
ifeq ($(WIFI_STATIC),1)
WIFI_SUFFIX := _wifi_static
endif
ifeq ($(WIFI_DHCP),1)
WIFI_SUFFIX := _wifi_dhcp
endif
ifeq ($(WIFI_DNS),1)
WIFI_SUFFIX := _wifi_dns
endif
ifeq ($(WIFI_UDP),1)
WIFI_SUFFIX := _wifi_udp
endif
ifeq ($(WIFI_TCP),1)
WIFI_SUFFIX := _wifi_tcp
endif
ifeq ($(WIFI_TCPBULK),1)
WIFI_SUFFIX := _wifi_tcpbulk
endif
ifeq ($(WIFI_MQTT),1)
WIFI_SUFFIX := _wifi_mqtt
endif
endif

#
# Images are named after the profile they were built with.  Every image is
# flashed by hand from a downloads folder, so identically-named artifacts are
# a live hazard: an SMP=1 build was once flashed in place of an SMP=0 one and
# only the banner in the log revealed it.
#
EXE_FILE := mtk3pico_smp$(SMP)_$(CONSOLE)$(WIFI_SUFFIX)

GCC := arm-none-eabi-gcc
AS := arm-none-eabi-gcc
LINK := arm-none-eabi-gcc
SIZE := arm-none-eabi-size
# ELF -> UF2 packaging.  The RP2040 bootloader presents a mass-storage drive
# and accepts UF2, not ELF, so the linked image has to be repackaged.  A Pico
# SDK/CMake project gets this for free from pico_add_extra_outputs(); this
# tree uses the upstream BSP's plain make build, which has no notion of host
# helper tools, so build the converter on demand from the vendored source.
#
# This is a HOST binary: compiled with the host compiler, run on the host,
# never on the target.  Override HOSTCXX for a cross/host-split environment.
E2U          := tools/elf2uf2
E2U_DIR      := ../tools/elf2uf2
E2U_SRC      := $(E2U_DIR)/main.cpp
HOSTCXX      ?= g++
HOSTCXXFLAGS ?= -O2 -std=c++14

$(E2U): $(E2U_SRC) $(E2U_DIR)/elf.h $(E2U_DIR)/boot/uf2.h
	@echo 'Host tool: $@'
	@mkdir -p $(dir $@)
	$(HOSTCXX) $(HOSTCXXFLAGS) -I"$(E2U_DIR)" -o "$@" "$(E2U_SRC)"

CFLAGS := -mcpu=cortex-m0plus -mthumb -ffreestanding\
    -std=gnu11 \
    -O2 -g3 \
    -MMD -MP \
    -mfloat-abi=soft \

ASFLAGS := -mcpu=cortex-m0plus -mthumb -ffreestanding\
    -x assembler-with-cpp \
    -O2 -g3 \
    -MMD -MP \

LFLAGS := -mcpu=cortex-m0plus -mthumb -ffreestanding \
    -nostartfiles \
    -O2 -g3 \
    -mfloat-abi=soft \

LNKFILE := "../etc/linker/pico_rp2040/tkernel_map.ld"

# Applied here, after CFLAGS/ASFLAGS are assigned with ':=', which would
# otherwise discard anything appended above.
ifeq ($(SMP),1)
CFLAGS  += -DCNF_SMP=1
ASFLAGS += -DCNF_SMP=1
endif

################################################################################
# Console selection
#
#   CONSOLE=usb_cdc             USB CDC-ACM console, with the UART0 mirror
#                               retained for early boot and panics.
#   CONSOLE=uart                UART0 only, no TinyUSB dependency.
#
# usb_cdc pulls seven TinyUSB sources and a set of pico-sdk headers from
# PICO_SDK_PATH.  Only headers and those sources are used; the port does not
# link the pico-sdk runtime, and lib/libtm/sysdepend/pico_rp2040/usb/
# usb_sdk_compat.c supplies the few SDK hooks TinyUSB calls, mapped onto
# micro T-Kernel's tk_def_int().
################################################################################

################################################################################
# Stale-object guard
#
# Every profile compiles into the same object tree, so switching profiles
# without cleaning silently relinks the previous profile's objects: SMP=0 and
# SMP=1 built back to back produced byte-identical images. Record the profile
# and discard the objects whenever it changes. Only *.o and *.d are removed:
# build_make/mtkernel_3/ also holds the tracked subdir.mk files, so removing
# the directory wholesale deletes source. Images are named after their
# profile, so both can coexist. This runs at parse time, so it completes
# before any recipe executes.
################################################################################

PROFILE_ID := smp$(SMP)-$(CONSOLE)-wifi$(WIFI)-join$(WIFI_JOIN)-netif$(WIFI_NETIF)-static$(WIFI_STATIC)-dhcp$(WIFI_DHCP)-dns$(WIFI_DNS)-udp$(WIFI_UDP)-tcp$(WIFI_TCP)-bulk$(WIFI_TCPBULK)-mqtt$(WIFI_MQTT)-app$(PROJECT_APP)
PROFILE_FILE := .build_profile

ifneq ($(strip $(shell cat $(PROFILE_FILE) 2>/dev/null)),$(PROFILE_ID))
$(info Build profile is now $(PROFILE_ID); discarding objects built for another profile.)
$(shell find mtkernel_3 -name '*.o' -o -name '*.d' | xargs -r rm -f; \
	  rm -f $(EXE_FILE).elf $(EXE_FILE).map $(EXE_FILE).uf2; \
	  echo $(PROFILE_ID) > $(PROFILE_FILE))
endif


ifeq ($(CONSOLE),usb_cdc)

PICO_SDK_PATH ?= ../../sdk/pico-sdk
TINYUSB_PATH := $(PICO_SDK_PATH)/lib/tinyusb

ifeq ($(wildcard $(TINYUSB_PATH)/src/tusb.c),)
$(error TinyUSB not found at $(TINYUSB_PATH). Set PICO_SDK_PATH, or build with CONSOLE=uart)
endif

USBDIR := ../lib/libtm/sysdepend/pico_rp2040/usb

USB_SRCS := usb_console.c usb_descriptors.c usb_sdk_compat.c usb_tinyusb_glue.c
USB_OBJS := $(addprefix ./mtkernel_3/lib/libtm/sysdepend/pico_rp2040/usb/,$(USB_SRCS:.c=.o))

TINYUSB_SRCS := tusb.c \
                common/tusb_fifo.c \
                device/usbd.c \
                device/usbd_control.c \
                class/cdc/cdc_device.c \
                portable/raspberrypi/rp2040/rp2040_usb.c \
                portable/raspberrypi/rp2040/dcd_rp2040.c
TINYUSB_OBJS := $(addprefix ./mtkernel_3/tinyusb/,$(TINYUSB_SRCS:.c=.o))

USB_INCLUDES := -I"$(USBDIR)" \
                -I"$(TINYUSB_PATH)/src" \
                -I"$(PICO_SDK_PATH)/src/common/pico_base_headers/include" \
                -I"$(PICO_SDK_PATH)/src/common/pico_binary_info/include" \
                -I"$(PICO_SDK_PATH)/src/rp2040/pico_platform/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_compiler/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_sections/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_panic/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_common/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_base/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_irq/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_resets/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_sync/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_sync_spin_lock/include" \
                -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_timer/include" \
                -I"$(PICO_SDK_PATH)/src/rp2040/hardware_regs/include" \
                -I"$(PICO_SDK_PATH)/src/rp2040/hardware_structs/include"

CFLAGS += -DTM_CONSOLE_USB_CDC=1 -DNDEBUG $(USB_INCLUDES)

OBJS += $(USB_OBJS) $(TINYUSB_OBJS)
C_DEPS += $(USB_OBJS:.o=.d) $(TINYUSB_OBJS:.o=.d)

mtkernel_3/lib/libtm/sysdepend/pico_rp2040/usb/%.o: $(USBDIR)/%.c
	@echo 'Building file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

mtkernel_3/tinyusb/%.o: $(TINYUSB_PATH)/src/%.c
	@echo 'Building TinyUSB file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

else ifneq ($(CONSOLE),uart)
$(error Unknown CONSOLE '$(CONSOLE)'; use usb_cdc or uart)
endif

################################################################################
# CYW43439 radio (polling architecture; optional NO_SYS lwIP netif)
################################################################################

ifeq ($(WIFI),cyw43)

PICO_SDK_PATH ?= ../../sdk/pico-sdk
CYW43_DRIVER_PATH := $(PICO_SDK_PATH)/lib/cyw43-driver
CYW43_PICO_PATH := $(PICO_SDK_PATH)/src/rp2_common/pico_cyw43_driver
WIFIDIR := ../lib/libwifi/sysdepend/pico_rp2040
LWIP_PORT_DIR := ../lib/libnet/lwip
LWIP_PATH := $(PICO_SDK_PATH)/lib/lwip

ifeq ($(wildcard $(CYW43_DRIVER_PATH)/src/cyw43_ll.c),)
$(error CYW43 driver not found below $(PICO_SDK_PATH); initialize the pico-sdk submodules)
endif

ifeq ($(WIFI_NETIF),1)
ifeq ($(wildcard $(LWIP_PATH)/src/core/init.c),)
$(error lwIP not found below $(PICO_SDK_PATH); initialize the pico-sdk submodules)
endif
endif

WIFI_LOCAL_SRCS := cyw43_utk.c cyw43_utk_kernel.c cyw43_utk_compat.c
WIFI_LOCAL_OBJS := $(addprefix ./mtkernel_3/lib/libwifi/sysdepend/pico_rp2040/,$(WIFI_LOCAL_SRCS:.c=.o))

CYW43_OBJS := ./mtkernel_3/cyw43/cyw43_ll.o \
              ./mtkernel_3/cyw43/cyw43_ctrl.o \
              ./mtkernel_3/cyw43/cyw43_stats.o \
              ./mtkernel_3/cyw43/cyw43_bus_pio_spi.o \
              ./mtkernel_3/cyw43/pio.o \
              ./mtkernel_3/cyw43/dma.o \
              ./mtkernel_3/cyw43/gpio.o

ifeq ($(WIFI_NETIF),1)
CYW43_OBJS += ./mtkernel_3/cyw43/cyw43_lwip.o

# Match the Pico SDK's pico_lwip_core + IPv4 + Ethernet source sets.  Features
# not used in this phase compile away under the local lwipopts.h; keeping the
# upstream set intact makes later DHCP/DNS/TCP/UDP phases configuration-only.
LWIP_SRCS := core/init.c core/def.c core/dns.c core/inet_chksum.c \
             core/ip.c core/mem.c core/memp.c core/netif.c core/pbuf.c \
             core/raw.c core/stats.c core/sys.c core/altcp.c \
             core/altcp_alloc.c core/altcp_tcp.c core/tcp.c core/tcp_in.c \
             core/tcp_out.c core/timeouts.c core/udp.c \
             core/ipv4/autoip.c core/ipv4/dhcp.c core/ipv4/etharp.c \
             core/ipv4/icmp.c core/ipv4/igmp.c core/ipv4/ip4_frag.c \
             core/ipv4/ip4.c core/ipv4/ip4_addr.c core/ipv4/acd.c \
             netif/ethernet.c
LWIP_OBJS := $(addprefix ./mtkernel_3/lwip/,$(LWIP_SRCS:.c=.o))
ifeq ($(WIFI_MQTT),1)
LWIP_OBJS += ./mtkernel_3/lwip/apps/mqtt/mqtt.o
endif
LWIP_PORT_OBJS := ./mtkernel_3/lib/libnet/lwip/lwip_utk.o
ifneq ($(filter 1,$(WIFI_STATIC) $(WIFI_DHCP)),)
LWIP_PORT_OBJS += ./mtkernel_3/lib/libnet/lwip/lwip_utk_ipv4.o
endif
ifeq ($(WIFI_UDP),1)
LWIP_PORT_OBJS += ./mtkernel_3/lib/libnet/lwip/lwip_utk_udp.o
endif
ifeq ($(WIFI_TCP),1)
LWIP_PORT_OBJS += ./mtkernel_3/lib/libnet/lwip/lwip_utk_tcp.o
endif
ifeq ($(WIFI_TCPBULK),1)
LWIP_PORT_OBJS += ./mtkernel_3/lib/libnet/lwip/lwip_utk_tcpbulk.o
endif
endif

WIFI_INCLUDES := -I"$(WIFIDIR)" \
                 -I"../lib/libtm/sysdepend/pico_rp2040/usb" \
                 -I"$(CYW43_DRIVER_PATH)/src" \
                 -I"$(CYW43_DRIVER_PATH)/firmware" \
                 -I"$(CYW43_PICO_PATH)/include" \
                 -I"$(PICO_SDK_PATH)/src/boards/include" \
                 -I"$(PICO_SDK_PATH)/src/common/pico_base_headers/include" \
                 -I"$(PICO_SDK_PATH)/src/common/hardware_claim/include" \
                 -I"$(PICO_SDK_PATH)/src/common/pico_binary_info/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2040/pico_platform/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2040/hardware_regs/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2040/hardware_structs/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_common/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_compiler/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_sections/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/pico_platform_panic/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_base/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_pio/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_gpio/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_dma/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_irq/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_sync/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_sync_spin_lock/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_clocks/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_resets/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/hardware_timer/include"

ifeq ($(WIFI_NETIF),1)
WIFI_INCLUDES += -I"$(LWIP_PORT_DIR)/include" \
                 -I"$(LWIP_PATH)/src/include" \
                 -I"$(PICO_SDK_PATH)/src/rp2_common/pico_lwip/include"
endif

CFLAGS += -DTM_WIFI_CYW43=1 -DTM_WIFI_JOIN=$(WIFI_JOIN) \
          -DTM_WIFI_NETIF=$(WIFI_NETIF) -DCYW43_LWIP=$(WIFI_NETIF) \
          -DTM_WIFI_STATIC=$(WIFI_STATIC) -DTM_WIFI_DHCP=$(WIFI_DHCP) \
          -DTM_WIFI_DNS=$(WIFI_DNS) -DTM_WIFI_UDP=$(WIFI_UDP) \
          -DTM_WIFI_TCP=$(WIFI_TCP) -DTM_WIFI_TCPBULK=$(WIFI_TCPBULK) \
          -DTM_WIFI_MQTT=$(WIFI_MQTT) \
          -DCYW43_ENABLE_BLUETOOTH=0 \
          -DCYW43_USE_OTP_MAC=1 -DPICO_CYW43_LOGGING_ENABLED=0 \
          -DPICO_RP2040=1 -DPICO_32BIT=1 -DPICO_ON_DEVICE=1 -DPICO_BUILD=1 \
          -DPICO_NO_HARDWARE=0 -DRASPBERRYPI_PICO_W=1 \
          -DCYW43_DEFAULT_PIN_WL_REG_ON=23u \
          -DCYW43_DEFAULT_PIN_WL_DATA_OUT=24u \
          -DCYW43_DEFAULT_PIN_WL_DATA_IN=24u \
          -DCYW43_DEFAULT_PIN_WL_HOST_WAKE=24u \
          -DCYW43_DEFAULT_PIN_WL_CLOCK=29u \
          -DCYW43_DEFAULT_PIN_WL_CS=25u \
          -DNDEBUG -ffunction-sections -fdata-sections $(WIFI_INCLUDES)
ifeq ($(WIFI_NETIF),1)
CFLAGS += -DCYW43_DEFAULT_IP_STA_ADDRESS=0 \
          -DCYW43_DEFAULT_IP_STA_GATEWAY=0 \
          -DCYW43_DEFAULT_IP_MASK=0 -DCYW43_DEFAULT_IP_DNS=0
endif
LFLAGS += -Wl,--gc-sections

OBJS += $(WIFI_LOCAL_OBJS) $(CYW43_OBJS) $(LWIP_OBJS) $(LWIP_PORT_OBJS)
C_DEPS += $(WIFI_LOCAL_OBJS:.o=.d) $(CYW43_OBJS:.o=.d) \
          $(LWIP_OBJS:.o=.d) $(LWIP_PORT_OBJS:.o=.d)

mtkernel_3/lib/libwifi/sysdepend/pico_rp2040/%.o: $(WIFIDIR)/%.c
	@echo 'Building Wi-Fi port file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/cyw43_ll.o: $(CYW43_DRIVER_PATH)/src/cyw43_ll.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/cyw43_ctrl.o: $(CYW43_DRIVER_PATH)/src/cyw43_ctrl.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/cyw43_stats.o: $(CYW43_DRIVER_PATH)/src/cyw43_stats.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/cyw43_lwip.o: $(CYW43_DRIVER_PATH)/src/cyw43_lwip.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/lwip/%.o: $(LWIP_PATH)/src/%.c
	@echo 'Building lwIP file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/lib/libnet/lwip/%.o: $(LWIP_PORT_DIR)/%.c
	@echo 'Building lwIP port file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/cyw43_bus_pio_spi.o: $(CYW43_PICO_PATH)/cyw43_bus_pio_spi.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/pio.o: $(PICO_SDK_PATH)/src/rp2_common/hardware_pio/pio.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/dma.o: $(PICO_SDK_PATH)/src/rp2_common/hardware_dma/dma.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

mtkernel_3/cyw43/gpio.o: $(PICO_SDK_PATH)/src/rp2_common/hardware_gpio/gpio.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

endif

include mtkernel_3/lib/libtm/sysdepend/pico_rp2040/subdir.mk
include mtkernel_3/lib/libtm/sysdepend/no_device/subdir.mk
include mtkernel_3/lib/libtk/sysdepend/cpu/rp2040/subdir.mk
include mtkernel_3/lib/libtk/sysdepend/cpu/core/armv6m/subdir.mk
include mtkernel_3/lib/libbsp/sysdepend/cpu/rp2040/subdir.mk
include mtkernel_3/kernel/sysdepend/pico_rp2040/subdir.mk
include mtkernel_3/kernel/sysdepend/cpu/rp2040/subdir.mk
include mtkernel_3/kernel/sysdepend/cpu/core/armv6m/subdir.mk

ifeq ($(PROJECT_APP),1)
PROJECT_DIR := ../../../src/buddy1
PROJECT_OBJS := ./mtkernel_3/buddy1/main.o ./mtkernel_3/buddy1/state.o \
                ./mtkernel_3/buddy1/protocol.o
ifeq ($(WIFI_MQTT),1)
PROJECT_OBJS += ./mtkernel_3/buddy1/mqtt_service.o
endif
OBJS += $(PROJECT_OBJS)
C_DEPS += $(PROJECT_OBJS:.o=.d)
CFLAGS += -I"$(PROJECT_DIR)"

mtkernel_3/buddy1/%.o: $(PROJECT_DIR)/%.c
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
endif
