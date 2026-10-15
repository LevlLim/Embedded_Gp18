# µT-Kernel 3.0 - RP2040 dual-core SMP Port

µT-Kernel 3.0 (IEEE 2050-2018) on the Raspberry Pi Pico W (RP2040, two Cortex-M0+ cores), derived from the official µT-Kernel 3.0 BSP Pico port. It builds from one source tree in two qualified configurations: **single core** (the default) and **dual-core SMP**, where both cores run the same kernel against a global ready queue serialized by a recursive big kernel lock built on RP2040 hardware spinlocks.

```sh
make          # single core  (default)
make SMP=1    # dual-core SMP
```

**Current capabilities**

- **Single core by default, SMP when you ask for it** - one `SMP` switch, one source tree; both configurations pass their own hardware gate (single core 11/11, SMP 58/58)
- **True SMP scheduling** - global ready queue with per-core dispatch, static task affinity, cross-core wake, inter-processor interrupts, and remote task management, all qualified on hardware
- **Full preemptive kernel** - tasks, priorities, semaphores, event flags, mutexes, mailboxes, message buffers, fixed and variable memory pools, cyclic and alarm handlers with per-processor assignment
- **Measured 1.799x speedup** - the same 384-job workload runs 44.4% faster with one worker per core than with both pinned to one
- **Two qualified console profiles** - UART0 (the baseline) and an optional SMP USB-CDC profile that survives a dual-core print storm with zero unintended drops
- **CYW43439 radio bring-up** - firmware boot, station enable, OTP MAC and active scan pass a 72/72 hardware gate as a development profile
- **Working IP networking** - DHCP, DNS and a repeating 64-packet UDP echo exchange, confirmed end to end against a Linux host on both one core and two

See [docs/PORT_RP2040.md](docs/PORT_RP2040.md) for exactly what is validated on hardware, what is ported but not yet validated, and what is not ported.

> [!IMPORTANT]
> The qualified release boundary is **SMP + UART**, plus the optional SMP
> USB-CDC console. IP networking and arbitrary concurrent peripheral access
> are development profiles, not part of that boundary.
> [docs/PORT_RP2040.md](docs/PORT_RP2040.md) states the supported scope and
> the RP2040 safety rules.

---

## Hardware

- Raspberry Pi **Pico W** (RP2040, dual Cortex-M0+, 2 MB flash)
- USB cable (data, not charge-only)
- A USB-serial adapter for the UART console - GP0 TX, GP1 RX, 115200 8N1
- An **external LED on GP16** for the liveness indicator. The Pico W's on-board LED is wired to the CYW43439 radio rather than to an RP2040 pin, so it is unavailable in the radio-down qualified profiles. GP23, GP24, GP25 and GP29 are reserved by the radio.

---

## Quick start

```sh
git clone https://github.com/sirfonzie/mtk3smp-rp2040.git
cd mtk3smp-rp2040

# build -- single core by default
cd build_make
make -j8
```

Hold **BOOTSEL** while connecting the Pico, then copy
`build_make/mtk3pico_smp0_uart.uf2` to the `RPI-RP2` drive that appears.

To build the dual-core SMP kernel instead, add `SMP=1` -- see
[Building for one core or two](#building-for-one-core-or-two).

Requires `arm-none-eabi-gcc` (baseline: 13.2.1) and a host `g++`. The build
compiles the ELF-to-UF2 converter itself on first run - the RP2040 bootloader
accepts UF2, not ELF, and unlike a Pico SDK/CMake project nothing else
provides that helper here. See
[docs/GETTING_STARTED.md](docs/GETTING_STARTED.md) for a full step-by-step
walkthrough.

---

## Documentation

| Doc | Covers |
|---|---|
| [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md) | Step-by-step toolchain setup, build, flash, console, troubleshooting |
| [docs/PORT_RP2040.md](docs/PORT_RP2040.md) | Feature status: what is validated on hardware, what is ported but not validated, and what is not ported. Also carries the RP2040 safety rules, the driver-ownership matrix, and the idle/exception model |

---

## Building for one core or two

**The build is single core by default.** `SMP` is the switch, and it is the
only thing that decides how many RP2040 cores the kernel uses:

| You want | Command | Produces |
|---|---|---|
| **One core** (default) | `make -j8` | `mtk3pico_smp0_uart.uf2` |
| **Two cores** (SMP) | `make SMP=1 -j8` | `mtk3pico_smp1_uart.uf2` |

`make` with no arguments is exactly `make SMP=0`. Core 1 is never started, the
SMP scheduler is compiled out, and the kernel behaves as the stock
single-core µT-Kernel 3.0 Pico BSP. This is the conservative choice and the
one to use if you are unsure.

`SMP=1` starts the second core, enables the global ready queue, the recursive
big kernel lock and the IPI transport, and lets tasks be assigned to either
processor. Both configurations are qualified — Phase 12 passed **58/58** in
SMP and **11/11** in the single-core regression — and both are built from the
same source tree.

Switching between profiles needs no manual cleanup: the build stamps a
profile id into `build_make/.build_profile` and discards objects built for a
different one automatically, reporting
`Build profile is now ...; discarding objects built for another profile.`

### Choosing the console

`CONSOLE` is independent of `SMP`:

| Console | Command | Notes |
|---|---|---|
| UART0 (default) | `make SMP=1 -j8` | GP0 TX / GP1 RX, 115200 8N1. Needs a USB-serial adapter. No external dependencies. |
| USB-CDC | `make SMP=1 CONSOLE=usb_cdc PICO_SDK_PATH=... -j8` | Console over the Pico's own USB port. Qualified for SMP at 71/71. **Requires the Pico SDK** - it links TinyUSB from it. |

The USB console is the only part of this tree that needs the Pico SDK, and it
uses just the TinyUSB library, not the SDK's CMake build. Point
`PICO_SDK_PATH` at your checkout, or export it once. Without it the build
stops with `TinyUSB not found at <path>. Set PICO_SDK_PATH, or build with
CONSOLE=uart`.

### All profiles at a glance

| Profile | Command | Status |
|---|---|---|
| Single core, UART | `make -j8` | **Qualified** — the default, 11/11 regression gate |
| Dual core, UART | `make SMP=1 -j8` | **Qualified** — the SMP baseline, 58/58 |
| Dual core, USB-CDC | `make SMP=1 CONSOLE=usb_cdc -j8` | **Qualified** — optional, 71/71 |
| Radio boot + active scan | `make SMP=1 WIFI=cyw43 -j8` | Development — 72/72 hardware gate, not in the release |

The artifact name always follows the profile:
`mtk3pico_smp<SMP>_<CONSOLE>[<wifi>].uf2`.

### Networking (development only)

Networking builds on the radio profile through additional knobs —
`WIFI_JOIN`, `WIFI_NETIF`, `WIFI_STATIC`, `WIFI_DHCP`, `WIFI_DNS`,
`WIFI_UDP`, `WIFI_TCP`, `WIFI_TCPBULK`. Each builds on the one before it
(`WIFI_UDP` needs `WIFI_DNS`, `WIFI_TCP` needs `WIFI_UDP`). These read local
configuration headers that are deliberately **not** committed; copy the
matching `config/*.example.h` and edit it:

```sh
cp config/wifi_credentials.example.h config/wifi_credentials.h
cp config/network_config.example.h   config/network_config.h
# fill in the values, and set each file's *_SET flag from 0 to 1
make SMP=1 WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 -j8
```

Each template guards itself: an unedited copy fails at compile time with a
message naming the file and the flag, rather than building and then failing
to associate at runtime.

`config/wifi_credentials.h`, `network_config.h`, `udp_test_config.h`,
and `tcp_test_config.h` are all gitignored. Never commit
a populated copy.

## What the demo does (what you see on first flash)

A small **producer/consumer pipeline** across four tasks, putting one pass
through every IPC primitive the kernel offers. Radio builds add a fifth task
that reports WiFi status and the UDP echo traffic:

```
producer  --[fixed memory pool]--> record
          --[mutex]-------------> shared sequence counter
          --[message buffer]----> consumer
consumer  --[semaphore]---------> credit returned to producer
          --[event flag]--------> monitor woken
monitor   prints a status line
blink     LED liveness, independent of all of the above
```

Under SMP the producer is pinned to processor 1 and the consumer to processor
2 with `TA_ASSPRC`, so every message crosses a core boundary:

```
=== uT-Kernel 3.0 / RP2040 demo ===
SMP build: 2 processors
producer -> [mbf] -> consumer, 2 credits, 500 ms period

[monitor] seq=6 produced=6 consumed=6 dropped=0  made_on=prc1 seen_on=prc2
```

Built with `SMP=0` the same source runs unchanged on one core and both read
`prc1`. The blink task on GP16 is deliberately independent of the console: if
output stops but the LED still blinks, the console is at fault; if the LED
freezes, the kernel is.

It is one file with the tunables at the top - meant to be read, changed and
reflashed. **Full walkthrough**, including which call demonstrates which
object and what to try changing:
[docs/GETTING_STARTED.md § 6b](docs/GETTING_STARTED.md#6b-what-the-demo-does-and-what-you-should-see).

---

## Performance numbers

The measured qualification results - wake/dispatch latency, cross-core wake,
lock contention and hold times, idle stack high water, and the Phase 13
scaling benchmark - are recorded in
[docs/PORT_RP2040.md](docs/PORT_RP2040.md), with the date and profile of each
run.

**The phase harnesses that produced them, and the per-phase hardware captures,
are not part of this release.** They are held back with the numbers being
prepared for publication, and will be published alongside them so the results
are reproducible from this repository.

The figures are observed measurements and broad bounded-response gates, **not
hard-real-time guarantees**. The recursive big kernel lock masks local
interrupts while held, so workload-specific worst-case latency still requires
application-level analysis.

---

## Project structure

```
kernel/            µT-Kernel 3.0 core, with the SMP scheduler and per-core state
include/           Kernel and BSP headers; sysdepend/pico_rp2040/ holds board config
lib/               libtk / libtm support libraries
device/            Device manager and inherited peripheral drivers
config/            Kernel configuration; *.example.h templates for local secrets
build_make/        GNU make build; pico_rp2040.mk is the Pico target
etc/linker/pico_rp2040/
                   Linker script
tools/elf2uf2/     UF2 packaging tool, built from source
app_program/       The demo: the tasks, the kernel objects, and usermain()
docs/              Getting started and port feature status
```

---

## Testing and Bug Reports

This port is qualified by phase gates run on real Pico W hardware. The
qualified boundary is SMP + UART (Phase 12: SMP 58/58, single-core 11/11),
plus the optional SMP USB-CDC console (Phase 11: 71/71). Phase 13 adds an
application-level scaling demonstration at 65/65.

**Soak testing note:** the longest recorded soak is **ten seconds** - 501
ownership audits and 3,785,087 core-1 lock operations with zero invariant
failures. Behaviour over hours or days has not been characterised. The
qualified profiles should be considered well-gated but not long-run proven.

Embedded software depends heavily on board revision, toolchain version, and
usage patterns that may not have been covered. If you hit unexpected
behaviour, a crash, or a reproducible failure, please open an issue with what
happened, how to reproduce it, the console output at the point of failure,
your `arm-none-eabi-gcc` version, and your board.

**Where help is most useful:** [docs/PORT_RP2040.md](docs/PORT_RP2040.md)
lists what is validated and what is not. The "Ported, not validated" entries -
IP networking over CYW43439, the inherited peripheral drivers
under concurrent cross-core use, and anything long-running - are where an
extra pair of hands would make the most difference. A result is worth having
whether it worked or not.

---

## How the port works

The RP2040 has two Cortex-M0+ cores and no cache coherency problem to solve,
but also no load-linked/store-conditional: atomicity comes from the hardware
spinlock block. This port builds a **recursive big kernel lock** on those
spinlocks and serializes all kernel state behind it, so both cores run the
same kernel image against one global ready queue.

Core 0 boots normally and is the sole SysTick timekeeper; core 1 is brought up
afterwards and explicitly disables its local SysTick, with the common handler
rejecting and counting any tick that arrives off core 0 anyway. Cross-core
work - waking a task resident on the other core, delivering a cyclic or alarm
callback to its assigned processor, remote task management - travels by
inter-processor interrupt, with reasons coalesced so a burst costs one IPI.

Peripheral IRQs are permanently owned by processor 1 (RP2040 core 0). The
inherited mSDI driver layer was converted from local interrupt masking to
SMP-aware serialization: an atomic `FastLock` counter, interrupt-preserving
cross-core locks around each task/IRQ handoff, and auto-cleared event flags in
place of sleep/wakeup pairs so a timeout/completion race cannot strand a stale
wakeup.

### What SMP support cost, in source terms

The SMP conversion changes **36 files** relative to the upstream Pico BSP
(`tron-forum/mtk3_bsp`, branch `pico_rp2040`, commit `15ed232`) — 11 added and
25 modified. That count excludes documentation, the qualification
applications, USB/Wi-Fi work, and unrelated Pico W baseline fixes.

**Added** — the SMP interfaces (`include/tk/smp.h`, `kernel/knlinc/smp.h`,
`kernel/knlinc/smp_lock.h`), the RP2040 backend
(`kernel/sysdepend/cpu/rp2040/smp_atomic.c`, `smp_rp2040.c`), and the generic
scheduler and locking (`kernel/tkernel/ready_queue_smp.c`, `smp_dispatch.c`,
`smp_lock.c`).

**Modified** — configuration to select `TK_SUPPORT_SMP` and `TK_MAX_CORE`; the
ARMv6-M context-switch, exception and dispatch paths to make saved task
context processor-aware; board init and idle handling to start core 1 and give
each core a private Thread-mode idle context; and the generic kernel
(`task.c`, `task_manage.c`, `task_sync.c`, `wait.c`, `timer.c`,
`time_calls.c`, `klock.c`, `cpuctl.c`) for affinity, remote task management,
cross-core wakeups and single-owner timekeeping.

The generic scheduler and locking policy is largely reusable. Atomics,
interrupts, secondary-core startup, context switching and the IPI transport
are target-specific — `kernel/sysdepend/cpu/rp2040/` is the principal
replacement point for another SoC.

Separately, the inherited peripheral drivers (`device/ser/`, `device/i2c/`,
`device/adc/`, `lib/libtk/fastlock.c`) were made SMP-safe. That work is not
required for the core SMP kernel to boot.

---

## About µT-Kernel 3.0

µT-Kernel 3.0 is a real-time OS for small-scale embedded systems and IoT edge
nodes, developed by TRON Forum.

- Compliant with IEEE Standard 2050-2018; highly compatible with µT-Kernel 2.0
- Released as open source under T-License 2.2

Specification: [tron-forum.github.io/mtkernel_3](https://tron-forum.github.io/mtkernel_3/)
Upstream BSP: [github.com/tron-forum/mtk3_bsp](https://github.com/tron-forum/mtk3_bsp), branch `pico_rp2040`, tag `v1.00.00.B5-pico_rp2040`
TRON Forum: [www.tron.org](https://www.tron.org)

This port is derived from that tag, which is retained as the `upstream` remote
and as the root commit of this repository; the original BSP README is
preserved there.

---

## Author

Muhamed Fauzi Bin Abbas, with AI Assistants.

---

## Licensing

This repository combines upstream µT-Kernel 3.0 source with port work written
for the RP2040. Nothing here is relicensed: upstream copyright headers are
left intact in every file, and where this section and a file header disagree,
**the file header governs**. Full licence texts are in [`LICENSES/`](LICENSES/).

| Part of the repository | Copyright | Licence |
|---|---|---|
| `kernel/`, `include/`, `lib/libtk/`, `lib/libtm/` (core), `device/`, `config/` - µT-Kernel 3.0 and this port's SMP additions to it: per-core kernel state, the global ready queue, the recursive big kernel lock, cross-core dispatch, and the RP2040 SMP backend | µT-Kernel 3.0, © 2006-2023 Ken Sakamura, released by TRON Forum | T-License 2.2 - [`T-License-2.2_TEF000-219-200401.pdf`](LICENSES/T-License-2.2_TEF000-219-200401.pdf) |
| `lib/libnet/lwip/`, `lib/libwifi/`, `lib/libtm/sysdepend/pico_rp2040/usb/`, `app_program/` - the lwIP raw-API driver, the CYW43439 polling service, the TinyUSB console glue, and the demo application. Built against third-party libraries, but not derived from them | © 2026 Muhamed Fauzi Bin Abbas | Apache-2.0 - [`Apache-2.0.txt`](LICENSES/Apache-2.0.txt) |
| `lib/libwifi/sysdepend/pico_rp2040/cyw43_bus_pio_spi.pio.h`, `tools/elf2uf2/` - generated from, or copied from, Pico SDK sources | © 2020 Raspberry Pi (Trading) Ltd. | BSD-3-Clause - [`BSD-3-Clause_Raspberry-Pi.txt`](LICENSES/BSD-3-Clause_Raspberry-Pi.txt) |

### A note on T-License 2.1

TRON Forum releases µT-Kernel 3.0 as a whole under **T-License 2.2**, and the
upstream repository names `TEF000-219-200401.pdf` as the governing document.

64 files still carry "T-License 2.1" in their headers, predating the relicense
- 36 of them in `kernel/tkernel/`, the rest spread across `kernel/knlinc/`,
`kernel/tstdlib/`, `kernel/usermain/`, `kernel/sysdepend/`, `include/sys/`,
`include/tk/`, `include/tm/` and `lib/libtk/`. T-License 2.1
([`T-License-2.1_TEF000-218-150401.pdf`](LICENSES/T-License-2.1_TEF000-218-150401.pdf))
is included in `LICENSES/` so those headers resolve to a text present in this
repository. This is a record of what the files say, not a claim that any part
of µT-Kernel 3.0 is offered under 2.1 rather than 2.2.

Every file added by this port carries a 2.2 header, an Apache-2.0 SPDX tag, or
the original Raspberry Pi BSD-3-Clause notice.

### Build dependencies (not redistributed here)

**The `arm-none-eabi` GCC toolchain** is required to build and is not included.

**The Raspberry Pi Pico SDK** is required only for the `CONSOLE=usb_cdc` and
`WIFI=cyw43` profiles, and is not included. Those profiles compile sources
from three of its submodules, each under its own licence and obtained with the
SDK rather than from here:

| Component | Upstream | Licence |
|---|---|---|
| TinyUSB | `hathach/tinyusb` | MIT, © 2018 hathach (tinyusb.org) |
| lwIP | `lwip-tcpip/lwip` | BSD-3-Clause, © 2001-2002 Swedish Institute of Computer Science |
| cyw43-driver | `georgerobotics/cyw43-driver` | © 2019-2022 George Robotics Pty Ltd. **Hardware-restricted** - see below |

> [!IMPORTANT]
> The **cyw43-driver licence is not a plain BSD licence.** Raspberry Pi Ltd
> grants use "solely with the Licensor's microcontroller chip (RP2040) or any
> other semiconductor device produced by the Licensor. No other use is
> permitted." Its redistribution clause carries the same restriction. That
> covers this port, which is RP2040-only by definition, but it does mean the
> `WIFI=cyw43` profile cannot be carried to non-Raspberry-Pi silicon. The
> kernel, the qualified UART and USB-CDC profiles, and everything under
> `kernel/`, `include/`, `lib/libtk/` and `lib/libtm/` are unaffected.

**CYW43439 radio firmware** is distributed as a binary blob inside the Pico
SDK under Infineon's own terms. It is not redistributed here.

### Sources of the licence texts

| Licence | Obtained from |
|---|---|
| T-License 2.1 and 2.2 | TRON Forum, distributed with the upstream µT-Kernel 3.0 release |
| Apache-2.0 | The canonical text from apache.org |
| BSD-3-Clause (Raspberry Pi) | `LICENSE.TXT` from the Pico SDK repository |
