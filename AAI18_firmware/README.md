# INF2004 Group AAI18 — Autonomous Robot Car Firmware

Firmware for the Week 10 / Week 13 build, written against
[INF2004_ProjectTemplate](https://github.com/sirfonzie/INF2004_ProjectTemplate)
(micro T-Kernel 3.0 on the Raspberry Pi Pico W) and implementing
**Design Review V2**, including **Change Requirement 1 (minimum-bit data
representation)**.

> **Read this first.** This revision has been cross-compiled against the
> real template, but it has **not been run on hardware**. See
> [Status](#status-what-is-and-is-not-verified).

---

## Status: what is and is not verified

| | |
|---|---|
| ✅ **Cross-compiles against the real template** | `arm-none-eabi-gcc` 13.2, zero warnings, with the Pico SDK 2.2.0: the default USB build (± `TEST=1`) and `NET=lwip MQTT=1`, via both `make` and CMake (identical `.uf2` files). |
| ✅ **Host unit tests pass** | 90/90 checks, including 57 344 exhaustive motion-command round trips, saturation of every telemetry field, and the pure helpers of subsystems 3, 4 and 5. |
| ✅ **Strict warnings clean** | `-Wall -Wextra -Wconversion -Wsign-conversion -Wshadow` against the stub headers, `TEST=0` and `TEST=1`. |
| ❌ **Never run on hardware** | Every register offset, timing constant and threshold is from a datasheet or an estimate. |
| ❌ **No measurement anywhere** | Everything marked `PROVISIONAL` is a guess that unblocks implementation. |

**First bench session, in this order:** flash a `TEST=1` build with the
car on blocks → read the self-test output → confirm the ADC reports 0
overruns and the IMU answers on I²C → confirm the servo moves → *then* let
a wheel turn.

---

## Hardware note: ultrasonic on GP0/GP1

The HC-SR04P is wired to **GP0 (TRIG)** and **GP1 (ECHO)** — the pins the
template otherwise uses for its UART0 console. The template releases them
only in the USB-console build with `CRASH_REPORT=0`, so **the console is
always USB** in this project; `CONSOLE=uart` and `CRASH_REPORT=1` are
refused at compile time. (Design Review V2 §2 still lists GP6/GP7; update
the pin table there.)

## How to build and run

### 1. Host tests (no robot)

```sh
cd testing/host
make                 # every buddy's CR1 / pure-logic tests
make test3           # one buddy only (test1 .. test5)
make target-sizes    # sizeof() of the unpacked messages on the ARM target
```

### Testing your own part (per buddy)

Each buddy's folder has a **`TESTING.md`**: their files, the host tests,
the on-car test, the bench checks still owed, and the settings to tune.

| Buddy | Folder | Host | Car test image (CMake preset / make) |
|---|---|---|---|
| 1 | `subsystem1/` Telemetry | `make test1` | `test-b1` / `make TEST_BUDDY=1` |
| 2 | `subsystem2/` Motion | `make test2` | `test-b2` / `make TEST_BUDDY=2` (car on blocks) |
| 3 | `subsystem3/` Line + Barcode | `make test3` | `test-b3` / `make TEST_BUDDY=3` |
| 4 | `subsystem4/` IMU | `make test4` | `test-b4` / `make TEST_BUDDY=4` |
| 5 | `subsystem5/` Obstacle + Controller | `make test5` | `test-b5` / `make TEST_BUDDY=5` |

A `TEST_BUDDY=N` image is `mtk3pico_smp1_usb_cdc_wifi_test_bN.uf2`. The
whole car runs as normal; after the 4 s boot calibration only Buddy N's
`subsystemN_test()` runs. Some tests are hands-on and say so on the
console (slide the car across the line, tilt it, move a hand in front).
`TEST=1` still runs every buddy's test in one image.

### 2. Firmware

1. Install `make` and the Arm GNU toolchain (`arm-none-eabi-gcc`).
2. Clone the template, then copy this project over it — **both folders**:

   ```sh
   git clone https://github.com/sirfonzie/INF2004_ProjectTemplate.git
   rm -rf INF2004_ProjectTemplate/app_program
   cp -r app_program INF2004_ProjectTemplate/
   cp -r template_patches/. INF2004_ProjectTemplate/     # see below
   ```

3. Build (in `INF2004_ProjectTemplate/build_make`):

   | Build | Command | Needs |
   |---|---|---|
   | Normal (USB console) | `make -j8` | `PICO_SDK_PATH` set |
   | Self-tests, all buddies | add `TEST=1` | **car on blocks** |
   | Self-test, one buddy | add `TEST_BUDDY=1`..`5` | see that buddy's `TESTING.md` |
   | MQTT telemetry | `make NET=lwip MQTT=1 -j8` | SDK + `config/wifi_secrets.h` (copy the example) |

4. Hold **BOOTSEL**, plug in, copy the `.uf2` from `build_make/` to the drive.
5. Open the console at 115200 baud (`screen /dev/ttyACM0 115200`, or PuTTY).

Without `MQTT=1`, telemetry frames print to the console as hex, so bring-up
needs no broker.

### 3. Windows without WSL: CMake build (Pico – Visual Studio Code)

The **`AAI18_firmware` folder is ready to build as it is**: it already
contains the course template, this project's code with the template
patches applied, and `pico-sdk-min/` (the parts of Pico SDK 2.2.0 the
build uses: TinyUSB, the CYW43 Wi-Fi driver and lwIP). Nothing to clone.
It needs only CMake, Ninja, `arm-none-eabi-gcc` and Python, which the
"Pico - Visual Studio Code" program provides.

1. Open **Pico - Visual Studio Code** (the Start-menu shortcut, so the
   compiler is on PATH), then **File → Open Folder… → `AAI18_firmware`**.
2. In the CMake side panel, **Configure preset** → pick one:

   | Preset | Builds | UF2 |
   |---|---|---|
   | `usb` | normal driving firmware | `build\usb\mtk3pico_smp1_usb_cdc_wifi.uf2` |
   | `test-b1` … `test-b5` | one buddy's self-test | `build\test-bN\mtk3pico_smp1_usb_cdc_wifi_test_bN.uf2` |
   | `usb-test` | every buddy's self-test | `build\usb-test\…_test.uf2` |
   | `mqtt` | MQTT telemetry (needs `config\wifi_secrets.h`) | `build\mqtt\…_lwip_mqtt.uf2` |

3. Click **Build** (or press F7). The `.uf2` appears in `build\<preset>\`.

From a terminal in that folder: `cmake --preset test-b3` then
`cmake --build --preset test-b3`. The CMake and make builds give
byte-identical `.uf2` files. Using your own full SDK instead: set
`AAI18_PICO_SDK_PATH` in `CMakePresets.json`. There are no `uart` presets:
GP0/GP1 carry the ultrasonic sensor, so the UART console is refused.

### `template_patches/` — template files this project changes or adds

| File | Change | Why |
|---|---|---|
| `config/config.h` | `CNF_TIMER_PERIOD` 10 → **1** | The 1 ms barcode and 5 ms line tasks cannot exist on a 10 ms tick. `app.h` refuses to compile until this is applied. |
| `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` | Subscribes to `car18/cmd`; drains the telemetry outbox into `mqtt_publish()`; passes raw inbound bytes and link up/down to the application | lwIP is `NO_SYS` and owned by the radio task, so only that task may call it. Only used in `MQTT=1` builds. |
| `CMakeLists.txt`, `CMakePresets.json`, `cmake/` | **New.** CMake/Ninja build equal to the makefile, plus a Python `elf2uf2`; presets `test-b1`..`test-b5` | Build on Windows with no WSL and no host C++ compiler. |
| `build_make/pico_rp2040.mk` | Adds `TEST_BUDDY=0..5` (passes `APP_TEST_BUDDY`, names the image `_test_bN`) | One self-test image per buddy. |

---

## Layout

```
app_program/
  app.h                  pin map, priorities, periods, PROVISIONAL constants,
                         message-buffer sizing macro, periodic-task helpers
  app_main.c             usermain(), task + cyclic-handler creation,
                         which buddy's self-test runs (APP_TEST_BUDDY)
  msg_defs.h/.c          ★ CR1: every packed field, encode/decode  (Buddy 1)
  fault.h/.c             ★ CR1: the 10-bit shared fault bitmask    (Buddy 5)
  drivers/               shared by more than one buddy
    rp2040_regs.h        registers the template's BSP does not provide
    gpio_irq.h/.c        single IO_BANK0 dispatcher             (Buddy 2 + 5)
  subsystem1/  Buddy 1   Telemetry / Comms — WiFi, MQTT, remote cmds
  subsystem2/  Buddy 2   Motion Control — motors, encoders, PID, e-stop
  subsystem3/  Buddy 3   Line + Barcode — adc_sampler (sole ADC owner),
                         line sensor, barcode decode
  subsystem4/  Buddy 4   IMU / Terrain — i2c0 (sole I²C0 owner), hump,
                         tilt, collision
  subsystem5/  Buddy 5   Obstacle + Vehicle Controller — mission state
  (each subsystemN/ has subsystemN_test.c and TESTING.md)
template_patches/        template files this project changes or adds
testing/
  host/                  PC unit tests + target sizeof probe
  stub/                  fake kernel headers for host checks only
```

---

## Timing model

Every periodic task is woken by its own **cyclic handler**
(`app_start_periodic_task()`), and loops on `app_wait_period()`. Periods
therefore do not drift with execution time, and an overrun is reported
once instead of replayed. This replaces `tk_dly_tsk()` loops, which on the
template's 10 ms tick turned every "1/5/10 ms" task into ~20 ms.

Core 0 runs every driving task; core 1 runs Telemetry. The template's
WiFi/lwIP task is also on core 0, at priority 4 — **above** Motion Control —
so NFR6 depends on that task being short; measure it in T6.

Anything read across cores is either a single aligned word or protected
by a kernel object. `DI()` masks interrupts on the calling core only, so it
is never relied on for data Telemetry reads.

---

## Change Requirement 1: where it lives

| CR1 requirement | Where |
|---|---|
| Minimum bits per field | `msg_defs.h` (widths, ranges, static asserts) |
| 1. Motion control data | `task_motion.c`: mode, directions, duty, setpoint, sequence and encoder health in **one 27-bit control word** |
| 2. Line and barcode sensing | Line reading = **one 32-bit word** (7-bit error, valid, 24-bit ms stamp). ADC ring = **one word per sample** (12-bit value + 20-bit µs stamp). Barcode widths `uint16_t`. |
| 3. Obstacle detection | 5-byte profile; profiling sweep stored as 25 × `uint8_t` cm |
| 4. Hump estimation | Peak + SAT flag in **one byte** (`msg_pack_hump`) |
| 5. Vehicle / controller states | Mission state stored **only** as its 5-bit encoded byte |
| 6. Shared task / ISR data | Single-word records (no lock needed); Motion→Telemetry fields under a `TA_INHERIT` mutex; encoder total single-writer, differenced by the reader |
| 7. Inter-task messages | Motion 2 B, barcode 1 B, obstacle 5 B — buffers sized for the kernel's 4-byte header (`APP_MBF_BUFSZ`) |
| 8. MQTT payloads | 12-byte binary telemetry (schema v2), 2-byte fault word, 1-byte heartbeat, **2-byte binary commands** `[seq][cmd]` |
| Fault conditions | `fault.h` — 10 bits, severity masks, set/clear discipline |
| Race-free packed access | Single writer per word (rule R1); `tk_set_flg` ORs fault bits; requests to another task are flags it applies itself |
| Saturation (R5) | Every pack function clamps; every unpack rejects undefined codes |

### Measured figures

**Message size** (`make target-sizes` on `arm-none-eabi-gcc`, which uses 1-byte enums):

| Message | Unpacked `sizeof` | Packed | Kernel buffer slot, unpacked → packed |
|---|---|---|---|
| Motion command | 6 B | 2 B | 12 → 8 B |
| Barcode command | 3 B | 1 B | 8 → 8 B |
| Obstacle profile | 7 B | 5 B | 12 → 12 B |
| Telemetry frame | 18 B (struct) | 12 B | — |
| Line reading (shared record) | 12 B | 4 B | — |

Inside a µT-Kernel message buffer each message costs a 4-byte header plus
its body rounded up to 4 bytes, so packing the barcode and obstacle
messages saves no buffer RAM. Their gain is the bit-level validation, not
bytes. Report it that way.

**Application static RAM** (`arm-none-eabi-size` over `app_program` objects):
**2,488 B → 1,424 B** (data + bss), mostly from the ADC ring
(2,048 → 1,024 B). Task stacks (7 × 2,048 B) and kernel objects come from
the kernel heap and are not in that figure; count them separately.

**Still owed:** a true *before* build. The original code never ran, so the
honest baseline is a compile-time variant with plain structs, a `bool` per
fault and JSON telemetry, measured the same way. Per-task WCET and CPU
(NFR13, T6) need hardware.

### Fields deliberately NOT packed

Encoder counters, ECHO timestamps and intermediate PID / odometry / tilt
maths stay native 32-bit (Design Review §7.4): a free-running counter's
valid range *is* 32 bits, and packing them would buy a few bytes and pay
with torn reads in the 1 ms and 10 ms paths.

---

## Barr-C:2018 compliance

Scripted checks, all passing: 0 tabs, 0 CRLF, 0 lines over 80 columns,
braces on their own lines (1.3), one statement per line (3.3a), end-of-file
comments (3.3c), names of 3+ characters (7.1e), `b_`/`p_`/`h_` prefixes
(7.1k–n), no `continue` (1.7d), no function over 100 lines (6.2a), ISR named
`_isr` (6.5b), constants on the left of `==` (8.6a), `default` in every
`switch` (8.3b), fixed-width types (5.2), header guards (4.2b).

Globals: kernel object IDs are now `static` handles (`h_…`) private to the
file that uses them, reached through accessor functions, so no variable is
declared `extern` in a header (4.2c, 7.1j).

### Declared deviations

| ID | Rule | Deviation and reason |
|---|---|---|
| D2 | 5.1a | `ER`, `ID`, `UINT`… come from µT-Kernel and cannot be renamed. |
| D3 | 1.1 | `//` comments are permitted; new code uses `/* */`. |
| D4 | — | The template's own files use tabs; they were not reformatted (including the patched `mqtt_utk.c`). |
| D5 | 6.3a | Function-like macros are kept where a compile-time constant is needed: register addresses (`PADS_GPIO`, `PWM_SLICE_DIV`, …), `APP_MBF_BUFSZ`, `MSG_STATIC_ASSERT`, and `CHECK` (passes its call-site text). |
| D6 | 1.6a | Not every cast carries a comment. Casts that reinterpret a bit pattern are commented; plain widening/narrowing of an already range-checked value is not. |

(D1, the five over-length functions, is resolved: each was split.)

---

## Known weak points (be ready to be asked)

1. **Barcode encoding is a placeholder** (risk R5). `s3_decode_widths()`
   uses a 4-element scheme that must be replaced once the real symbols are
   measured. Sampling is now 500 S/s per channel (2 ms resolution).
2. **Hump height from accelerometer-only tilt** (A7). No gyroscope.
3. **D_detect, D_stop and all bypass geometry are guesses.** Leg
   distances come from `S5_ROBOT_*` and the profile; measure them.
4. **Servo may face away when an obstacle appears** (risk R6).
5. **Encoder ISR cost unknown** (risk R8); `gpio_irq_max_duration_us()`.
6. **Quadrature counts one channel only** (half resolution).
7. **`S2_COUNTS_PER_REV = 20` is a placeholder.**
8. **No closed-loop wheel-speed PID yet (NFR1).** Moves run at a fixed
   duty; the PID only steers in line following.
9. **MQTT path never run against a broker.** It compiles and links with
   the real lwIP; QoS 1 command delivery, reconnect timing (10 s, NFR6)
   and the outbox under load are untested.
10. **ADC is polled, not DMA.** Adequate at 1.5 kS/s; the design's DMA ring
    is the upgrade path if more resolution is needed.
11. **Status LED and buzzer are not implemented.**

---

## Test plan coverage

| Test | What | Where | Status |
|---|---|---|---|
| T2 | Packed field min/max/out-of-range, enum rejection | `testing/host` | ✅ automated |
| T3 | Obstacle profile round trip + profile builder | `testing/host` | ✅ automated (logic); bypass video owed |
| T4 | Fault bitmask | `subsystem5_test.c` | ⚠ sequential only — needs concurrent setters on both cores + an ISR |
| T5 | E-stop latency | `subsystem2_test.c` | ⚠ software path only; logic-analyser capture owed |
| T6 | CPU and deadlines | `app_wait_period()` returns missed periods | ⚠ instrumentation exists, not yet logged |
| T7 | Shared ADC integrity | `subsystem3_test.c` (overrun count must be 0) | on target |
| T8 | Stale line error | `msg_line_age_ms()` tested on host | ⚠ on-target suspend test not written |

---

## Ownership

Only the owner edits a subsystem folder. `app.h`, `app_main.c`,
`msg_defs.*`, `fault.*` and `template_patches/` are shared — change them by
agreement.

| # | Buddy | Owns |
|---|---|---|
| 1 | Levl Lim (2501118) | `subsystem1/`, **`msg_defs.*`**, `mqtt_utk.c` patch |
| 2 | Ze Kai (2501273) | `subsystem2/`, `drivers/gpio_irq.*` (shared with Buddy 5) |
| 3 | Zhi Kai (2500466) | `subsystem3/` (incl. `adc_sampler.*`) |
| 4 | Jian Yan (2500853) | `subsystem4/` (incl. `i2c0.*`) |
| 5 | Jun Hong (2501666) | `subsystem5/`, **`fault.*`** |
