# INF2004 Group AAI18: autonomous robot car on micro T-Kernel 3.0

Firmware for our line-following robot car: micro T-Kernel 3.0 running on
**both cores** of a Raspberry Pi Pico W (RP2040) on a Robo Pico board, with
a USB serial console, and telemetry and remote control over WiFi / MQTT.

The car follows a black line, reads Code 39 barcodes to choose its turn at
the next junction, stops for an obstacle, measures it with a scanning
ultrasonic sensor and drives around it, speeds up on the ramp, and stops
its motors when it is lifted. Who owns what (subsystems, pins, priorities,
interfaces) is in **[`TEAM.md`](TEAM.md)**.

You need the car (or a bare Pico W), a USB cable, and a computer running
macOS, Linux, or Windows. On Windows, either use **WSL (Ubuntu)** and run
every command below inside it, or use the CMake build in step 2.2b,
which needs no WSL.

---

## 1. Set up the Pico C SDK and the compiler

You need two separate things:

| What | Why | Where it comes from |
|---|---|---|
| **Pico C SDK** | USB, WiFi and lwIP (TCP/IP, MQTT) driver source code | `git clone` (step 1.2), or the copy in `pico-sdk-min/` for the CMake build |
| **Arm GNU Toolchain** (`arm-none-eabi-gcc`) | the compiler | a separate download (step 1.3) |

> ⚠️ **The SDK does NOT include the compiler.** You must install the compiler
> on its own and put its `bin` folder on your `PATH` (step 1.4).

**1.1 Install the basic tools: git, make and a host C++ compiler**

```sh
# macOS
xcode-select --install

# Linux / WSL (Ubuntu)
sudo apt update && sudo apt install -y git make g++
```

**1.2 Download the Pico C SDK** (any folder works; this guide uses `~/pico`)

```sh
mkdir -p ~/pico && cd ~/pico
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
```

The `submodule` line is required: it downloads the USB and WiFi drivers,
plus `lib/lwip` for our MQTT build. Only the CMake build (step 2.2b) can
skip this step: it uses `pico-sdk-min/`, the parts of SDK 2.2.0 it needs.

**1.3 Install the Arm GNU Toolchain (the compiler)**

- **macOS / Windows (WSL) / Linux:** download from
  <https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>.
  Pick the **AArch32 bare-metal target (arm-none-eabi)** package for your
  computer. For WSL, choose the **Linux x86_64** `.tar.xz`.
  Extract it, e.g. to `~/tools/arm-gnu-toolchain-<version>/`.
- **Linux / WSL shortcut:** `sudo apt install -y gcc-arm-none-eabi`
  installs it straight onto your PATH. If you use this, skip the PATH line in 1.4.
- **Windows without WSL:** install **Pico - Visual Studio Code**. It
  provides the compiler, CMake, Ninja and Python for step 2.2b.

**1.4 Tell your shell where the compiler and SDK are**

Add these two lines to the end of `~/.zshrc` (macOS) or `~/.bashrc` (Linux/WSL),
**changing both paths to match your computer**:

```sh
export PATH="$HOME/tools/arm-gnu-toolchain-15.3.rel1/bin:$PATH"   # folder that contains arm-none-eabi-gcc
export PICO_SDK_PATH="$HOME/pico/pico-sdk"                         # the SDK from step 1.2
```

Open a **new** terminal, then check both:

```sh
arm-none-eabi-gcc --version     # must print a version, not "command not found"
ls $PICO_SDK_PATH               # must list the SDK files
```

---

## 2. Clone this repo and build the `.uf2` file

**2.1 Clone the project**

```sh
cd ~/pico
git clone https://github.com/LevlLim/Embedded_Gp18.git
cd Embedded_Gp18/AAI18_firmware
```

For the MQTT build only: copy `config/wifi_secrets.example.h` to
`config/wifi_secrets.h` and fill in the WiFi name, password and the
broker's IP address. This file is git-ignored: **never commit it.**

**2.2a Build with make** (macOS, Linux, WSL)

```sh
cd build_make
make -j8
```

Your firmware is **`build_make/mtk3pico_smp1_usb_cdc_wifi.uf2`**. For WiFi
telemetry and remote control, build `make NET=lwip MQTT=1 -j8` instead
(→ `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`). Other builds are in
section 3.

If `make` stops with an error, it tells you which one is wrong:
`arm-none-eabi-gcc not found` means fix `PATH`, and
`PICO_SDK_PATH is not set` means fix `PICO_SDK_PATH` (step 1.4).

**2.2b Build with CMake** (Windows without WSL, or any computer)

Open **Pico - Visual Studio Code**, then **File → Open Folder… →
`AAI18_firmware`**. In the CMake side panel pick a **Configure preset**,
then click **Build** (F7):

| Preset | Builds | `.uf2` |
|---|---|---|
| `usb` | normal driving firmware | `build/usb/mtk3pico_smp1_usb_cdc_wifi.uf2` |
| `mqtt` | driving firmware + WiFi telemetry, log and remote control | `build/mqtt/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2` |
| `usb-test` | every subsystem's self-test | `build/usb-test/..._test.uf2` |
| `test-b1` … `test-b5` | one subsystem's self-test | `build/test-bN/..._test_bN.uf2` |

From a terminal: `cmake --preset mqtt`, then `cmake --build build/mqtt`.
The CMake build uses `pico-sdk-min/`, so it needs no `PICO_SDK_PATH`.

**2.3 Flash it onto the Pico W**

1. Hold the **BOOTSEL** button and plug the Pico W into USB, then release.
   A drive called **RPI-RP2** appears.
2. Copy the `.uf2` file onto that drive. On WSL, run `explorer.exe .` in
   `build_make` to open the folder in Windows. The Pico reboots by itself.

**2.4 See the output and drive the car**

Open the Pico's USB serial port at 115200 baud:

```sh
screen /dev/tty.usbmodem* 115200      # macOS   (quit: Ctrl-A then K)
screen /dev/ttyACM0 115200            # Linux
```

On Windows, use PuTTY or the VS Code *Serial Monitor* on the new COM port.
After the line sensors calibrate you see regular `[DBG]` status lines
(sensor readings, line error, motor duty, pitch / roll, encoder counts,
ultrasonic distance, mission state, fault word) and `[S3]`, `[S4]`,
`[S5]` event lines. Press the Robo Pico's **START** button (GP20) to
drive and **STOP** (GP21) to stop.

**Without a USB cable (MQTT build):** run a Mosquitto broker on a laptop
on the same 2.4 GHz network (`listener 1883 0.0.0.0` and
`allow_anonymous true` in its config file), then:

```sh
mosquitto_sub -h <laptop IP> -t car18/log                 # the console, over WiFi
mosquitto_pub -h <laptop IP> -t car18/cmd -m start        # or stop, reset
```

Binary telemetry (12 bytes, 5 Hz) is published on `car18/tele/#`. While
the car is parked, `fwd`, `back`, `left` and `right` make calibration moves.

**2.5 Where the code is**

Our code lives in **`app_program/`**. Every `.c` file there, and in its
subfolders, is compiled automatically.

- **`app_main.c` → `usermain()` is where the program starts.** It calls
  every subsystem's `subsystemN_init()` (create kernel objects), then every
  `subsystemN_start()` (start tasks).
- **One folder per student**, each with `subsystemN.h` (its public
  interface), `subsystemN_test.c` (its self-test) and `TESTING.md` (how to
  test it):

  | Folder | Subsystem | Owner | Tasks |
  |---|---|---|---|
  | `subsystem1/` | Telemetry & Comms | Levl Lim | `telemetry_task` (200 ms, core 1) |
  | `subsystem2/` | Motion Control | Ze Kai | `motion_task` (10 ms) |
  | `subsystem3/` | Line & Barcode Sensing | Zhi Kai | `line_task` (5 ms), `barcode_task` (1 ms) |
  | `subsystem4/` | IMU / Terrain | Jian Yan | `imu_task` (10 ms) |
  | `subsystem5/` | Vehicle Controller & Obstacle | Jun Hong | `controller_task` (event-driven), `obstacle_task` (60 ms) |

- `app.h` holds the definitions shared by the whole team: pin map, task
  priorities and periods.
- `msg_defs.h/.c` holds every packed message and field (Change Requirement
  1). `fault.h/.c` holds the 10-bit shared fault table.
- Drivers are in their owner's subsystem folder (`subsystem2/motor_control.c`,
  `subsystem3/adc_sampler.c`, `subsystem4/i2c0.c`). The exception is the
  GPIO interrupt dispatcher shared by S2 (encoders) and S5 (ultrasonic
  echo), which is in `app_program/drivers/`. Nothing was added under `device/`.

**Working as a team:** owners, pins, peripherals, priorities and the
interfaces between subsystems are in **`TEAM.md`**. On-car self-tests are
in each `subsystemN_test.c` (section 3: `TEST`, `TEST_BUDDY`). Host unit
tests are in **`testing/host/`**: `make` there runs all of them on a PC
with no robot, and `make test1` … `make test5` runs one subsystem's. Test
and demo videos are listed in **`videos/`**.

Pins: every pin is claimed in `TEAM.md`. **GP0/GP1 carry the ultrasonic
sensor**, so this project always uses the USB console and cannot build
with `CONSOLE=uart` or `CRASH_REPORT=1` (`app.h` refuses to compile).
GP23/GP24/GP25/GP29 are the Pico W radio and are never used.

After editing, build again (step 2.2a or 2.2b) and copy the new `.uf2`.

---

## 3. Build options: `SMP`, `CONSOLE`, `WIFI`, `TEST`, `TEST_BUDDY`, `CRASH_REPORT`, `NET`, `MQTT`

`make` on its own uses the defaults. To build differently, put options
after `make`. They can be combined in any order:

```sh
make -j8                          # defaults: SMP=1 CONSOLE=usb_cdc WIFI=cyw43 TEST=0 CRASH_REPORT=0
make NET=lwip MQTT=1 -j8          # WiFi telemetry, log and remote control (needs config/wifi_secrets.h)
make TEST=1 -j8                   # every subsystem's self-test (car on blocks)
make TEST_BUDDY=3 -j8             # only subsystem 3's self-test
```

| Option | Values (**default**) | What it does | Image name | C macro your code can test |
|---|---|---|---|---|
| `SMP` | `0`, **`1`** | `1` runs the kernel on both RP2040 cores; `0` uses core 0 only. Our task layout needs `1` (telemetry runs on core 1). | `smp1` / `smp0` | `TK_SUPPORT_SMP` |
| `CONSOLE` | **`usb_cdc`**, `uart` | where `tm_printf` output goes. **Only `usb_cdc` works here**: GP0/GP1 are the ultrasonic sensor. | `_usb_cdc` | `TM_CONSOLE_USB_CDC` |
| `WIFI` | **`cyw43`**, `none` | `cyw43` powers up the Pico W radio and its driver task | `_wifi` / *(nothing)* | `TM_WIFI_CYW43` |
| `TEST` | **`0`**, `1` | `1` compiles every `subsystemN_test.c` and runs them after the boot calibration | `_test` / *(nothing)* | `APP_TEST` |
| `TEST_BUDDY` | **`0`**, `1`–`5`, `6` | *added by us:* `1`–`5` runs only that subsystem's self-test (implies `TEST=1`); `6` is the ultrasonic bench test | `_test_bN` / `_test_us` | `APP_TEST_BUDDY` |
| `CRASH_REPORT` | **`0`**, `1` | copies the console to UART0 on GP0/GP1. **Refused in this project** (GP0/GP1 are the ultrasonic). | `_crash` | `TM_CONSOLE_UART` |
| `NET`, `MQTT` | *(off)*, `NET=lwip MQTT=1` | TCP/IP and the MQTT client inside the kernel's WiFi task. Needs the SDK's `lib/lwip` and `config/wifi_secrets.h`. | `_lwip_mqtt` | `TM_NET_LWIP`, `TM_MQTT` |

The options are part of the image name, e.g.
`mtk3pico_smp1_usb_cdc_wifi_test_b3.uf2`, so you can always tell which
build you are flashing. Switching options needs no `make clean`. All of
them are handled in **`build_make/pico_rp2040.mk`**. The CMake presets in
step 2.2b set the same options.

### Template files this project changes

We changed four of the template's shared files. Each change is marked with
an `AAI18` comment in the file:

| File | Change | Why |
|---|---|---|
| `config/config.h` | `CNF_TIMER_PERIOD` 10 → **1** ms | the 1 ms barcode and 5 ms line tasks cannot run on a 10 ms tick |
| `build_make/pico_rp2040.mk` | adds `TEST_BUDDY` | one self-test image per subsystem |
| `lib/liblwip/sysdepend/pico_rp2040/lwipopts.h` | `MQTT_OUTPUT_RINGBUF_SIZE` 256 → 1024 | room for the `car18/log` console lines |
| `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` | subscribes to `car18/cmd`, drains our telemetry outbox, passes commands to S1 | only the kernel's WiFi task may call lwIP / MQTT |

We also added `CMakeLists.txt`, `CMakePresets.json`, `cmake/` and
`pico-sdk-min/` for the CMake build (step 2.2b). The template's own README
is kept as `TEMPLATE_README.md`.

---

## 4. Change Requirement 1 and coding standard

**Change Requirement 1 (minimum-bit data):** every packed field, with its
bit width, range and saturation, is in `app_program/msg_defs.h`, checked at
compile time:

- motion command 2 B, barcode command 1 B, obstacle profile 5 B;
- telemetry frame 12 B, MQTT command 2 B `[seq][cmd]`, fault word 10 bits;
- line reading one 32-bit word, mission state 5 bits, controller state one
  bit-field struct (41 bits).

Data shared between tasks is either one word with a single writer or
passed through a kernel object. To run the host tests:
`cd testing/host && make`.

**BARR-C:2018:** all application code follows it, with these declared
deviations:

| ID | Rule | Deviation and reason |
|---|---|---|
| D2 | 5.1a | `ER`, `ID`, `UINT`… come from micro T-Kernel and cannot be renamed. |
| D3 | 1.1 | `//` comments are permitted; new code uses `/* */`. |
| D4 | — | The template's own files keep their tabs (including the patched `mqtt_utk.c`). |
| D5 | 6.3a | Function-like macros only where a compile-time constant is needed (register addresses, `APP_MBF_BUFSZ`, `MSG_STATIC_ASSERT`, the test `CHECK`). |
| D6 | 1.6a | Casts that reinterpret a bit pattern are commented; plain widening or narrowing of an already range-checked value is not. |

---

*micro T-Kernel 3.0 © Ken Sakamura / TRON Forum, distributed under the
T-License 2.2. This dual-core RP2040 port is derived from the official
μT-Kernel 3.0 BSP (`tron-forum/mtk3_bsp`, branch `pico_rp2040`).*
