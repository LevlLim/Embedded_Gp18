# Subsystem 4: IMU / Terrain

**Owner:** Buddy 4: Jian Yan (2500853). Only the owner edits this folder; shared files (`app.h`, `app_main.c`, `msg_defs.*`, `fault.*`, `drivers/`, `template_patches/`) change by team agreement.

## Your files

| File | What it does |
|---|---|
| `subsystem4.h/.c` | init/start, IMU registers, thresholds |
| `i2c0.h/.c` | the ONLY I²C0 code (GP16 SDA, GP17 SCL), polled master |
| `imu_terrain.c` | 10 ms IMU task: FIFO drain, calibration, tilt, hump sequence, collision |
| `subsystem4_test.c` | your car test |

## How to run your tests

**1. Host tests (on your laptop, no car needed).** These check the pure logic in your files.

```sh
cd testing/host
make test4
```

It ends with `=== X/X checks passed ===`; any `FAIL:` line names the check and the source line. `make` with no target runs every buddy's tests, so run it before you push to check you haven't broken anyone else.

**2. Car test (on the real hardware).** This runs only your `subsystem4_test.c`, with every task of the car running around it.

| Build with | Choose |
|---|---|
| CMake (Pico VS Code) | preset **"Self-test: Buddy 4 only"** (`test-b4`) |
| make | `make TEST_BUDDY=4 -j8` |

Flash `mtk3pico_smp1_usb_cdc_wifi_test_b4.uf2` and open the serial monitor. The firmware waits 4 s for the boot calibration, then prints `[TEST] === test run start: Buddy 4 only ===` and your `[TEST-S4]` lines. Look for `PASS` / `FAIL` on each.

## Host tests (`make test4`)

| Suite | Checks |
|---|---|
| pitch and hump height | atan pitch maths, height per distance, level adds nothing |
| single-word records (shared) | the 1-byte hump result |

## Car test (`test_b4` image)

Car flat and still at power-up (the IMU calibrates on boot).

| Line | Pass when |
|---|---|
| `PASS: IMU WHO_AM_I = 0x..` | the accelerometer answers on I²C (note the value: 0x33 for an LIS3DH / LSM303AGR-type part) |
| `i2c errors so far = 0` | no bus errors |
| `10 deg over 100 mm = ~17400 um`, `PASS: level chassis adds no height`, `pitch(176, 1000) = 10` | maths on the target |
| `LIFT THE NOSE, THEN THE TAIL (6 s)` | tilt the car by hand while it prints |
| `PASS: tilt range a..b deg` | the readings spread at least 20° |

Also check the boot lines `[S4] IMU WHO_AM_I = …` and no `[S4] … config writes failed`.

## Still to test on the bench

These need the real track or instruments, so the self-test cannot do them:

- **Hump:** drive over a real hump; check a hump result appears in telemetry and its height is about right.
- **Collision:** push the car into a soft obstacle while it drives; check `[S4] COLLISION: x … mg  y … mg` and set `S4_IMPACT_MILLI_G` from the numbers. Normal driving must NOT trip it.
- **Tilt fault:** tip the car past 30°; it must stop with `[S4] TILT`.

## Settings you are likely to tune

| What | Where | Now |
|---|---|---|
| Collision threshold / samples in a row | `S4_IMPACT_MILLI_G` / `S4_IMPACT_SAMPLES` | 2000 mg / 3 |
| Tilt fault angle | `S4_TILT_FAULT_DEG` | 30° |
| Hump start / level angle | `S4_HUMP_START_DEG` / `S4_HUMP_LEVEL_DEG` | 4° / 2° |
