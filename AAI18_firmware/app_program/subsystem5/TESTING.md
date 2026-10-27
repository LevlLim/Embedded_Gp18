# Subsystem 5: Obstacle + Vehicle Controller

**Owner:** Buddy 5: Jun Hong (2501666). Only the owner edits this folder; shared files (`app.h`, `app_main.c`, `msg_defs.*`, `fault.*`, `drivers/`, `template_patches/`) change by team agreement.

## Your files

| File | What it does |
|---|---|
| `subsystem5.h/.c` | init/start, servo and scan geometry, mission states |
| `obstacle_scan.c` | 60 ms ultrasonic task (TRIG GP0, ECHO GP1), servo, obstacle profile |
| `vehicle_controller.c` | the mission state machine: buttons, calibration at boot, line search, barcode actions, obstacle bypass, faults |
| `subsystem5_test.c` | your car test |

You also own the shared **`fault.h/.c`** (the 10-bit fault mask) and share `drivers/gpio_irq.*` with Buddy 2.

## How to run your tests

**1. Host tests (on your laptop, no car needed).** These check the pure logic in your files.

```sh
cd testing/host
make test5
```

It ends with `=== X/X checks passed ===`; any `FAIL:` line names the check and the source line. `make` with no target runs every buddy's tests, so run it before you push to check you haven't broken anyone else.

**2. Car test (on the real hardware).** This runs only your `subsystem5_test.c`, with every task of the car running around it.

| Build with | Choose |
|---|---|
| CMake (Pico VS Code) | preset **"Self-test: Buddy 5 only"** (`test-b5`) |
| make | `make TEST_BUDDY=5 -j8` |

Flash `mtk3pico_smp1_usb_cdc_wifi_test_b5.uf2` and open the serial monitor. The firmware waits for the boot calibration to finish (`[S5] READY`), then prints `[TEST] === test run start: Buddy 5 only ===` and your `[TEST-S5]` lines. Look for `PASS` / `FAIL` on each.

## Host tests (`make test5`)

| Suite | Checks |
|---|---|
| obstacle message | bearing range, boundaries, guards |
| vehicle state | parent + sub-state encoding |
| ranging and obstacle profile | echo µs → cm, side choice, profile builder |
| single-word records (shared) | line word and hump byte |

## Car test (`test_b5` image)

Car on the desk, nothing within 40 cm in front.

| Line | Pass when |
|---|---|
| `580 us echo = 10 cm` | ranging maths on the target |
| `PASS: 100/100 iterations, no bit lost` | fault mask (T4) |
| `PASS: WIFI_LOSS does not stop the car`, `PASS: COLLISION reads as E-STOP` | fault severities |
| `MOVE A HAND FROM ~40 cm TO ~10 cm IN FRONT (6 s)` | move your hand slowly in front of the ultrasonic |
| `PASS: ultrasonic saw near (<20 cm) and far (>30 cm)` | the sensor works at both ends |

## Still to test on the bench

These need the real track or instruments, so the self-test cannot do them:

- **Obstacle stop on the track:** drive at a box; the car must stop by 20 cm (`[S5] D_STOP: obstacle at … cm`) and wait, then carry on when it is removed.
- **Line search:** lift the car off the line while driving; it must find the line within 5 s or stop with LINE_LOST_HARD.
- **Buttons:** GP20 starts (and resets from FAULT), GP21 stops.
- **Servo sweep / bypass** is OFF (`S5_SERVO_FIXED 1`) until the sensor wiring can follow the servo; re-enable and test the bypass legs then.

## Settings you are likely to tune

| What | Where | Now |
|---|---|---|
| Servo fixed straight ahead | `S5_SERVO_FIXED` | 1 |
| Stop / detect distance | `PROV_D_STOP_CM` / `PROV_D_DETECT_CM` in `app.h` | 20 / 30 cm |
| Readings in a row to act | `S5_CONFIRM_PINGS` | 2 |
| Search step / lost time before search | `VC_SEARCH_STEP_DEG` / `VC_LOST_SEARCH_MS` in `vehicle_controller.c` | 20° / 600 ms |
