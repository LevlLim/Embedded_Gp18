# Subsystem 2: Motion Control

**Owner:** Buddy 2: Ze Kai (2501273). Only the owner edits this folder; shared files (`app.h`, `app_main.c`, `msg_defs.*`, `fault.*`, `drivers/`, `template_patches/`) change by team agreement.

## Your files

| File | What it does |
|---|---|
| `subsystem2.h/.c` | init/start, the motion command queue, tuning constants |
| `task_motion.c` | 10 ms motion task: line PID, measured moves (forward / reverse / turns), stall and encoder checks, e-stop |
| `motor_control.h/.c` | the ONLY code that drives the motors: PWM, brake, duty slew limit, PID maths, mm/degree conversions |
| `subsystem2_test.c` | your car test |

You share `drivers/gpio_irq.*` (encoder interrupts) with Buddy 5 (echo pin).

## How to run your tests

**1. Host tests (on your laptop, no car needed).** These check the pure logic in your files.

```sh
cd testing/host
make test2
```

It ends with `=== X/X checks passed ===`; any `FAIL:` line names the check and the source line. `make` with no target runs every buddy's tests, so run it before you push to check you haven't broken anyone else.

**2. Car test (on the real hardware).** This runs only your `subsystem2_test.c`, with every task of the car running around it.

| Build with | Choose |
|---|---|
| CMake (Pico VS Code) | preset **"Self-test: Buddy 2 only"** (`test-b2`) |
| make | `make TEST_BUDDY=2 -j8` |

Flash `mtk3pico_smp1_usb_cdc_wifi_test_b2.uf2` and open the serial monitor. The firmware waits for the boot calibration to finish (`[S5] READY`), then prints `[TEST] === test run start: Buddy 2 only ===` and your `[TEST-S2]` lines. Look for `PASS` / `FAIL` on each.

## Host tests (`make test2`)

| Suite | Checks |
|---|---|
| motion command | every command type × argument × sequence packs and unpacks exactly |
| clamp, geometry, PID, slew | duty clamp; mm ↔ counts; PID turns TOWARD the line; integral cannot wind up; duty ramps by `S2_SLEW_PCT`; brake is instant |
| single-word records (shared) | line word and hump byte |

## Car test (`test_b2` image)

**Put the car on blocks: this test drives the wheels.**

| Line | Pass when |
|---|---|
| `100 mm = N counts, 90 deg = N counts` | the numbers look sensible for your wheels (see `S2_COUNTS_PER_REV`, `S2_WHEEL_DIAM_MM`) |
| `PASS: left encoder +N counts forward` / `right` | both wheels count UP while driving forward. `counts BACKWARDS` means swap that encoder's A/B wires; `no edges` means wiring or motor power. |
| `e-stop software path = N ms` | N ≤ 100 (FR11). This is software only: the logic-analyser capture (T5) is still owed. |
| `max IO_BANK0 dispatcher = N us` | a few µs (risk R8) |

## Still to test on the bench

These need the real track or instruments, so the self-test cannot do them:

- **Measure and set the PROVISIONAL geometry** in `subsystem2.h`: `S2_COUNTS_PER_REV` (turn a wheel exactly once by hand, read the count), `S2_WHEEL_DIAM_MM`, `S2_TRACK_WIDTH_MM`. Turns and distances depend on them.
- **Check a measured move:** send FORWARD 500 mm (or REVERSE / TURN 90°) and measure what the car actually does.
- **Tune line following** on the real track with `S2_KP_Q8`, `S2_KD_Q8`, `S2_KI_Q8`, `S2_SLEW_PCT`, `S2_TURN_DUTY_PCT` and `PROV_CRUISE_PCT` (in `app.h`).
- **T5:** logic analyser on GP8–GP11, trigger a fault, measure to both pins LOW.

## Settings you are likely to tune

| What | Where | Now |
|---|---|---|
| Steering strength | `S2_KP_Q8` | 120 |
| Duty change per 10 ms | `S2_SLEW_PCT` | 8 |
| Spin-turn / search power | `S2_TURN_DUTY_PCT` | 25 |
| Cruise speed | `PROV_CRUISE_PCT` in `app.h` | 40 |
| Stall: duty / timeout | `S2_STALL_DUTY_PCT` / `S2_STALL_TIMEOUT_MS` | 25 / 300 ms |
