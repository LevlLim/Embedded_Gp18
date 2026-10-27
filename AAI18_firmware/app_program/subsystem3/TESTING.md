# Subsystem 3: Line + Barcode

**Owner:** Buddy 3: Zhi Kai (2500466). Only the owner edits this folder; shared files (`app.h`, `app_main.c`, `msg_defs.*`, `fault.*`, `drivers/`, `template_patches/`) change by team agreement.

## Your files

| File | What it does |
|---|---|
| `subsystem3.h/.c` | init/start, thresholds and sensor position settings |
| `adc_sampler.h/.c` | the ONLY code that touches the ADC: round-robin GP26/27/28 at 1.5 kS/s, polled every 1 ms |
| `line_sensor.c` | 5 ms line task, calibration (swept or static), steering error |
| `barcode_decoder.c` | 1 ms barcode task, bar widths, decode |
| `subsystem3_test.c` | your car test |

## How to run your tests

**1. Host tests (on your laptop, no car needed).** These check the pure logic in your files.

```sh
cd testing/host
make test3
```

It ends with `=== X/X checks passed ===`; any `FAIL:` line names the check and the source line. `make` with no target runs every buddy's tests, so run it before you push to check you haven't broken anyone else.

**2. Car test (on the real hardware).** This runs only your `subsystem3_test.c`, with every task of the car running around it.

| Build with | Choose |
|---|---|
| CMake (Pico VS Code) | preset **"Self-test: Buddy 3 only"** (`test-b3`) |
| make | `make TEST_BUDDY=3 -j8` |

Flash `mtk3pico_smp1_usb_cdc_wifi_test_b3.uf2` and open the serial monitor. The firmware waits 4 s for the boot calibration, then prints `[TEST] === test run start: Buddy 3 only ===` and your `[TEST-S3]` lines. Look for `PASS` / `FAIL` on each.

## Host tests (`make test3`)

| Suite | Checks |
|---|---|
| barcode message | saturation and guards on the 1-byte barcode message |
| barcode width classification | narrow/wide decode from synthetic widths, noise rejected |
| single-word records (shared) | the line word you publish to Motion Control |

## Car test (`test_b3` image)

Car on the desk **on the track**, USB connected.

| Line | Pass when |
|---|---|
| `[ADC] samples=N overruns=0` | samples climbing, **0 overruns** (T7) |
| `line err=… valid=… age=N ms` | age a few ms (the reading is fresh) |
| `SLIDE THE CAR SLOWLY ACROSS THE LINE (6 s)` | slide the car sideways back and forth over the line while it prints |
| `PASS: side sensor saw black and white` / `centre` | each sensor went above AND below its threshold. A FAIL means that sensor's height, wiring or threshold is wrong. |
| `N,N,N,W: PASS` … | synthetic barcode decodes |

Tip: during the 3 s boot calibration, slide the car across the line too, so both sensors get a measured threshold (`… swept` in the `[S3] thresholds:` line).

## Still to test on the bench

These need the real track or instruments, so the self-test cannot do them:

- **Sensor height and spacing:** side sensor 12–15 mm from the centre one (never more than 18 mm, the line width), both the same height.
- **Barcode sensor (GP28):** mount it over the bars (they start 20 mm beyond the line edge) and drive over a real barcode; check the console prints `[S5] barcode cmd N conf N` with the right command, and tune the PROVISIONAL `S3_QUIET_ZONE_US`, `S3_MIN_BAR_US`, `S3_WIDE_RATIO_Q8` at your speed.
- **Lighting (assumption A2):** log readings in the bright and shadowed parts of the track.

## Settings you are likely to tune

| What | Where | Now |
|---|---|---|
| Side sensor left/right of centre | `S3_SIDE_SENSOR_RIGHT` | 1 (right) |
| Static side threshold (white +) | `S3_SIDE_STATIC_OFFSET` | 400 |
| Minimum contrast for a sweep | `S3_MIN_SWEEP_CONTRAST` | 120 |
| Line-lost debounce | `S3_LOST_DEBOUNCE` | 4 × 5 ms |
