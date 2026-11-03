# Subsystem 3: Line + Barcode

**Owner:** Buddy 3: Zhi Kai (2500466). Only the owner edits this folder; shared files (`app.h`, `app_main.c`, `msg_defs.*`, `fault.*`, `drivers/`, `template_patches/`) change by team agreement.

## Your files

| File | What it does |
|---|---|
| `subsystem3.h/.c` | init/start, thresholds and sensor position settings |
| `adc_sampler.h/.c` | the ONLY code that touches the ADC: round-robin GP26/27/28 at 1.5 kS/s, polled every 1 ms |
| `line_sensor.c` | 5 ms line task, calibration (swept or static), steering error, junction detection |
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

Flash `mtk3pico_smp1_usb_cdc_wifi_test_b3.uf2` and open the serial monitor. The firmware waits for the boot calibration to finish (`[S5] READY`), then prints `[TEST] === test run start: Buddy 3 only ===` and your `[TEST-S3]` lines. Look for `PASS` / `FAIL` on each.

## Host tests (`make test3`)

| Suite | Checks |
|---|---|
| barcode message | saturation and guards on the 1-byte barcode message |
| barcode width classification | narrow/wide decode from synthetic widths, noise rejected |
| single-word records (shared) | the line word you publish to Motion Control |

## Car test (`test_b3` image)

Car on the desk **on the track** with the line **between** the two sensors, USB connected.

| Line | Pass when |
|---|---|
| `[ADC] samples=N overruns=0` | samples climbing, **0 overruns** (T7) |
| `line err=… valid=… age=N ms` | age a few ms (the reading is fresh) |
| `SLIDE THE CAR SLOWLY ACROSS THE LINE (6 s)` | slide the car sideways back and forth over the line while it prints |
| `PASS: LEFT sensor saw black and white` / `RIGHT` | each sensor went above AND below its threshold. A FAIL means that sensor's height, wiring or threshold is wrong. |
| `N,N,N,W: PASS` … | synthetic barcode decodes |

Tip: during the 3 s boot calibration, slide the car side to side across the line so both sensors see it (`swept` in the `[S3] thresholds:` line), and **end with the line between the sensors**. Ending with a sensor on black fails calibration.

## How the two sensors steer (straddle layout)

| Left | Right | Meaning | Car does |
|---|---|---|---|
| W | W | centred, or the line slipped out | steers gently (err ±12) toward the sensor that saw the line last: a line in the middle drifts to the other sensor, a line outside comes back |
| B | W | drifted right | steers left (err −20…−50) |
| W | B | drifted left | steers right (err +20…+50) |
| B | B | junction | straight; `[S5] junction N: …` and the stored barcode turn is done |
| W | W after one side was B for > 0.4 s | line went out the far side | LOST: turns toward that side until a sensor sees the line |
| W | W for > 1.5 s | lost | LOST: turns toward the last side; the Controller searches |

## Still to test on the bench

These need the real track or instruments, so the self-test cannot do them:

- **Sensor spacing:** the line is 18 mm wide. Put the sensors **22–26 mm apart, centre to centre** (each 2–4 mm outside a line edge), at the same height. Both must read W with the car centred and B when moved 5 mm toward them.
- **Junctions:** drive over a cross or T: the console prints `[S5] junction N: …` once per junction (`jct N` in the `[DBG]` line counts them). A branch on one side only is not detected.
- **Axle distance:** measure from the sensors to the wheel axle and set `S5_SENSOR_TO_AXLE_MM` (subsystem5.h), so the car turns about the junction centre.
- **Barcode sensor (GP28):** mount it over the bars (they start 20 mm beyond the line edge) and drive over a real barcode; check the console prints `[S5] barcode cmd N conf N` with the right command, and tune the PROVISIONAL `S3_QUIET_ZONE_US`, `S3_MIN_BAR_US`, `S3_WIDE_RATIO_Q8` at your speed.
- **Lighting (assumption A2):** log readings in the bright and shadowed parts of the track.

## Settings you are likely to tune

| What | Where | Now |
|---|---|---|
| Which pin is left (0 = GP26 left, GP27 right) | `S3_SWAP_SIDES` | 0 |
| Static threshold (white +) | `S3_STATIC_OFFSET` | 400 |
| Minimum contrast for a sweep | `S3_MIN_SWEEP_CONTRAST` | 400 |
| Steering strength, one sensor on black | `S3_ERR_MIN` / `S3_ERR_MAX` | 20 / 50 |
| Dark spell that means "lost outward" | `S3_OUTWARD_MS` | 400 ms |
| Gentle steer while both white | `S3_SEEK_ERROR` | 12 |
| All white this long = lost | `S3_WHITE_LOST_MS` | 1500 ms |
| Both-black samples for a junction | `S3_JUNCTION_SAMPLES` | 3 × 5 ms |
| Line-lost debounce | `S3_LOST_DEBOUNCE` | 4 × 5 ms |
