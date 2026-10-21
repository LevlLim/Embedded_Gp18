# Car_VSC - Buddy 3

This project contains the Buddy 3 implementation for the INF2004 Pico Car:

- two-sensor IR line following input;
- Code 39 barcode capture and decoding;
- one owner for all three RP2040 ADC channels; and
- thread-safe outputs for the Motion Control and Vehicle Controller tasks.

Buddy 3 only measures and interprets the track. It never writes to the motors.

## Hardware connections

| Signal | Pico W pin | ADC channel | Sample period |
| --- | ---: | ---: | ---: |
| Left line sensor | GP26 | ADC0 | 5 ms |
| Centre line sensor | GP27 | ADC1 | 5 ms |
| Right barcode sensor | GP28 | ADC2 | 1 ms |

The GP28 VBAT measurement jumper on the Robo Pico board must be open before an
external sensor is connected to GP28.

The RP2040 contains one ADC with a channel multiplexer. `adc_sampler.c` is
therefore the only module allowed to select ADC channels and read the ADC.

## Source layout

| File | Purpose |
| --- | --- |
| `src/buddy3/adc_sampler.*` | Owns and sequences ADC0, ADC1, and ADC2. |
| `src/buddy3/line_sensor.*` | Normalizes the two line sensors and calculates line error. |
| `src/buddy3/barcode_decoder.*` | Converts 1 ms samples into Code 39 characters. |
| `src/buddy3/buddy3_config.*` | Stores timing, threshold, and calibration settings. |
| `src/buddy3/buddy3_task.*` | Runs the modules and publishes atomic snapshots/events. |
| `src/buddy3_main.*` | Standalone Pico bring-up program. |
| `tests/test_buddy3_main.*` | Hardware-independent unit tests. |

## Behaviour and interfaces

Call `buddy3_task_process_1ms()` once every millisecond. It reads the barcode
sensor every call and reads both line sensors every fifth call. A late call is
counted in the diagnostics instead of being silently ignored.

The consumer can use:

- `buddy3_task_get_line()` for the latest timestamped line snapshot;
- `buddy3_task_get_line_message()` for the 12-byte packed line message;
- `buddy3_task_get_barcode()` to remove one decoded barcode event from the
  queue;
- `buddy3_task_get_barcode_message()` to remove the equivalent 15-byte packed
  barcode message without expanding it;
- `buddy3_task_take_line_lost()` for a one-shot line-lost notification; and
- `buddy3_task_get_diagnostics()` for missed ticks, ADC failures, decoder
  failures, queue overflows, and rejected packed values.

`line_sensor_data_t.error` is normalized to the range -1000 to +1000. A
negative value means the car should steer left; a positive value means it
should steer right. Motion Control remains responsible for converting this
error into motor PWM.

The decoder expects Code 39 with an asterisk start character, one or more data
characters, and an asterisk stop character. It accepts either travel direction
and reports the payload in its normal reading order. It does not translate a
barcode letter into a route decision; that mapping belongs to the Vehicle
Controller.

## Calibration

The defaults in `buddy3_config.c` are safe provisional values, not final track
measurements. Before integration, measure each sensor over white and black
paper and replace `white_adc` and `black_adc`. The normalizer supports either
sensor polarity.

After calibration, verify these items on the real car:

1. A 5 ms line sample period is fast enough at maximum speed.
2. A 1 ms barcode sample period gives at least two samples for the narrowest
   printed element at the chosen cruising speed.
3. `barcode_quiet_ms`, thresholds, and hysteresis tolerate the actual paper
   and room lighting.
4. The line error sign agrees with Motion Control's steering convention.

## µT-Kernel integration

The repository did not yet contain the team's µT-Kernel task definitions or
object IDs, so this code does not invent them. Keep the hardware-independent
modules unchanged and connect them as follows:

1. Initialize Buddy 3 once with `buddy3_task_init()`.
2. From the team's 1 ms cyclic handler or high-priority sensor task, call
   `buddy3_task_process_1ms()` with a monotonic millisecond timestamp.
3. Have Motion Control copy `line_sensor_data_t` with
   `buddy3_task_get_line()`.
4. Have Vehicle Controller drain barcode events with
   `buddy3_task_get_barcode()`.

The short critical sections copy complete structures, preventing consumers
from seeing a mixture of old and new fields.

## Minimum-bit data representation

Buddy 3 keeps natural-width working values while calculating sensor results,
then validates and packs the values at every shared-data boundary. Each packed
object has one writer. Complete objects are copied inside the existing critical
section, so no task performs an unprotected read-modify-write on a shared byte.

| Data | Valid range | Bits | Packed representation |
| --- | --- | ---: | --- |
| ADC sample | 0 to 4095 | 12 | Three samples in 5 bytes |
| Line darkness | 0 to 1000 | 10 each | Two fields in the line message |
| Line error | -1000 to +1000 | 11 | Bias of +1000 |
| Line state | 5 states | 3 | State field in the line message |
| Line valid flag | false or true | 1 | Flag in the line message |
| Code 39 payload symbol | 43 symbols | 6 each | Eight symbols in 6 bytes |
| Barcode payload length | 0 to 8 | 4 | Length field in the barcode message |
| Barcode direction | 2 directions | 1 | Direction flag |
| Barcode valid flag | false or true | 1 | Valid flag |
| Barcode element width | 0 to 511 ms | 9 each | 99 widths in 112 bytes |

The line and barcode messages are 12 and 15 bytes respectively. On the RP2040
ABI, their previous unpacked structures were 16 and 20 bytes. The packed
barcode decoder state is 192 bytes instead of 280 bytes. Across the Buddy 3
decoder, shared snapshots, four-entry barcode queue, diagnostics, and added
packing counter, this saves approximately 104 bytes of static RAM.

Packing rejects ADC, normalized line, line-error, state, payload-length, and
Code 39 symbol values outside their declared ranges. Unpacking also rejects
reserved bits or impossible encoded values. Host tests cover both signed
boundaries, maximum ADC/darkness values, all eight payload positions, and
malformed messages. A Pico W release build with warnings as errors confirms
the packed code compiles for the target. Measure the 1 ms execution time on the
car before treating the performance evidence as complete.

## Build and test

For a Pico W build, use the installed Pico SDK:

```powershell
cmake -S . -B build -G Ninja
cmake --build build
```

The output includes `car_vsc.uf2`. This standalone target prints decoded
barcodes and line data over USB serial, which is useful before µT-Kernel is
merged.

Hardware-independent tests can be built from `tests/CMakeLists.txt`, or run by
compiling the three algorithm modules and `tests/test_buddy3_main.c` with a C11
desktop compiler.

Run the lightweight coding-standard checks with:

```powershell
powershell -ExecutionPolicy Bypass -File tools/check_barr_c.ps1
```

The checks supplement, but do not replace, a manual review against the BARR-C
Embedded C Coding Standard (2018).
