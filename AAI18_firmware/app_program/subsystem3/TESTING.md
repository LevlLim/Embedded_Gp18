# Buddy 3: sensors hugging the centre line

Buddy 3 owns this folder. Motor commands and mission decisions belong to
Buddies 2 and 5; this change adds sensing and a junction observation interface.

## Mounting and wiring

Viewed from above, facing forwards:

```text
                       forward ^
             white   BLACK LINE   white          barcode strip
               L         ||         R                 B
             GP26        ||       GP27              GP28
```

- GP26 / ADC0 is the LEFT edge sensor (`ADC_CH_LINE_L`).
- GP27 / ADC1 is the RIGHT edge sensor (`ADC_CH_LINE_R`). The legacy
  `ADC_CH_LINE_C` name and the shared `PIN_IR_LINE_C` map still refer to this
  same pin for compatibility. There is no sensor on the centre line now.
- GP28 / ADC2 remains dedicated to Code 39 barcodes.

Mount the line sensors at equal height, just outside opposite edges of the
black line. Sensor spacing must exceed the line width, allowing for each
sensor's optical footprint. For an 18 mm line, the old 12–15 mm spacing is no
longer appropriate. Set the actual spacing by checking that BOTH sensors see
white when aligned and that a small sideways movement makes one see black.
Keep GP28's VBAT measurement jumper open on the Robo Pico board.

## Behaviour

The line task runs every 5 ms. After successful calibration and activation:

| Left | Right | Error | Meaning |
| --- | --- | ---: | --- |
| White | White | 0 | Continue straight; the line may also be missing |
| Black | White | -50 | Steer left |
| White | Black | +50 | Steer right |
| Black | Black | 0 | Junction candidate; no preferred steering direction |

Three consecutive DISTINCT both-black ADC pairs confirm a junction candidate
(`S3_JUNCTION_CONFIRM_SAMPLES`). This is nominally three 5 ms samples, with
10 ms between the first and third observations. A duplicate pair cannot advance
confirmation. Either white reading, invalid/stale ADC data, deactivation or
recalibration breaks confirmation. After clearing, a new encounter must confirm
again. Persistent both-black produces a level indication, not repeated events.

All four fresh combinations publish valid `MSG_LINE_TRACK` steering data.
`MSG_LINE_LOST` now means acquisition unavailable/stale, not both-white.
The higher ADC value is treated as black, as in the preceding integrated code.
Hysteresis retains each sensor's previous classification near its threshold.

Both-white cannot prove the presence of a line. Both-black cannot distinguish
a T, crossing or wide black marking. These are physical sensing limitations.

## Calibrate and test on the car

1. Build the **Self-test: Buddy 3 only** preset (`test-b3`). During the 3 s boot
   calibration, sweep EACH of GP26, GP27 and GP28 across black AND white. The
   calibration requires enough fresh samples and contrast on all three inputs;
   a partial/failed calibration does not replace any thresholds.
2. Check `[TEST-S3]` output for the new pin roles, truth table, ADC freshness
   and `*A* -> A -> left` decode result. This test sends no motion command.
3. Test normal line-following at low speed. Aligned: both white, error 0.
   Move the line under GP26: negative error. Under GP27: positive error.
   Confirm that the actual wheel response steers in the stated direction.
4. Hold both over black for at least three samples. Read
   `s3_read_line_sensors()` to check `S3_LINE_JUNCTION`. Brief black noise and
   interrupted sequences must not confirm. Clear either sensor and verify
   the flag clears. A subsequent encounter must confirm again.
5. Check white/white for several seconds: valid zero error, no line-lost event.
   Confirm ADC failure/staleness still invalidates steering and junction data.
6. Verify smooth bends, actual sensor spacing/height, lighting and speed on the
   real track. Do not treat a successful synthetic decode as a physical scan.

The debug output labels GP26 as `left` and GP27 as `right`, and includes `J1`
for a confirmed junction (`J0` otherwise, `J?` if observations are unavailable).
Its B/W labels compare a threshold without hysteresis;
`s3_read_line_sensors()` reports the actual classification.

The actual car's measured settings are preserved: readings at or above 3950
are treated as no surface and excluded from calibration/capture. Either line
sensor losing its surface invalidates the line reading, rather than treating
air as black or aligned white. A calibration sweep needs at least 400 ADC
counts of contrast on each channel, avoiding the measured black-level noise.

## Host regressions

From PowerShell, run this folder's `tests/Run-HostTests.ps1` (requires host gcc).
Optional parameters: `-Compiler <gcc-path>` and `-OutputDirectory <path>`.
It builds with warnings as errors and runs production-code regressions for:

- all four edge combinations, sustained white, steering sign and zero error;
- junction debounce, duplicate rejection, clearing, rearming and stop/start;
- transactional three-channel calibration, ADC failure, stale observations;
- all 26 Code 39 letters in both directions, malformed scans and queue limits;
- ADC sampling/timestamp behaviour and retained-letter telemetry compatibility.
- independent Code 39 reference patterns at three timings in both directions;
- exact quiet-zone boundaries, back-to-back scans, and short quiet-zone rejection;
- delayed batch rejection before publication, calibration changes mid-capture;
- initialization/task retries without duplicate objects, and ADC counter wrap.

The team host harness keeps its other-buddy tests and now checks real A-Z frames instead of the former four-width placeholder.

## Interface for junction handling

```c
uint8_t flags = 0U;
if (s3_read_line_sensors(&flags))
{
    bool junction = (0U != (flags & S3_LINE_JUNCTION));
    bool left_black = (0U != (flags & S3_LINE_LEFT_BLACK));
    bool right_black = (0U != (flags & S3_LINE_RIGHT_BLACK));
    /* Controller consumes these observations in its own manoeuvre state. */
}
```

The call returns false and clears the output for unavailable, inactive,
uncalibrated or stale input; a null pointer also returns false. A true result
with zero flags means both white. Flags and timestamp are published together in
one atomic 32-bit record (three flag bits, one valid bit, 28-bit millisecond
timestamp). Junction flags use this Buddy 3 API without new shared events.
Retained-letter telemetry separately uses schema version 3: 13 bytes, with
byte 12 encoding A=1 through Z=26, or zero before the first valid decode.
Telemetry readers must recognize schema 3. The junction API is a fresh
LEVEL: poll faster than the junction can pass beneath the sensors, or add an
agreed queued event interface.

## Controller work required before barcode-directed junction driving

The current Buddy 5 controller executes a barcode immediately. It does NOT yet
remember it and wait for `S3_LINE_JUNCTION`. Buddy 3's retained barcode API
(`s3_read_barcode`) already provides the last letter, mapped direction,
confidence and sequence. A retained result survives stop/start, so its presence
alone must not arm a new mission or repeat a consumed instruction.

The controller needs to remember a fresh accepted command, latch a confirmed
junction once, position the axle, execute the turn and reacquire the outgoing
line. The axle-to-sensor distance and pivot behaviour require track measurements;
there is no guessed 90-degree manoeuvre in Buddy 3.

Two existing integration assumptions also need changing:

- Buddy 2's `line_in_view()` and Buddy 5's `poll_running()` currently use
  `MSG_LINE_TRACK` as proof the line was reacquired. In this layout, WHITE/WHITE
  is TRACK too. Use the fresh left/right flags plus manoeuvre history instead.
- During a 90-degree turn, first clear the original both-black junction, then
  require the direction-appropriate edge crossing before settling into
  WHITE/WHITE. WHITE/WHITE by itself must not finish a turn/search. Keep an
  encoder/angle limit and a timeout if reacquisition fails.

These changes belong to the Motion Control/Vehicle Controller integration;
ordinary edge following and junction sensing are implemented in Buddy 3 here.

## Barcode repairs (28 October 2026)

- A white-to-black edge at exactly the required trailing quiet interval now
  publishes the completed barcode and can start the next frame on that edge.
  Short trailing white still rejects the candidate.
- Buffered samples whose batch end is already too old are rejected before
  decoding, so delayed capture cannot publish an obsolete command first.
- Calibration changes cancel partial frames. Capture compares the calibration
  snapshot directly as well as honoring the reset ticket, covering preemption
  between the threshold commit and reset request. The retained pair survives.
- Mapping outputs are checked before packing: only STRAIGHT, LEFT, RIGHT and
  U_TURN fit the existing interface. An unsupported value preserves the prior
  pair and emits no replacement command. Change assignments in
  `barcode_mapping.c`; keep the supported direction enum/interface stable.
- Failed barcode-buffer initialization remains retryable. Startup remembers
  already-created tasks so retrying the next step cannot duplicate a writer.
- The ADC ring keeps its full history across the internal write-counter wrap.
- The pure width decoder and live capture apply consistent gap/time limits.

The on-car synthetic A test prints the mapped direction dynamically, so a
future reassignment of A does not require editing the hardware self-test.
The host tests check mapping validity and decoder independence without
hardcoding the temporary letter assignments. Mapping remains separate from decoding.

## Build in Car_VSC

Open PowerShell in `Car_VSC/AAI18_firmware` and run `./Build-Buddy3.ps1` for
the full-car application with Buddy 3 self-tests. Run
`./Build-Buddy3.ps1 -Normal` for the normal full-car application.
The helper configures and builds the existing `test-b3` or `usb` CMake preset
using this project's sources, SDK and build directory. The resulting UF2 is
printed when the build succeeds.


## Change the letter mapping

Edit only the 26-character `g_barcode_mapping` string in `barcode_mapping.c`:

```c
    /* ABCDEFGHIJKLMNOPQRSTUVWXYZ */
      "LRSUSSSSSSSSSSSSSSSSSSSSSS";
```

Each position is the action for the letter above it. Use L=left, R=right,
S=straight, U=U-turn. The current assignments are A=left, B=right, C=straight,
D=U-turn and E..Z=straight. For example, change the first character to R to
make A mean right. Keep exactly 26 uppercase characters.

Length mistakes fail compilation. Unsupported characters fail mapping
validation and barcode initialization; no unsupported command is published.
Run the host regressions, then rebuild/flash with `./Build-Buddy3.ps1` (or
`-Normal`). The decoded/retained LETTER and the four-direction interface are
independent of the assignments, so no other subsystem needs editing.

A future CSV option can generate this table during the build. This version
uses the C string; it does not load a CSV at runtime or read PC files from
the Pico. Mapping changes require rebuilding and reflashing firmware.
