# Buddy 4 - IMU / Terrain, minimum bit data representation

Group AAI18, Buddy 4 (2500853). Covers Change Requirement 1 for the IMU,
terrain and collision subsystem (`imu_terrain.c`).

## What changed

| Week 6 | Now | Why |
|---|---|---|
| IMU_FAULT and collision stored as their own variables | Single bits in one shared 10 bit fault flag | Change Requirement 1. A true/false fault does not need an int. |
| Hump height, tilt and peak acceleration sent as separate 32 bit values | One 8 byte packed event message | Same fields, 28 bytes down to 8 |
| Raw 400 Hz FIFO samples stored as int16 | Unchanged | The sensor already outputs 16 bit and the samples are discarded each cycle, so packing them costs CPU and saves no lasting RAM |
| Tilt x encoder distance computed in 32 bit | Unchanged | Only stored and transmitted results are packed. Packing the intermediate would truncate it |

## Data representation table

| Field | Valid range | Resolution | Bits | Representation |
|---|---|---|---|---|
| Event type | 6 event types | n/a | 3 | `w0[2:0]`, enum `terrain_evt_t` |
| Valid | 0 or 1 | n/a | 1 | `w0[3]` |
| Hump height | 0 to 127 mm | 1 mm | 7 | `w0[10:4]`, unsigned |
| Tilt | -45.0 to +45.0 deg | 0.5 deg | 8 | `w0[18:11]`, stored as half degrees + 90 |
| Peak acceleration | 0 to 8.00 g | 0.05 g | 8 | `w0[26:19]`, unsigned |
| Timestamp | 0 to 1048575 ms | 1 ms | 20 | `w1[19:0]` |
| Sequence | 0 to 15 | 1 | 4 | `w1[23:20]`, drop detection |
| IMU_FAULT | 0 or 1 | n/a | 1 | `FLT_IMU_FAULT`, bit 2 of the shared fault flag |
| Collision | 0 or 1 | n/a | 1 | `FLT_COLLISION`, bit 3 of the shared fault flag |

### Why 7 bits and not 5 for hump height

FR5 allows ±5 mm on the reported hump height. The quantisation step adds
straight into that budget, so it has to stay well below the tolerance. A 1 mm
step uses a fifth of it and needs 7 bits for 0 to 127 mm. Five bits would
force a 4 mm step, which uses most of the tolerance before the sensor error is
even counted.

### Why the timestamp is 20 bits and not 16

A 16 bit millisecond counter wraps at 65.5 s. The flat track runs in the test
plan are 60 s and longer, so hump and collision events would alias onto each
other. 20 bits covers about 17.5 minutes.

## Shared fault flag, one writer per field

The RP2040 is Cortex-M0+. There is no LDREX/STREX and no bit banding, so

```c
g_faults |= FLT_IMU_FAULT;   /* LDR, ORR, STR */
```

is not atomic. If the Vehicle Controller writes between the load and the
store, a fault bit is lost and nothing reports it.

Team decision from Tribunal 1:

- each task sets only the bits it owns
- only the Vehicle Controller clears bits
- set and clear go through the kernel event flag, which is atomic

| Bit | Fault | Owner |
|---|---|---|
| 0 | MOTOR_STALL | Buddy 2 |
| 1 | ENCODER_FAULT | Buddy 2 |
| 2 | IMU_FAULT | Buddy 4 |
| 3 | COLLISION | Buddy 4 |
| 4 | NO_ECHO | Buddy 5 |
| 5 | SERVO_FAULT | Buddy 5 |
| 6 | LINE_LOST | Buddy 3 |
| 7 | BARCODE_FAULT | Buddy 3 |
| 8 | WIFI_LOST | Buddy 1 |
| 9 | MQTT_LOST | Buddy 1 |

`test_imu_pack.c` checks that no two owner masks overlap, so the one writer
rule cannot be broken by accident later.

## Message size comparison

Measured by `test_imu_pack.c` on the host build.

| | Before | After | Saved |
|---|---|---|---|
| IMU / terrain event message | 28 bytes | 8 bytes | 20 bytes, 71% |
| Fault storage | 10 variables x 4 bytes = 40 bytes | 10 bits inside one 4 byte flag | 36 bytes |

RAM comparison for the full firmware goes in the Week 13 submission, taken
from the map file before and after this change.

## Correctness evidence

`make -C test` runs on any machine with gcc, no Pico needed.

- every in-range combination of event, hump height, tilt and peak
  acceleration survives pack then unpack unchanged
- out of range values saturate and are counted, they never wrap. A 500 mm
  reading wrapping to 116 mm would look like a plausible hump and silently
  fail FR5, which is Risk R3
- no two buddies own the same fault bit
- the hump height maths stays inside the FR5 tolerance for a known input

## Still open

| Item | Status |
|---|---|
| ±5 mm hump accuracy (A7) | Untested, needs the real hump. Bit width confirmed after measuring |
| 4 ms task WCET (NFR11) | `imu_terrain_last_cycle_us()` is instrumented, not measured yet |
| Tilt estimate | Accelerometer only, corrupted by linear acceleration. No gyro on the hardware list |
| IMU_FAULT response time | Controller action agreed, timing not yet defined |
