# Buddy 4 - IMU / Terrain subsystem (Group AAI18)

Jian Yan, 2500853. Physics Watchdog: hump estimation, tilt, collision
detection. Covers Change Requirement 1 (minimum bit data representation) for
this subsystem.

```
include/app_bits.h          packing helpers, clamping, compile time width checks
include/fault_flags.h       shared 10 bit fault word, bit ownership table
include/imu_terrain.h       field ranges, packed message, public API
src/fault_flags.c           atomic set and clear through the kernel event flag
src/imu_terrain.c           task cycle: FIFO drain, tilt, hump, collision, publish
test/test_imu_pack.c        host tests, no hardware needed
test/Makefile               gcc build
docs/buddy4_data_representation.md   the Week 13 evidence table
```

## Run the tests

```
cd test
make
```

Expected output ends with `all tests passed` and prints the before and after
message sizes.

## Hooks the target build must provide

These are stubbed in the test file. Wire them to the real drivers when
integrating:

```c
int32_t  imu_hal_read_fifo(int16_t *ax, int16_t *ay, int16_t *az, int32_t max);
int32_t  encoder_distance_mm(void);          /* Buddy 2 */
void     terrain_publish(terrain_msg_t msg); /* message buffer to Vehicle Controller */
uint32_t board_time_us(void);                /* time_us_32() on the Pico */
```

`src/fault_flags.c` expects `ID g_flg_fault` to be created in `main` and
shared by all tasks.

## Notes for integration

- Do not read-modify-write the fault word. Call `fault_set` / `fault_clear`.
- Only the Vehicle Controller calls `fault_clear`.
- Values marked provisional in the docs stay provisional until the car runs.
