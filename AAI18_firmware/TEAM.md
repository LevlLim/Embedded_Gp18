# Team and subsystems

Group **AAI18**: autonomous line-following robot car (Pico W on a Robo Pico
board, micro T-Kernel 3.0 SMP). This is the single place that says **who
owns what**: pins, peripherals, tasks, priorities and the interfaces between
subsystems. Keep it in step with `app_program/app.h`, which holds the same
pin map and priorities in code.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|---|
| 1 | Telemetry & Comms: 5 Hz binary telemetry, heartbeat, remote START / STOP / RESET over MQTT, wireless log | Levl Lim | 2501118 | `LevlLim` | `app_program/subsystem1/` | `app_program/subsystem1/TESTING.md`, `subsystem1_test.c` |
| 2 | Motion Control: motor PWM, wheel encoders, line PID, encoder moves and turns, climb boost, e-stop brake | Ze Kai | 2501273 | `ZK-Student-Account` | `app_program/subsystem2/` | `app_program/subsystem2/TESTING.md`, `subsystem2_test.c` |
| 3 | Line & Barcode Sensing: ADC sampling, line position and junctions, Code 39 barcode decoding | Zhi Kai | 2500466 | `tzhikai` | `app_program/subsystem3/` | `app_program/subsystem3/TESTING.md`, `subsystem3_test.c` |
| 4 | IMU / Terrain: accelerometer over I2C, tilt and roll, hump height, lift stop | Jian Yan | 2500853 | `TJianyan` | `app_program/subsystem4/` | `app_program/subsystem4/TESTING.md`, `subsystem4_test.c` |
| 5 | Vehicle Controller & Obstacle: mission state machine, junction turns, ultrasonic + servo scan, obstacle bypass, fault table | Jun Hong | 2501666 | `AAILimJunHong` | `app_program/subsystem5/` | `app_program/subsystem5/TESTING.md`, `subsystem5_test.c` |

Host unit tests for every subsystem (no robot needed): `testing/host/`
(`make`, or `make test1` … `make test5` for one subsystem).

**Shared files**, changed only by agreement of the team:

| File | Keeper | What it holds |
|---|---|---|
| `app_program/app.h`, `app_program/app_main.c` | whole team | pin map, priorities, periods, task start-up |
| `app_program/msg_defs.h/.c` | S1 (Levl Lim) | every packed message and field (Change Requirement 1) |
| `app_program/fault.h/.c` | S5 (Jun Hong) | the 10-bit shared fault table |
| `app_program/drivers/gpio_irq.h/.c` | S2 (Ze Kai), shared with S5 | the single GPIO interrupt dispatcher (encoders + ultrasonic echo) |
| `app_program/drivers/rp2040_regs.h` | S2 (Ze Kai) | RP2040 registers the template's BSP does not provide |
| `config/config.h`, `build_make/pico_rp2040.mk`, `lib/liblwip/sysdepend/pico_rp2040/lwipopts.h`, `.../mqtt_utk.c` | whole team (`mqtt_utk.c`: S1) | template files we changed; see README, "Template files this project changes" |
| this file | whole team | |

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S5 | HC-SR04P ultrasonic TRIG (so the console is always USB and `CRASH_REPORT=1` is refused at compile time) |
| GP1 | S5 | HC-SR04P ultrasonic ECHO (GPIO interrupt) |
| GP2 | S2 | left encoder phase A (GPIO interrupt) |
| GP3 | S2 | left encoder phase B |
| GP4 | S2 | right encoder phase A (GPIO interrupt) |
| GP5 | S2 | right encoder phase B |
| GP6 | | free |
| GP7 | | free |
| GP8 | S2 | left motor, PWM slice 4 A |
| GP9 | S2 | left motor, PWM slice 4 B |
| GP10 | S2 | right motor, PWM slice 5 A |
| GP11 | S2 | right motor, PWM slice 5 B |
| GP12 | S5 | ultrasonic scan servo, PWM slice 6, 50 Hz |
| GP13 | | free |
| GP14 | | free |
| GP15 | | free |
| GP16 | S4 | I2C0 SDA (accelerometer) |
| GP17 | S4 | I2C0 SCL (accelerometer) |
| GP18 | S5 | WS2812 status LED (claimed, not implemented) |
| GP19 | | free |
| GP20 | S5 | Robo Pico START button (input, pull-up) |
| GP21 | S5 | Robo Pico STOP button (input, pull-up) |
| GP22 | S5 | buzzer, PWM slice 3 (claimed, not implemented) |
| GP26 | S3 | left line IR sensor, ADC0 |
| GP27 | S3 | right line IR sensor, ADC1 |
| GP28 | S3 | barcode IR sensor, ADC2 |

## 3. Peripherals

Each peripheral is driven by exactly one task, its *owning task*. Every
other task asks the owner through a kernel object or a single-word getter
(section 5) and never touches the peripheral itself.

| Peripheral | Owner subsystem | Owning task / context | Pins |
|---|---|---|---|
| UART0, UART1 | *unused* | (console is USB CDC) | |
| I2C0 | S4 | `imu_task` (`subsystem4/i2c0.c`) | GP16, GP17 |
| I2C1 | *unused* | | |
| SPI0, SPI1 | *unused* | | |
| ADC | S3 | `adc_sampler` cyclic handler (`subsystem3/adc_sampler.c`), read by `line_task` and `barcode_task` | GP26–GP28 |
| PWM slices 4, 5 | S2 | `motion_task` | GP8–GP11 |
| PWM slice 6 | S5 | `obstacle_task` | GP12 |
| PWM slice 3 | S5 | (buzzer, not implemented) | GP22 |
| GPIO bank 0 interrupt | S2 (shared with S5) | `gpio_irq` ISR: encoder edges (S2), echo timing (S5) | GP1, GP2, GP4 |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT

No application task calls `cyw43_...`, lwIP or MQTT functions.

- **Out:** `telemetry_task` (S1) fills the `mbf_outbox` message buffer.
  The MQTT client inside `cyw43_utk_task` drains it with `TMO_POL` (it
  never blocks) and publishes it. The hook functions are `app_mqtt_*`,
  declared in `mqtt_utk.c`.
- **In:** the same task passes `car18/cmd` payloads to
  `app_mqtt_on_message()`. S1 checks the sequence number (QoS 1
  duplicates) and forwards the command to S5.
- Topics: `car18/tele/#` (binary telemetry), `car18/log` (console text),
  `car18/cmd` (commands). Build with `NET=lwip MQTT=1` (CMake preset `mqtt`).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Our
bands follow the timing need (Design Review V2 section 3), not the
subsystem number. Every periodic task is woken by its own cyclic handler
(`app_start_periodic_task()`).

| Subsystem | Priority band | Tasks (name – priority – period – core) |
|---|---|---|
| 1 | 20 | `telemetry_task` – 20 – 200 ms (5 Hz) – core 1 |
| 2 | 5 | `motion_task` – 5 – 10 ms – core 0 |
| 3 | 7–8 | `line_task` – 7 – 5 ms – core 0; `barcode_task` – 8 – 1 ms – core 0 |
| 4 | 9 | `imu_task` – 9 – 10 ms (4 ms WCET budget) – core 0 |
| 5 | 12–13 | `controller_task` – 12 – event-driven – core 0; `obstacle_task` – 13 – 60 ms – core 0 |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1. Core 0 runs everything that
drives the car; core 1 runs telemetry only.

## 5. Interfaces between subsystems

The provider creates the object (or owns the word) and publishes its access
functions in its own `subsystemN.h`. The user only sends, waits or reads.

| ID | From → To | Mechanism | Object / function | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S5 → S2 | message buffer (owned by S2) | `mbf_cmd`, `s2_send_command()` | packed 2-byte motion command: STOP / LINE_PID / FORWARD / REVERSE / TURN_L / TURN_R / U_TURN + distance or angle | S2, S5 |
| IF-02 | S3 → S5 | message buffer (owned by S3) | `mbf_barcode`, `s3_take_barcode()` | packed 1-byte barcode command: LEFT / RIGHT / STRAIGHT / U_TURN | S3, S5 |
| IF-03 | S3 → S2, S5, S1 | single 32-bit word, one writer (owned by S3) | `s3_read_line_word()`, `s3_get_line_state()` | 7-bit line error, valid bit, 24-bit ms timestamp | S1, S2, S3, S5 |
| IF-04 | S1, S2, S3, S4 → S5 | event flag (owned by S5) | `flg_events`, `s5_post_event()` | START / STOP / RESET (S1), MOVE_DONE (S2), JUNCTION and BARCODE (S3), HUMP (S4) | all |
| IF-05 | all → all | event flag (owned by S5) | `flg_fault`, `fault_set()` / `fault_get()` | 10-bit fault table; S2 brakes on any E-STOP bit | all |
| IF-06 | S2 → S1 | shared data under a `TA_INHERIT` mutex (owned by S1) | `mtx_snapshot`, `s1_publish_motion()` | wheel speeds and distance for telemetry | S1, S2 |
| IF-07 | S4 → S2, S1 | single-byte getters (owned by S4) | `s4_get_roll_deg()`, `s4_get_tilt_deg()`, `s4_get_hump_packed()` | roll for the climb boost (S2); tilt and packed hump for telemetry (S1) | S1, S2, S4 |
| IF-08 | S1 → kernel MQTT task | message buffer (owned by S1) | `mbf_outbox`, `app_mqtt_next_publish()` | 12-byte telemetry frame, fault word, heartbeat, log lines | S1 |
| IF-09 | S1 → S5 | single 32-bit word (owned by S5) | `s5_manual_move()` | parked calibration move: type + distance or angle | S1, S5 |
| IF-10 | S2 → S3, S4 | single-writer encoder totals | `gpio_irq_read_encoders()` (S3), `s2_get_distance_mm()` (S4) | wheel distance for barcode bar widths (S3) and hump length (S4) | S2, S3, S4 |
| IF-11 | S5 → S3 | function call (owned by S3) | `s3_set_active()`, `s3_calibrate()`, `s3_force_straight()` | start / stop sensing, calibrate, hold straight through a junction | S3, S5 |

Interfaces are exercised by the host tests in `testing/host/` and the
on-car self-tests; see each subsystem's `TESTING.md`.

## 6. Kernel object budget

Limits are set in `config/config.h`.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 | 2 | 1 | 2 |
| Semaphore | 16 | | | | | |
| Event flag | 16 | | | | | 2 (`flg_events`, `flg_fault`) |
| Mailbox | 12 | | | | | |
| Mutex | 12 | 1 | | | | |
| Message buffer | 12 | 1 | 1 | 1 | | 1 (`mbf_obstacle`, internal) |
| Fixed memory pool | 12 | | | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | 1 | 1 | 3 (2 tasks + ADC) | 1 | 1 |
| Alarm handler | 12 | | | | | |

Totals: 7 tasks, 2 event flags, 1 mutex, 4 message buffers, 7 cyclic
handlers (the kernel's USB and WiFi service tasks use a few more).
