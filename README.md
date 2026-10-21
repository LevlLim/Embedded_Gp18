# Buddy 2 — Motor Control files

## Included files

- `motor_control.h` — compact command/state/fault definitions and public API.
- `motor_control.c` — motor command handling, stop/fault behaviour, and compact binary encoding.
- `test_motor_control.c` — small host-side smoke test using a fake motor driver.

## Design basis from the supplied PDF

The design review identifies a Pico W using C, the Pico SDK and µT-Kernel; wheel encoder inputs on GP2–GP5; and motor driver control/PWM pins on GP8–GP11. It also specifies speed-control accuracy and a stop response requirement following a collision, unrecoverable line loss or remote stop. The existing hardware/driver wrapper and exact PWM configuration are not visible in the PDF's readable text, so this module uses a hardware callback rather than inventing those details.

## Minimum-bit representation

- Motor command: 3 logical bits (eight possible codes; seven defined command codes including STOP).
- Controller state: 2 logical bits (four states).
- Five listed fault types: five logical bits in one `uint8_t` storage unit.
- Speed percentage 0–100: 7 logical bits.
- Queue/MQTT command representation: exactly 2 bytes with explicitly defined bit positions. Do not send a C struct's memory directly.

The C enums and status members are not guaranteed to occupy their logical bit widths. The wire format is explicitly packed; in-memory members use ordinary byte types for predictable access and to avoid compiler-dependent bit-field layout. This is a practical compromise: minimum logical field width with clear, deterministic serialization.

## Integrating with the robot

1. Connect `mc_platform_t.set_motor_percent` to the project's existing motor driver. It must map signed wheel percentages (-100..100) to the board's actual direction/PWM implementation and return `false` on a driver error.
2. Provide `lock` and `unlock` callbacks if status/fault data is shared across tasks. Use a µT-Kernel primitive suitable for the access context. Do not call a blocking mutex from an ISR; ISR-originated events should be handed to a task or protected with an ISR-safe critical section/atomic operation.
3. Route collision, unrecoverable line-loss and remote-stop events to `mc_stop()` with the required priority. Confirm the full stop path meets the project's stated timing requirement on hardware.
4. Route fault sources to `mc_set_faults()` and clear only the fault bits that have actually recovered.
5. For MQTT, place the two bytes returned by `mc_encode_command()` in a binary payload and decode with `mc_decode_command()`. Ensure the MQTT library and broker path preserve binary bytes; do not convert them to a null-terminated string.
6. Integrate encoder-based PID speed control in the existing motion-control task. This module exposes the command boundary but does not invent PID gains, encoder resolution, control period, or PWM frequency because these are not established by the source document.

## Build and smoke test

Host smoke test (GCC or Clang):

```sh
cc -std=c11 -Wall -Wextra -Werror motor_control.c test_motor_control.c -o test_motor_control
./test_motor_control
```

For the Pico build, add `motor_control.c` to the existing project target and adapt the callback to its SDK/µT-Kernel APIs. The host test does not prove real-time response or electrical behaviour; those require on-robot tests.

## Important implementation note

This is a starter module, not a drop-in replacement for an unseen existing motor driver. Before merging, align command semantics with the team's agreed turn/distance APIs, verify safe brake/coast behaviour, and perform hardware timing tests. The design PDF states that both motor-driver inputs LOW are used for braking; the callback must preserve the actual board's truth table and polarity.
