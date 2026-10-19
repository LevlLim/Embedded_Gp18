# Buddy 1 binary MQTT protocol and minimum-bit evidence

The 19 October change requirement applies to application data in RAM, across
tasks/interrupts and on MQTT. This document describes the Buddy 1 portion.
Sensor ranges below are **provisional**, as the team has not measured their
limits. The setters reject out-of-range values and return `0`; they never
silently truncate them. Confirm each range with the other buddies before
freezing the protocol for Week 13.

## Topics and frames

| Topic | Sender | Payload | Rate |
| --- | --- | --- | --- |
| `aai18/robot/telemetry` | Pico W | Binary telemetry | At least 5 Hz; implementation target 200 ms |
| `aai18/robot/heartbeat` | Pico W | Binary heartbeat or one-byte offline will | 1 s while connected |
| `aai18/robot/command` | PC/controller | Binary START or STOP | On demand, QoS 0, retain **off** |

All multi-byte integers and packed fields are little-endian. Within a byte,
the first field uses the least-significant bit. Every frame starts with:

| Header bits | Meaning |
| --- | --- |
| 0–2 | version = 1 |
| 3–4 | kind: 0 command, 1 telemetry, 2 heartbeat, 3 offline will |
| 5–6 | command action: 1 START, 2 STOP; zero otherwise |
| 7 | zero |

The **7-byte command** is header (1), current boot session ID (32 bits), then
command sequence (16 bits). The Pico accepts only the exact length, version,
action and session. It accepts a sequence only once, with a forward difference
of 1–32767 modulo 65536. This rejects duplicate or delayed START after a newer
command, including after MQTT reconnect. The 32-bit session changes at boot,
so a retained command from an older boot is rejected with high probability.
The session comes from the course port's timer-mixed PRNG; it is a replay
guard, **not authentication**. Secure the broker before motor-enabled use.
STOP still raises `flg_estop` directly and latches the local stop state; it
does not wait for the Vehicle Controller. A command rejected as stale raises
no event. The PC helper is `scripts/mqtt-tool.py`.

The **9-byte heartbeat** is header (1), boot session (32), uptime in ms (32).
The broker's last will for unexpected disconnect is a single `0x19` byte on
the heartbeat topic. It is not a periodic heartbeat. MQTT topic strings are
transport addresses; application values are binary.

Telemetry begins with header (8), boot session (32), uptime ms (32), message
sequence (16), mission state (3), valid-field mask (5), fault mask (6), for
**102 bits / 13 bytes**. The message sequence wraps at 65535; the receiver
should use session plus modulo sequence when spotting gaps. A set valid bit
appends its group in the order shown below; unset groups consume no bits.
Trailing bits in the last byte are zero. Fully populated telemetry is exactly
**224 bits / 28 bytes**.

| Field/group | Valid range | Bits on wire | Shared RAM representation | Notes |
| --- | --- | ---: | --- | --- |
| mission state | 0–7 | 3 | 3-bit C bit-field | Team to assign the final state IDs |
| valid groups | five flags | 5 | 5-bit C bit-field | Motion, line, barcode, hump, obstacle |
| faults | six flags | 6 | 6-bit C bit-field | See fault map below |
| left/right speed | −2047…2047 mm/s each | 12 each | two 12-bit C bit-fields | Two's complement; provisional max |
| left/right encoder | −8,388,607…8,388,607 each | 24 each | two 24-bit C bit-fields | Two's complement; verify counts per run |
| line error | −2047…2047 | 12 | 12-bit C bit-field | Team to confirm scale and timestamp separately |
| decoded barcode | 0…15 | 4 | 4-bit C bit-field | Team to agree code mapping; zero can mean none |
| hump peak | 0…1023 mm | 10 | 10-bit C bit-field | Reject negative estimates |
| obstacle distance/width | 0…4095 mm each | 12 each | two 12-bit C bit-fields | Other profile fields await team API |
| uptime | 0…2³²−1 ms | 32 | uint32, generated at publication | Wraps after about 49.7 days |
| telemetry sequence | 0…65535 | 16 | uint16, generated at publication | Wraps modulo 65536 |
| boot session | 0…2³²−1 | 32 | uint32, generated at boot | Needed for command freshness |
| command sequence | 0…65535 | 16 | uint16 tracker | Per boot, forward half-window |
| command action | START/STOP | 2 | two header bits | Zero and three invalid |

Fault bits: 0 motor stall, 1 encoder fault, 2 IMU fault, 3 no ultrasonic echo,
4 Wi-Fi loss, 5 line loss. These are report bits, not six separate mission
states. The team must decide which faults brake, latch, recover, or allow
degraded driving. The current communication code does **not** yet infer all
of these faults from absent sensors. If the team needs additional fault bits
or a larger valid range, update the table, packer/decoder and wire version.

The shared snapshot is written under a `TA_INHERIT` µT-Kernel mutex. The
telemetry task copies the entire 24-byte packed struct under the mutex and
decodes it after unlocking. The getters and setters are task APIs, **not ISR
APIs**. The radio task uses a polling lock and skips one publication if the
snapshot is busy. Other ISR/task shared values, such as encoder counts and
line error, need their own safe ownership/copy design from the relevant buddy.

## Measured comparison, same MQTT build profile

Build: `SMP=0`, USB CDC, CYW43 Wi-Fi, DHCP, MQTT and Buddy 1 app on Pico W.
`arm-none-eabi-size` reports `.data + .bss` as the linked RAM figure; this
includes the RTOS and networking stack, not only Buddy 1. Baseline is the
previous text-payload ELF measured **before** rebuilding on 19 October 2026.

| Measure | Previous text firmware | New binary firmware | Difference |
| --- | ---: | ---: | ---: |
| Shared snapshot object | 56 B | 24 B | −32 B |
| Linked `.data` | 2,476 B | 2,476 B | 0 B |
| Linked `.bss` | 50,368 B | 50,348 B | −20 B |
| Linked `.data + .bss` | 52,844 B | 52,824 B | −20 B |
| Example empty telemetry, uptime 1000, seq 0 | 155 B JSON | 13 B binary | −142 B |
| Example all fields at provisional limits | 190 B JSON | 28 B binary | −162 B |
| Heartbeat, uptime 1000 | 4 B text | 9 B binary | +5 B; includes new 4 B session |
| START command | 5 B text | 7 B binary | +2 B; includes session and sequence |

Message comparisons use the same application sample; the new messages also
include the boot session. Smaller frame size is not the only goal: the new
session and sequence make replay rejection possible. Flash text size was
331,265 B before and 330,601 B after this build. The test runner checks
field boundaries, negative signed values, exact packed sizes, malformed
frames, session mismatch, duplicate/stale sequences and wraparound. A second
host test checks packed RAM snapshot round-trip, range rejection, mutex copy,
and START/STOP event flag routing.

## Remaining Week 13 evidence

- Reflash the new UF2 and measure binary telemetry interval, 1 s heartbeat,
  command acceptance and reconnect on real Pico W hardware. The 15 October
  physical test was on the *previous text protocol*.
- When Buddy 2–5 code is integrated, measure actual Motion Control mutex
  blocking, task WCET, total CPU utilisation, and e-stop latency. Target CPU
  utilisation is under 70%; remote/collision stop target is under 100 ms.
- Fill in the team's field ranges and justify them from encoder, track,
  barcode and sensor measurements. Re-test after any width change.
- Audit all other application structs, message buffers and ISR/task handoffs
  for bit widths and atomic read-modify-write. Buddy 1 alone cannot satisfy
  a requirement that applies to all firmware.
