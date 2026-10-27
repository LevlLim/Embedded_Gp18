# Subsystem 1: Telemetry / Comms

**Owner:** Buddy 1: Levl Lim (2501118). Only the owner edits this folder; shared files (`app.h`, `app_main.c`, `msg_defs.*`, `fault.*`, `drivers/`, `template_patches/`) change by team agreement.

## Your files

| File | What it does |
|---|---|
| `subsystem1.h/.c` | init/start, topic names, command ids |
| `telemetry_comms.c` | 200 ms telemetry task, fault + heartbeat topics, MQTT hooks, remote command handling, the `[DBG]` console line |
| `subsystem1_test.c` | your car test |

You also own the shared **`msg_defs.h/.c`** (every packed message) and the template patch `template_patches/lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`.

## How to run your tests

**1. Host tests (on your laptop, no car needed).** These check the pure logic in your files.

```sh
cd testing/host
make test1
```

It ends with `=== X/X checks passed ===`; any `FAIL:` line names the check and the source line. `make` with no target runs every buddy's tests, so run it before you push to check you haven't broken anyone else.

**2. Car test (on the real hardware).** This runs only your `subsystem1_test.c`, with every task of the car running around it.

| Build with | Choose |
|---|---|
| CMake (Pico VS Code) | preset **"Self-test: Buddy 1 only"** (`test-b1`) |
| make | `make TEST_BUDDY=1 -j8` |

Flash `mtk3pico_smp1_usb_cdc_wifi_test_b1.uf2` and open the serial monitor. The firmware waits 4 s for the boot calibration, then prints `[TEST] === test run start: Buddy 1 only ===` and your `[TEST-S1]` lines. Look for `PASS` / `FAIL` on each.

## Host tests (`make test1`)

| Suite | Checks |
|---|---|
| saturation | out-of-range values clamp, never wrap |
| telemetry saturation | every telemetry field at min / max / over |
| telemetry frame | 12-byte frame round trip, short-buffer guard |
| buffer guards | every pack/unpack refuses a short buffer |
| single-word records (shared) | line word and hump byte |
| CR1 size table | bytes before / after packing |

## Car test (`test_b1` image)

No special setup: the car can sit on the desk.

| Line | Pass when |
|---|---|
| `max mutex hold = N us` | N is a few tens of µs. Over 500 prints `WARN` (risk R6). |
| `PASS: duplicate seq rejected` | the same command sequence number is refused twice |
| `PASS: advancing seq accepted` | the next sequence number is accepted |

## Still to test on the bench

These need the real track or instruments, so the self-test cannot do them:

- **MQTT end to end has never run against a broker.** Build the `mqtt` preset with your `config/wifi_secrets.h`, run Mosquitto on a laptop, and check `mosquitto_sub -t "car18/#" -v -F "%t %x"` shows a state frame every 200 ms and a heartbeat every 1 s.
- **Remote commands:** publish 2 bytes `[seq][cmd]` to `car18/cmd` and check START / STOP / RESET work and a repeated `seq` is ignored.
- **Wi-Fi loss (NFR6):** turn the hotspot off while driving. The car must keep going; fault bit `0x200` (WIFI_LOSS) is set and clears on reconnect.

## Settings you are likely to tune

| What | Where |
|---|---|
| Readable `[DBG]` line instead of hex frames | `S1_DEBUG_PRINT` in `subsystem1.h` |
| Topic names | `S1_TOPIC_*` in `subsystem1.h` |
