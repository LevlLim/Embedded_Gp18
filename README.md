# INF2004 Group AAI18 robot car

This repository contains Group 18's robot car project. The `Levl_Buddy1`
branch now holds a **safe communications starting point** for Buddy 1 (Levl):
Pico W Wi-Fi, DHCP, MQTT telemetry, heartbeat, and remote START/STOP reception.
It does not drive motors yet. Other buddies will add sensing, motion, and the
Vehicle Controller through the interfaces below.

## What is what?

- **Pico W:** the green microcontroller with Wi-Fi.
- **Robo Pico:** the purple carrier with motor drivers and connectors. The
  Pico W can first be tested on the Maker Pi Pico carrier over USB alone.
- **MQTT:** a messaging protocol implemented in software. It needs an MQTT
  **broker** (server program) reachable over Wi-Fi. It is not another hardware
  component to wire. [Eclipse Mosquitto](https://mosquitto.org/download/) is
  one broker you can run on a computer.
- **µT-Kernel:** the course's real-time OS. The course-linked
  [RP2040 port](https://github.com/sirfonzie/mtk3smp-rp2040) is copied into
  `vendor/mtk3smp-rp2040` from commit
  `2e8e6ad480c2d84a369b12995694b9bf5e9ddfb3`. Its licences remain in
  `vendor/mtk3smp-rp2040/LICENSES`. Project additions use the `PROJECT_APP`
  and `WIFI_MQTT` build options.

The design review calls for telemetry at least 5 times per second, a 1-second
heartbeat, remote START through the Vehicle Controller, remote STOP through
`flg_estop`, and nonblocking reconnection. The firmware provides those paths;
timing and recovery still need measurement on the real robot.

## Build on Windows

Prerequisites: Git, GNU Make and host `g++` (the script defaults to
`C:\mingw64\bin\mingw32-make.exe`), and Arm GNU `arm-none-eabi-gcc`.
Levl's PC already has these. Other PCs may pass `-ArmBin` and `-MakeExe` to
the build script.

From PowerShell in this repository:

```powershell
.\scripts\setup-sdk.ps1
.\scripts\build.ps1 -Profile base
.\scripts\build.ps1 -Profile wifi
```

`setup-sdk.ps1` gets Pico SDK **2.2.0**, the version documented by the
course-linked µT-Kernel port, with TinyUSB, CYW43, and lwIP. It stays in the
Git-ignored `.tools/` directory. `base` needs no Wi-Fi details; `wifi` scans
without joining. UF2 outputs are in `vendor/mtk3smp-rp2040/build_make/`.

If PowerShell blocks a local script, use
`powershell -ExecutionPolicy Bypass -File .\scripts\build.ps1 -Profile wifi`.

We use single-core µT-Kernel (`SMP=0`) for initial integration. The existing
radio owner is priority 12 in MQTT builds, below planned motion and sensing
tasks. The small console status task is priority 15.

### Enable MQTT

Once the Wi-Fi scan test succeeds, configure the network on your own PC.
Find the broker PC's Wi-Fi IPv4 address using `ipconfig`; replace the example
address below with that address. The script prompts for your 2.4 GHz Wi-Fi
network name and password. The password stays in a local, Git-ignored header
and is never printed in the terminal:

```powershell
.\scripts\configure-local.ps1 -BrokerIp 192.168.1.100
```

The script uses WPA2 by default. Add `-WifiAuth WPA3` if the selected 2.4 GHz
network is shown as WPA3-Personal in Windows, or `-WifiAuth Mixed` for a
WPA2/WPA3 mixed network. Use the exact SSID: it is the network name, not a
username, and its capitalisation matters.

If PowerShell reports that running scripts is disabled, use this command for
this one run. It does not change the machine's execution policy:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\configure-local.ps1 -BrokerIp 192.168.1.100
```

Alternatively, copy the examples and edit the **copies** by hand:

```powershell
Copy-Item vendor\mtk3smp-rp2040\config\wifi_credentials.example.h vendor\mtk3smp-rp2040\config\wifi_credentials.h
Copy-Item vendor\mtk3smp-rp2040\config\mqtt_config.example.h vendor\mtk3smp-rp2040\config\mqtt_config.h
```

In `wifi_credentials.h`, enter a 2.4 GHz Wi-Fi SSID and password, and set
`UTK_WIFI_CREDENTIALS_SET` to `1`. In `mqtt_config.h`, enter the broker's LAN
IPv4 address and port, and set `AAI18_MQTT_CONFIG_SET` to `1`. The Pico and
broker must be on a network that allows devices to contact one another.
Both populated files are ignored by Git. Keep passwords out of commits and
messages.

```powershell
.\scripts\build.ps1 -Profile mqtt
```

This creates `mtk3pico_smp0_usb_cdc_wifi_mqtt.uf2`. Compilation proves that
the code links, not that physical Wi-Fi, MQTT, or timing goals are met.

## First hardware test (no motor battery)

1. Leave the Pico W on the Maker Pi Pico carrier. Disconnect any motor
   battery and unplug USB.
2. Hold **BOOTSEL** on the Pico W while plugging in its USB data cable.
   Windows should show the `RPI-RP2` drive as it did before.
3. Copy `vendor/mtk3smp-rp2040/build_make/mtk3pico_smp0_usb_cdc_wifi.uf2`
   to that drive. The drive disappearing after the copy is normal: the Pico
   is rebooting into the firmware.
4. Look for a new USB serial COM port in Device Manager. Open it in PuTTY or
   another serial terminal at 115200 baud. Expect an `[aai18]` line about
   every five seconds reporting radio readiness and scan completion.
5. If no COM port appears, unplug and reconnect without holding BOOTSEL.
   Send the COM port number or a screenshot of Device Manager and terminal
   output so we can diagnose it.

For the MQTT build, the USB status line also shows Wi-Fi join results. A
`link_status=-2` result means no matching Wi-Fi name was found: check the
exact spelling and capitalisation of the SSID, and make sure its 2.4 GHz
radio or phone hotspot is on and nearby. `link_status=-3` means Wi-Fi
authentication failed; re-enter the password locally. The Pico W only uses
2.4 GHz Wi-Fi. The computer running the MQTT broker must be reachable from
the Pico's network; connecting the computer to another network can change
its broker IP address.

### MQTT test after the Wi-Fi scan works

Install the official [Mosquitto Windows broker](https://mosquitto.org/download/)
on the computer connected to the same Wi-Fi LAN as the Pico. Its default
local-only listener cannot accept the Pico. For an initial **bench test with
the motor battery disconnected**, open another PowerShell window and run the
following command, replacing the IP address with that computer's Wi-Fi IPv4
address from `ipconfig`:

```powershell
.\scripts\run-broker.ps1 -ListenIp 192.168.1.100
```

If scripts are blocked, run
`powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run-broker.ps1 -ListenIp 192.168.1.100`
instead.

Leave that window open. It starts a temporary anonymous listener bound to
that IPv4 address. Add `-VerboseBroker` only when detailed connection logs
are needed; leaving it off keeps high-rate telemetry from flooding the console.
Windows Firewall may ask for permission for Mosquitto to
accept local-network connections. If port 1883 is already in use by a
Mosquitto service, stop that service before starting the bench listener.
Because this listener accepts commands from any device on the LAN, use it
only for this no-motor test. Set up broker authentication before a robot with
motors uses remote commands. See the
[Mosquitto configuration guide](https://mosquitto.org/man/mosquitto-conf-5.html).

On the broker PC, subscribe to both outputs:

```powershell
& 'C:\Program Files\mosquitto\mosquitto_sub.exe' -h 192.168.1.100 -t "aai18/robot/#" -v
```

Flash the MQTT UF2 after setting your Wi-Fi and broker details. Telemetry
should appear every 200 ms, with a heartbeat every 1000 ms. Initial telemetry
has `valid:0`, meaning the other buddies have not supplied sensor values yet.
Send exact uppercase commands:

```powershell
& 'C:\Program Files\mosquitto\mosquitto_pub.exe' -h 192.168.1.100 -t "aai18/robot/command" -m START
& 'C:\Program Files\mosquitto\mosquitto_pub.exe' -h 192.168.1.100 -t "aai18/robot/command" -m STOP
```

The USB console shows `start_seq` increasing for START and `remote_stop=1`
for STOP. The `mqtt gaps` line reports the largest gap between radio-owner
polls and counts failed connection and publish attempts. These counters help
separate firmware scheduling stalls from Wi-Fi or broker backpressure.
No motor moves in this firmware. Disconnect and reconnect Wi-Fi
and time how long telemetry takes to return. The 10-second goal remains
unverified until that physical test succeeds.

## Interfaces for the other buddies

Include `src/buddy1/state.h` from task code and add that directory to the C
include path. Call these functions **from tasks, not ISRs**:

| Owner | Interface | Data supplied |
| --- | --- | --- |
| Motion Control | `buddy1_set_motion(left_mm_s, right_mm_s, left_count, right_count)` | Speeds and encoder counts |
| Line Sensor | `buddy1_set_line(line_error)` | Latest line error |
| Barcode Decoder | `buddy1_set_barcode(code)` | Latest decoded code |
| IMU / Terrain | `buddy1_set_hump(peak_mm)` | Maximum hump height |
| Obstacle Scan | `buddy1_set_obstacle(distance_mm, width_mm)` | Distance and profile width |
| Vehicle Controller | `buddy1_set_mission(state, fault_flags)` | Mission state and faults |

Each setter briefly holds a priority-inheritance mutex. The radio owner
tries to copy the snapshot without waiting; if busy, it skips that one
publication. `valid_fields` identifies which subsystem has supplied data.

After the Vehicle Controller creates `flg_events` and `flg_estop`, it calls
`buddy1_attach_event_flags(events_id, start_bit, stop_bit, estop_id,
estop_bit)`. Remote START raises the Controller event and increments
`buddy1_start_sequence()`. Remote STOP immediately raises the e-stop flag,
raises the Controller event, and latches `buddy1_stop_latched()`. **Motion
Control must check this latch every 10 ms and brake; this hook is not yet
connected to motors.** The Controller may call
`buddy1_acknowledge_stop(buddy1_stop_sequence())` only after the robot is
safe and it is deliberately rearming.

## Checks completed and remaining

- Unmodified course µT-Kernel single-core UART and Wi-Fi/USB builds: linked.
- Group 18 base, Wi-Fi scan, and MQTT builds: linked to UF2 files.
- Host checks cover exact command parsing, malformed commands, signed
  integer limits, and output-buffer bounds.
- The software has not yet been flashed or observed on this Pico W; it was
  not connected during the builds. Network timing, recovery, and physical
  emergency-stop latency require the Pico and later motor integration.
