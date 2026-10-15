# Getting Started - µT-Kernel 3.0 dual-core SMP on RP2040

From a clean machine to a Pico W running both cores. Roughly 15 minutes.

---

## 1. Prerequisites

| Need | Why |
|---|---|
| Raspberry Pi **Pico W** | The target board. RP2040, two Cortex-M0+ cores. |
| `arm-none-eabi-gcc` | Cross-compiler. Baseline used for qualification: **13.2.1**. |
| A host C++ compiler (`g++`) | Builds `elf2uf2`, the UF2 packaging tool, from source. |
| GNU `make` | The build system. This port does not use CMake or the Pico SDK build. |
| USB-serial adapter | For the UART console - GP0 TX, GP1 RX, 115200 8N1. |
| Pico SDK (**only** for `CONSOLE=usb_cdc`) | The USB console links TinyUSB from the Pico SDK. Not needed for the default UART build. |
| External LED + resistor on **GP16** | Liveness indicator. The Pico W's on-board LED is on the CYW43439 radio, not an RP2040 pin, so it is unavailable while the radio is powered down. |

On Debian/Ubuntu:

```sh
sudo apt install gcc-arm-none-eabi build-essential
arm-none-eabi-gcc --version
```

---

## 1b. Pico SDK - only if you want USB-CDC or Wi-Fi

**Skip this entirely for the default build.** The qualified UART profiles need
nothing but `arm-none-eabi-gcc` and a host `g++`. The SDK is required only for:

| Profile | Needs from the SDK |
|---|---|
| `CONSOLE=usb_cdc` | `lib/tinyusb`, plus a set of pico-sdk headers |
| `WIFI=cyw43` | `lib/cyw43-driver`, `lib/lwip`, and the `hardware_pio` / `hardware_dma` / `hardware_gpio` sources |

Note what is **not** used: this port does not use the SDK's CMake build, does
not link the pico-sdk runtime, and does not need `picotool` or the SDK's
toolchain. It compiles a handful of SDK sources and headers into its own make
build. `lib/libtm/sysdepend/pico_rp2040/usb/usb_sdk_compat.c` supplies the few
SDK hooks TinyUSB calls, mapped onto µT-Kernel's `tk_def_int()`.

### Install

Qualified against **Pico SDK 2.2.0**.

```sh
git clone -b 2.2.0 https://github.com/raspberrypi/pico-sdk.git ~/pico/pico-sdk
cd ~/pico/pico-sdk
```

Only three of the SDK's five submodules are needed - `mbedtls` and `btstack`
are not used, and skipping them saves a large download:

```sh
git submodule update --init lib/tinyusb        # for CONSOLE=usb_cdc
git submodule update --init lib/cyw43-driver   # for WIFI=cyw43
git submodule update --init lib/lwip           # for WIFI=cyw43
```

(`git submodule update --init` with no path fetches all five, which also works.)

### Point the build at it

```sh
export PICO_SDK_PATH=$HOME/pico/pico-sdk
```

Add that to your shell profile to make it permanent, or pass it per build:

```sh
make SMP=1 CONSOLE=usb_cdc PICO_SDK_PATH=$HOME/pico/pico-sdk -j8
```

> [!WARNING]
> The makefile's fallback default is the **relative** path
> `../../sdk/pico-sdk`, which is unlikely to resolve for you. Always set
> `PICO_SDK_PATH` explicitly rather than relying on it.

### Check it

```sh
echo $PICO_SDK_PATH
ls $PICO_SDK_PATH/lib/tinyusb/src/tusb.c          # CONSOLE=usb_cdc
ls $PICO_SDK_PATH/lib/cyw43-driver/src            # WIFI=cyw43
ls $PICO_SDK_PATH/lib/lwip/src                    # WIFI=cyw43
```

If a submodule was not initialized the build stops immediately and says which
one:

```
TinyUSB not found at <path>/lib/tinyusb. Set PICO_SDK_PATH, or build with CONSOLE=uart
CYW43 driver not found below <path>; initialize the pico-sdk submodules
lwIP not found below <path>; initialize the pico-sdk submodules
```

---

## 2. Clone the repo

```sh
git clone https://github.com/sirfonzie/mtk3smp-rp2040.git
cd mtk3smp-rp2040
```

---

## 3. About the UF2 converter (no action needed)

Nothing to do here - it is automatic. This section only explains what you will
see scroll past on the first build.

The RP2040 bootloader does not accept an ELF. It presents a USB mass-storage
drive and expects **UF2**: a flat format of 512-byte blocks, each carrying its
own target address, designed to survive being written by an ordinary file
copy. So the linked image has to be repackaged.

A normal Pico project gets this from the Pico SDK's CMake build, where
`pico_add_extra_outputs()` builds the converter as a host tool invisibly. This
port does not use the Pico SDK build - it inherits the upstream µT-Kernel BSP's
plain GNU make system, the same one that builds the RX and STM32 targets, which
has no notion of host helper tools.

So the build does it itself. `build_make/pico_rp2040.mk` carries a rule that
compiles `tools/elf2uf2/main.cpp` with your **host** `g++` (not
`arm-none-eabi-gcc` - this program runs on your PC, never on the Pico) and the
linked image depends on it. On the first build you will see:

```
Host tool: tools/elf2uf2
```

after which every link ends with the packaging step. The converter is rebuilt
only if its own source changes, and it survives `make clean` - it is a host
artifact, gitignored, and not portable between machines. Override `HOSTCXX`
if your host compiler is not `g++`.

---

## 4. Build - one core or two

**The build is single core by default.** `SMP` is the only switch that decides
how many RP2040 cores the kernel uses.

### Single core (default)

```sh
cd build_make
make -j8
```

Produces `mtk3pico_smp0_uart.uf2`. Core 1 is never started and the SMP
scheduler is compiled out, so the kernel behaves as the stock single-core
µT-Kernel 3.0 Pico BSP. Start here if you are unsure - it is the conservative
configuration, and it has its own 11/11 hardware regression gate.

### Dual core (SMP)

```sh
cd build_make
make SMP=1 -j8
```

Produces `mtk3pico_smp1_uart.uf2`. This starts the second core and enables the
global ready queue, the recursive big kernel lock, the IPI transport and task
affinity. Qualified at 58/58.

### Console choice, independent of core count

```sh
make SMP=1 -j8                  # UART0 console (default): GP0 TX, GP1 RX, 115200 8N1
make SMP=1 CONSOLE=usb_cdc -j8  # console over the Pico's own USB port
```

The UART console has no external dependencies. **`CONSOLE=usb_cdc` links
TinyUSB from the Pico SDK** - see [step 1b](#1b-pico-sdk---only-if-you-want-usb-cdc-or-wi-fi)
for installing it and setting `PICO_SDK_PATH`.

### Radio (development profile, not in the qualified release)

Also needs the Pico SDK - see [step 1b](#1b-pico-sdk---only-if-you-want-usb-cdc-or-wi-fi).
`WIFI=cyw43` currently requires `SMP=1`.

```sh
make SMP=1 WIFI=cyw43 -j8
```

### Artifact naming

`mtk3pico_smp<SMP>_<CONSOLE>[<wifi>].uf2` - so `make` gives you
`mtk3pico_smp0_uart.uf2`, and `make SMP=1 CONSOLE=usb_cdc` gives you
`mtk3pico_smp1_usb_cdc.uf2`.

Switching profiles needs no manual cleanup. The build stamps a profile id
into `build_make/.build_profile` and discards objects built for a different
one automatically:

```
Build profile is now smp1-uart-wifinone-...; discarding objects built for another profile.
```

---

## 5. Flash

1. Hold **BOOTSEL** on the Pico W while plugging in the USB cable.
2. Release it. A USB mass-storage drive named `RPI-RP2` appears.
3. Copy the `.uf2` onto that drive:

```sh
cp build_make/mtk3pico_smp0_uart.uf2 /media/$USER/RPI-RP2/   # single core
# or, for the dual-core build:
cp build_make/mtk3pico_smp1_uart.uf2 /media/$USER/RPI-RP2/
```

The board reboots into the new firmware as soon as the copy completes. There
is no separate flash tool and no debug probe required.

---

## 6. Watch the console

Wire the USB-serial adapter to **GP0 (TX)** and **GP1 (RX)**, ground to
ground, then:

```sh
screen /dev/ttyUSB0 115200      # or: minicom -D /dev/ttyUSB0 -b 115200
```

If you built the USB-CDC profile instead, the console arrives over the Pico's
own USB connector - no adapter needed - and enumerates as a CDC ACM device
(`/dev/ttyACM0` on Linux).

---

## 6b. What the demo does, and what you should see

The shipped demo is a small **producer/consumer pipeline** that puts one pass
through every IPC primitive the kernel offers. It is meant to be read and
modified, not just run.

```
producer  --[fixed memory pool]--> record
          --[mutex]-------------> shared sequence counter
          --[message buffer]----> consumer
consumer  --[semaphore]---------> credit returned to producer
          --[event flag]--------> monitor woken
monitor   prints a status line
blink     LED liveness, independent of all of the above
```

### The tasks

Four always, plus a fifth in radio builds.

| Task | Priority | Processor (SMP) | What it does |
|---|---:|---|---|
| `producer_task` | 5 | 1 | Takes a credit, gets a block from the memory pool, stamps a mutex-guarded sequence number, sends it through the message buffer |
| `consumer_task` | 6 | 2 | Receives, checks the payload, returns the credit, sets the event flag |
| `monitor_task` | 7 | either | Waits on the event flag and prints one status line |
| `wifi_status_task` | 8 | either | **Radio builds only** (`WIFI=cyw43`). Reports link, address and UDP echo traffic. Does not drive the radio - a service task owns that |
| `blink_task` | 10 | either | Blinks GP16. Touches nothing the console touches |

Under SMP the producer is pinned to processor 1 and the consumer to processor
2 with `TA_ASSPRC`, so **every message crosses a core boundary**. Built with
`SMP=0` the same source runs unchanged on one core.

### Which API each object demonstrates

| Primitive | Calls | Used for |
|---|---|---|
| Fixed memory pool | `tk_cre_mpf` / `tk_get_mpf` / `tk_rel_mpf` | Allocating a record without a heap |
| Mutex | `tk_cre_mtx` / `tk_loc_mtx` / `tk_unl_mtx` | The one genuinely shared counter |
| Message buffer | `tk_cre_mbf` / `tk_snd_mbf` / `tk_rcv_mbf` | Passing the record between cores |
| Semaphore | `tk_cre_sem` / `tk_wai_sem` / `tk_sig_sem` | Credit-based flow control |
| Event flag | `tk_cre_flg` / `tk_set_flg` / `tk_wai_flg` | Waking the monitor on two conditions (`TWF_ANDW \| TWF_CLR`) |
| Tasks | `tk_cre_tsk` / `tk_sta_tsk` / `tk_dly_tsk` / `tk_slp_tsk` | Creating and pacing the tasks |
| SMP | `TA_ASSPRC`, `TP_PRC1` / `TP_PRC2`, `tk_get_prc()` | Pinning, and reporting where work ran |

### What you should see

**Dual core** (`make SMP=1 CONSOLE=usb_cdc`):

```
microT-Kernel Version 3.00

=== uT-Kernel 3.0 / RP2040 demo ===
SMP build: 2 processors
producer -> [mbf] -> consumer, 2 credits, 500 ms period

[monitor] seq=2 produced=2 consumed=2 dropped=0  made_on=prc1 seen_on=prc2
[monitor] seq=6 produced=6 consumed=6 dropped=0  made_on=prc1 seen_on=prc2
[monitor] seq=10 produced=10 consumed=10 dropped=0  made_on=prc1 seen_on=prc2
[monitor] seq=14 produced=14 consumed=14 dropped=0  made_on=prc1 seen_on=prc2
```

**Single core** (`make SMP=0 CONSOLE=usb_cdc`) - same source, same pipeline,
both ends on the one processor:

```
=== uT-Kernel 3.0 / RP2040 demo ===
single-core build
producer -> [mbf] -> consumer, 2 credits, 500 ms period

[monitor] seq=2 produced=2 consumed=2 dropped=0  made_on=prc1 seen_on=prc1
[monitor] seq=6 produced=6 consumed=6 dropped=0  made_on=prc1 seen_on=prc1
[monitor] seq=10 produced=10 consumed=10 dropped=0  made_on=prc1 seen_on=prc1
```

Both captured from a Pico W over USB-CDC. The banner line and the
`seen_on=` field are the two places the configuration shows up.

Captured from a Pico W over USB-CDC.

`made_on=prc1 seen_on=prc2` is the interesting part - it is the pipeline
crossing cores. On a `SMP=0` build both read `prc1` and the banner says
`single-core build`.

### Why `seq` climbs by about 4, not exactly 4

The producer runs every 500 ms and the monitor reports every 2000 ms, so you
expect 4 records per line. Mostly you get 4, and occasionally 3.

That is not a fault - it is `tk_dly_tsk` doing exactly what it says. The call
sleeps for an interval **measured from when it is called**, so the producer's
real period is 500 ms *plus* the time it spends taking a credit, getting a
pool block, locking the mutex and sending the message: about 510 ms. Over a
capture of 13 reports that works out to 3.92 records per report, and every so
often the fourth record lands just after the monitor has printed.

This is the classic difference between **relative** and **absolute** delay. If
you need a period that does not drift, do not sleep for a fixed interval -
compute the next absolute deadline and sleep until that, so the work time is
absorbed instead of accumulated. Making that change is a good exercise: the
`seq` deltas should settle to exactly 4.

### On the USB-CDC console

With `CONSOLE=usb_cdc` the console is a 4 KB ring drained by the host, so
timing matters in a way it does not over UART:

- The monitor **waits for the host to enumerate** (`tm_usb_state() >= 2`)
before printing the banner. Anything printed earlier would go into a ring
nobody is reading and be lost. The wait is bounded at about 20 s, so a
headless board still runs the demo - you just will not see the start of it.
- Expect the banner a second or two after you open the port, not at reset.
- One status line every `REPORT_MS` cannot overrun a 4 KB ring, so drops
should never appear. If they do, the monitor says so on its own line
(`[monitor] console dropped N bytes`).
- Over UART none of this applies: `tm_printf` writes straight to the UART
from the calling context, with no ring, no service task and no interrupt -
which is exactly why it stays the qualified console while the scheduler is
under test.

### What the LED tells you

The LED on GP16 blinks at 2 Hz once USB is up (immediately on a UART build).
Before that it blinks a *count* of how far USB bring-up got - long flash = 5,
short flash = 1:

| Level | Meaning |
|---|---|
| 0 (dark) | `tusb_init()` failed |
| 1 | `tk_def_int()` rejected the USB interrupt |
| 2 | NVIC never unmasked `USBCTRL_IRQ` |
| 3 | USB controller not enabled |
| 4 | D+ pull-up not asserted, so no host can see the device |
| 5 | pull-up on, but no USB interrupt has ever fired |
| 6 | interrupts firing, but no bus reset or connect from the host |
| 7 | host talking, enumeration not completing |
| 8 | fully enumerated |

### Reading a failure

The blink task never touches the console, and that is the point:

| Symptom | Meaning |
|---|---|
| Console output **and** LED both running | Healthy |
| Console stops, **LED still blinking** | The console is at fault, not the kernel |
| **LED freezes** | A console call blocked the kernel, or the kernel is wedged |
| LED blinking a repeating count | USB never finished coming up; read it against the table above |

### What is in `app_program/`

| File | Lines | What it is |
|---|---:|---|
| `app_main.c` | 526 | The whole demo: the tasks, the kernel objects, the WiFi reporter, `usermain()` |
| `demo_tasks.c/.h` | 90 | The console-independent blink task |
| `usb_console_compat.h` | 31 | Console-state accessors, with constant stubs for UART builds |

The build picks up whatever `.c` files are in this directory
(`build_make/mtkernel_3/app_program/subdir.mk` globs them), so adding one
needs no makefile edit.

### A note on stack sizes

Every task uses 4096 bytes rather than the 1-2 KiB that sufficed before SMP.
The kernel path is deeper now: each critical section runs the global
ready-queue assignment on the calling task's stack, so any task making a
blocking kernel call needs more headroom. **Applications carried over from a
single-core µT-Kernel may need their stacks revised upward.**

### Making it yours

Everything is in one file with the tunables at the top:

```c
#define N_RECORDS   4     /* blocks in the fixed memory pool */
#define CREDITS     2     /* messages in flight at once      */
#define PRODUCE_MS  500   /* one record every half second    */
#define REPORT_MS   2000  /* monitor prints this often       */
```

Things worth trying: set `CREDITS` to 1 and watch the producer block; swap the
message buffer for a mailbox; move both tasks onto the same processor and see
whether throughput changes; remove the mutex and see whether the sequence
numbers still come out unique under SMP; replace the producer's `tk_dly_tsk`
with an absolute deadline and watch the `seq` deltas settle to exactly 4.

Keep a console-independent blink task in whatever you build - it is the only
thing that tells you a hang is the console rather than the kernel.

---

## 7. Networking (development profile)

Networking is **not** part of the qualified release. It builds on the radio
profile through additional make knobs, each reading a local configuration
header that is deliberately not committed.

```sh
cp config/wifi_credentials.example.h config/wifi_credentials.h
cp config/network_config.example.h   config/network_config.h
```

Edit each copy: fill in the values, **and change its `*_SET` flag from 0 to
1**. Each template carries a guard so an unedited copy fails at compile time
rather than building fine and then failing to associate at runtime:

```c
#define UTK_WIFI_CREDENTIALS_SET   0     /* <- set to 1 when done */
#define UTK_WIFI_SSID       "replace-with-ssid"
#define UTK_WIFI_PASSWORD   "replace-with-password"
```

Forget it and the build stops with:

```
config/wifi_credentials.h: placeholder values. Set your network SSID and
password, then set UTK_WIFI_CREDENTIALS_SET to 1.
```

```sh

make SMP=1 WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 -j8
```

Further knobs: `WIFI_STATIC`, `WIFI_DNS`, `WIFI_UDP`, `WIFI_TCP`,
`WIFI_TCPBULK` - each with its own `config/*.example.h` template. They are
cumulative: `WIFI_UDP` requires `WIFI_DNS`, and `WIFI_TCP` requires `WIFI_UDP`.

A working chain looks like this - build up one knob at a time and check each
stage on the console before adding the next:

```sh
B="SMP=1 WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1"
make $B -j8                                  # associate, get an address, resolve a name
make $B WIFI_UDP=1 -j8                       # UDP echo against a host listener
make $B WIFI_UDP=1 WIFI_TCP=1 -j8            # TCP echo
make $B WIFI_UDP=1 WIFI_TCP=1 WIFI_TCPBULK=1 -j8
```

> [!WARNING]
> `config/wifi_credentials.h`, `network_config.h`, `udp_test_config.h`,
> and `tcp_test_config.h` are gitignored on purpose.
> Never commit a populated copy.

The radio profiles work on **either** core count. `SMP=0` builds the same
networking stack with the service task unpinned - there is only one processor
to run it on. `lib/libwifi/sysdepend/pico_rp2040/smp_compat.h` supplies the
single-core equivalents of the SMP primitives the driver uses: the cross-core
spinlock becomes a PRIMASK mask/restore, the processor query returns 1, and
the memory barrier stays a real `dmb` - the ordering it needs is against DMA
and PIO, not against another CPU.

A single-core radio build has been confirmed on hardware - association, DHCP
and a full 64-packet UDP echo against a Raspberry Pi Zero 2 W, matching
64/64 with the host's own count.

Expected output once the link comes up and the session runs:

```
[wifi] link UP  rssi=-40  mac=28:cd:c1:01:23:45
[udp] waiting: link=1 netif_up=1 link_up=1 dhcp=0 addr=0.0.0.0
[wifi] ip=192.168.1.50  gw=192.168.1.1  dns=192.168.1.1
[udp] session 1: echoing to 192.168.1.10:7007, 64 packets of 48 bytes
[udp] echo #0, 48 bytes: 55 54 4b 36 00 00 40 30 e2 f3 04 15 ...
[udp] echo #1, 48 bytes: 55 54 4b 36 00 01 40 30 ff 10 21 32 ...
   ... one line per packet, #0 through #63 ...
[udp] 192.168.1.10:7007  sent=64 recv=64 matched=64/64  corrupt=0 retries=0 errs=0  1360 ms
[udp] session 1 complete: result=0  ALL PACKETS ECHOED
```

The session then repeats every 5 s. Captured from a Pico W over USB-CDC
against `tools/linux_echo.py` on a Raspberry Pi Zero 2 W.

### How the WiFi reporting works

Worth understanding before you change it, because the shape is dictated by
who owns what.

**The application does not drive the radio.** A service task owns the
CYW43439, lwIP and the echo session, and publishes a status snapshot;
`wifi_status_task` only reads that snapshot and prints. It reads through
`cyw43_utk_get_status()`, which takes a **seqlock** copy - `lwip_utk_udp_get_status()`
is a plain struct copy that is only safe inside the service itself.

**The session repeats.** It sends 64 packets, reports, waits 5 s, and runs
again, counting sessions as it goes. Set `UTK_UDP_REPEAT_MS` to 0 in
`lib/libnet/lwip/lwip_utk_udp.c` for the original single-run behaviour.

**Echoes arrive far faster than anything can print them.** At roughly 21 ms
per packet, a reporting task polling at any sane rate would miss most of
them. So the driver keeps a **32-entry ring** of matched echoes and the
reporting task drains it, tracking a monotonic cursor. It polls every 200 ms
while the link is up - about 9 new entries per drain, comfortably inside the
ring - and backs off to 3 s when there is nothing to watch. If a reader ever
does fall behind it says so rather than printing stale slots.

**Lines are composed, then printed once.** `tm_printf` is atomic per *call*,
not per line: a line built from several calls can be spliced by a
higher-priority task printing between them. Every line here is formatted into
a buffer with `tm_sprintf` and emitted with exactly one `tm_printf`. Keep that
habit in anything you add.

The `[udp] waiting:` line appears if nothing has been sent for about 5 s, and
names what is still missing. DHCP normally finishes a second or two after the
link comes up, so seeing it once at startup with `dhcp=0` is normal - the
session starts as soon as an address arrives. If it repeats forever, the field
that stays `0` is the thing to chase.

The payload begins `55 54 4b 36` - ASCII `UTK6`, the driver's magic - so you
can see at a glance that what came back is what went out.

### The other end: an echo server

The Pico is the **client**. It sends and expects the same bytes back, so the
host side is a plain echo server - nothing parses the payload.

| Profile | Protocol | Default port | Traffic |
|---|---|---|---|
| `WIFI_UDP=1` | UDP | 7007 | 64 packets x 48 bytes, stop-and-wait, 250 ms reply timeout |
| `WIFI_TCP=1` | TCP | 7008 | connect, echo, close |

[`tools/linux_echo.py`](../tools/linux_echo.py) serves both. It needs only
Python 3 - no dependencies - so a Raspberry Pi Zero 2 W, or any Linux box on
the same network, is enough:

```sh
python3 tools/linux_echo.py
```

It prints the addresses it can see. Put the one on the Pico's network into
the config before building:

```sh
cp config/udp_test_config.example.h config/udp_test_config.h
cp config/tcp_test_config.example.h config/tcp_test_config.h
# set UTK_UDP_ECHO_ADDRESS / UTK_TCP_ECHO_ADDRESS to that address
```

Then build and flash the Pico:

```sh
B="SMP=1 WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1"
make $B WIFI_UDP=1 -j8
```

The server prints a line per packet, and a summary on Ctrl+C. Useful options:
`--udp-only`, `--tcp-only`, `--quiet`, `--bind`, `--udp-port`, `--tcp-port`.

A few things that catch people out: the ports must match the config exactly;
the Pi's firewall must allow inbound 7007/7008; both devices must be on the
same subnet, since the Pico sends unicast to a literal address with no
routing; and the session has a 30 s overall timeout, so start the server
first.

`tools/windows_udp_echo.ps1` and `tools/windows_tcp_echo.ps1` are the
equivalent for a Windows host.

### No application protocol ships - that part is yours

The transports stop at UDP, TCP and bulk TCP. There is deliberately **no MQTT,
HTTP or CoAP layer** in this tree: building one on top of the working
transports is left as an exercise.

What you have to build against:

| Piece | Where |
|---|---|
| Working UDP and TCP client/server paths | `lib/libnet/lwip/lwip_utk_udp.c`, `lwip_utk_tcp.c`, `lwip_utk_tcpbulk.c` |
| The netif/DHCP/DNS bring-up they sit on | `lib/libnet/lwip/lwip_utk.c`, `lwip_utk_ipv4.c` |
| lwIP configuration | `lib/libnet/lwip/include/lwipopts.h` |
| The radio poll service that drives it all | `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` |

Two things to know before you start. **lwIP here is raw-API and
single-threaded** - it is driven from the processor-1 polling service, so
every lwIP call must happen from that context, not from an arbitrary task;
hand work across with a kernel object instead. And if you build against a
library written for lwIP's `altcp` layer, turn on `LWIP_ALTCP` in
`lwipopts.h` - it is off because nothing in the tree needs it.

The protocol libraries themselves are already on disk: the Pico SDK checkout
carries lwIP's own `src/apps/` (MQTT among them). Linking one means adding it
to `LWIP_OBJS` in `build_make/pico_rp2040.mk` - the protocol is provided, the
session driver is what you write.

The existing `lwip_utk_udp.c` is the smallest complete example of the pattern:
init, a bounded session driven from the poll loop, and a status struct the
application reads.

---

## 8. Troubleshooting

### `RPI-RP2` drive does not appear

Hold BOOTSEL *before* connecting power and keep holding until the drive
mounts. Try a different USB cable - charge-only cables have no data lines.

### `make` succeeded but there is no `.uf2`

The build skips the packaging step when `build_make/tools/elf2uf2` is missing,
and does so quietly. Normally the build creates it, so this means the host
tool failed to compile - look further up the output for an error from `g++`,
and check it is installed:

```sh
g++ --version
ls -l build_make/tools/elf2uf2
```

Note that step uses the **host** compiler, not `arm-none-eabi-g++`. Set
`HOSTCXX` if your host compiler has another name. The `.elf` from the run is
fine - only the packaging was skipped.

### `TinyUSB not found` / `CYW43 driver not found` / `lwIP not found`

The Pico SDK is missing, `PICO_SDK_PATH` is unset or wrong, or the relevant
submodule was never initialized. See
[step 1b](#1b-pico-sdk---only-if-you-want-usb-cdc-or-wi-fi):

```sh
echo $PICO_SDK_PATH                                   # set? correct?
cd $PICO_SDK_PATH && git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
```

Remember the fallback default is the relative path `../../sdk/pico-sdk`, which
probably does not resolve from wherever you cloned this - set the variable
explicitly. If you only need the UART console, build with `CONSOLE=uart`
(the default) and the SDK is not required at all.

### Nothing on the UART console

Confirm TX/RX are not swapped - the adapter's RX goes to the Pico's GP0 (TX).
Confirm a common ground. Confirm 115200 8N1. If you flashed the `usb_cdc`
profile, output goes to the Pico's USB port instead, not GP0/GP1.

### Nothing on the USB-CDC console

The banner is printed only after the host enumerates, so it will not appear at
reset - open the port and wait a second or two. If nothing ever arrives, read
the GP16 LED against the bring-up table above; it tells you how far USB got.

### No LED activity

The Pico W's on-board LED is on the radio chip and will not light in the
radio-down profiles. Use an external LED on GP16, or change `BOARD_LED_PIN`
in `include/sys/sysdepend/pico_rp2040/sysdef.h`. Avoid GP23, GP24, GP25 and
GP29 - the radio reserves them.

### Only one core seems active

Confirm which image you actually flashed - the filename is the ground truth.
`mtk3pico_smp0_*` is single core, `mtk3pico_smp1_*` is dual core. The banner
also prints `SMP build: 2 processors` or `single-core build`. The build
switches profiles on its own, so a stale object tree is not the usual cause.

---

## 9. Next steps

- [docs/PORT_RP2040.md](PORT_RP2040.md) - what is validated on hardware, what is ported but not yet validated, and what is not ported. It also carries the **RP2040 safety rules** (part of the qualification boundary, not optional advice), the **driver-ownership matrix** to consult before adding a driver, and the idle/exception model
- The µT-Kernel 3.0 specification: https://tron-forum.github.io/mtkernel_3/
