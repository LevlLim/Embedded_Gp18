# µT-Kernel 3.0 on RP2040 - feature status

Dual-core **SMP** port of µT-Kernel 3.0 to the Raspberry Pi Pico W (RP2040, two
Cortex-M0+ cores), derived from the official µT-Kernel 3.0 BSP Pico port. This
is the only SMP port in the family - the ESP32-C3 and ESP32-C6 ports are
single-core.

This document states what works, what is implemented but not proven, and what
is absent. Each entry is one of three states:

- **Validated** - implemented and confirmed on real hardware, with the result recorded below.
- **Ported, not validated** - implemented and believed working, but without a rigorous or long-running hardware test behind it. Use with that in mind.
- **Not ported** - absent from this port. Calling it will fail, or the feature simply does not exist.

Validation dates and the profile used are given where they are known. The
phase harnesses and the per-phase hardware logs that produced these results
are not part of this release; see
[Performance numbers](../README.md#performance-numbers) in the README.

> [!IMPORTANT]
> "Qualified" in this port means a profile passed its full phase gate on
> hardware. The qualified boundary is **SMP + UART**, plus an optional
> **SMP + USB-CDC** console profile. Everything outside that boundary -
> notably the radio and arbitrary concurrent peripheral access - is a
> development profile, however far along it is.

---

## Validated

### SMP kernel core

Qualified through Phase 12: **SMP 58/58**, single-core regression **11/11**.

| Feature | Notes |
|---|---|
| Two-core symmetric scheduling | Global ready queue with per-core dispatch. Phase 6 (24/24) and Phase 7 (27/27). |
| Per-core kernel state | Separate per-core scheduler bookkeeping; Phase 2 (7/7) on both profiles. |
| Core 1 bring-up and IPI | Second core started and driven by inter-processor interrupt; Phase 3 (12/12). |
| Recursive big kernel lock | Built on RP2040 hardware spinlocks and atomics; Phase 4 (18/18) and Phase 5 (22/22). |
| Kernel objects under SMP | Semaphores, event flags, mutexes, mailboxes, message buffers, memory pools all serialized across cores; Phase 8 (34/34), both profiles. |
| Time management | Core 0 is the sole SysTick timekeeper; core 1 disables its local SysTick and the handler rejects and counts any off-core tick. Cyclic/alarm handlers support `TA_ASSPRC` processor assignment, with remote callbacks delivered by a coalesced IPI. Phase 9 (40/40). |
| Remote task management | Task operations targeting a task resident on the other core; Phase 10 (50/50). |
| Task affinity | Static assignment of a task to a processor. |
| Lock behaviour under load | 23,470 contentions at the SMP benchmark, 36 µs maximum wait, 85 µs maximum hold, zero illegal unlocks. |
| Ten-second SMP soak | 501 ownership audits, 3,785,087 core-1 lock operations, zero invariant failures. |

### Measured latency

64-sample runs on the qualified profiles.

| Path | min / avg / max |
|---|---|
| SMP local wake and dispatch | 16 / 16 / 43 µs |
| SMP cross-core wake | 17 / 17 / 24 µs |
| Single-core local wake and dispatch | 2 / 3 / 31 µs |

Idle stack high water: SMP 244/1024 bytes on core 0 and 108/1024 on core 1;
single-core 108/256 bytes.

These are observed measurements and broad bounded-response gates, **not
hard-real-time guarantees**. The recursive big kernel lock masks local
interrupts while held, so workload-specific worst-case latency still needs
application-level analysis.

### SMP scaling

Phase 13 (**65/65**) adds no kernel code and does not broaden the release
boundary. With two tasks and a 384-job workload: both workers pinned to
processor 1 took a 1,383,657 µs median; one worker per processor took
768,751 µs - **1.799x throughput, 44.4% less elapsed time**. All six trials
matched fixed golden checksums with zero API errors.

### Console

| Feature | Notes |
|---|---|
| UART console | UART0, GP0 TX / GP1 RX, 115200 8N1. The qualified console. |
| USB-CDC console | Optional qualified profile, Phase 11 (**71/71**). Dual-core print storm: two pinned tasks issued 64 framed `tm_printf` calls each from opposite processors; the host transcript contained all 128 frames, the device observed processor mask `3`, the whole-call lock recorded 111 contentions, and the ring reported zero unintended drops and a successful drain. Completed in 475,151 µs including the synchronous UART mirror. |

A later intentional overflow recorded 5,155 dropped bytes and kept running -
those drops are expected and excluded from the no-loss storm gate.

### Radio bring-up (development profile, not in the qualified release)

`make SMP=1 WIFI=cyw43` passed its **72/72** Pico W hardware gate: CYW43439
firmware boot, station interface enable, OTP MAC retrieval, and active scan,
with USB-CDC coexistence. A processor-1-owned polling task drives the radio.

A **single-core** radio build (`SMP=0 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1
WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_UDP=1`) was subsequently confirmed on
hardware: association, DHCP, and a full 64-packet UDP echo exchange against a
Raspberry Pi Zero 2 W - `sent=64 recv=64 matched=64/64 corrupt=0 retries=0
errs=0` in 1400 ms, with the host counting the same 64 datagrams. The
single-core equivalents of the SMP primitives are in
`lib/libwifi/sysdepend/pico_rp2040/smp_compat.h`.

The first run on 2026-08-16 preserved every kernel/SMP invariant but exposed
that PIO0/PIO1 and DMA had not been released from reset, so PIO-SPI bring-up
failed; the corrected run passed.

---

## Ported, not validated

| Feature | What is missing |
|---|---|
| IP networking over CYW43439 | lwIP-based DHCP, static addressing, DNS, UDP and TCP profiles exist as cumulative `WIFI_*` build knobs. Association, DHCP and the repeating 64-packet UDP echo have run end to end on hardware in **both core counts**, matching 64/64 with the host's own count across repeated sessions; static addressing and the TCP paths have had less exposure. None of it is part of the qualified release - treat as development work. |
| Inherited peripheral drivers under SMP | UART, I2C and ADC paths were converted from local interrupt masking to SMP-aware serialization - atomic mSDI `FastLock`, interrupt-preserving cross-core locks, each peripheral IRQ permanently owned by processor 1 / RP2040 core 0, auto-cleared event flags replacing sleep/wakeup handoffs. Both profiles compile and link, but **only** the monitor USB transmit path is qualified for concurrent cross-core use. |
| Long-run stability | The longest recorded soak is ten seconds. Behaviour over hours or days is uncharacterised. |

---

## Not ported

| Feature | Status |
|---|---|
| Arbitrary concurrent peripheral access | Cross-core concurrent use of the inherited UART, I2C, ADC, DMA, PWM or other drivers is outside the qualified boundary. Phase 11 qualifies only the monitor USB transmit path; treat every other shared block as single-owner. |
| Application protocols over lwIP | None ship. The transports (UDP, TCP, TCP bulk) are present and working; anything layered on top - MQTT, HTTP, CoAP - is left as an exercise. lwIP's own `apps/` sources are available in the Pico SDK checkout if you want to build against them, and `LWIP_ALTCP` in `lib/libnet/lwip/include/lwipopts.h` is the switch some of them need. |
| Rendezvous ports | `CNF_MAX_PORID=0` - compiled out. |
| mT-Kernel/DS debug support | Not present in this configuration. |
| SMP-safe flash erase/program | Not implemented. |
| Dynamic affinity | Task-to-processor assignment is static. |
| Partitioned scheduling, EDF | Not implemented. |
| Memory protection, execution budgets, temporal/fault isolation | Not implemented. |
| On-board LED | The Pico W's LED is on the CYW43439 radio, not an RP2040 pin, so the liveness indicator is an external LED on `BOARD_LED_PIN` (GP16 by default, in `include/sys/sysdepend/pico_rp2040/sysdef.h`). GP23, GP24, GP25 and GP29 are reserved by the radio. |

---

## Non-goals

These are deliberate constraints, not gaps.

- **No FPU.** The RP2040's Cortex-M0+ cores have no floating-point unit.
- **Exactly two processors.** The port assumes the RP2040's two cores; it does not generalise to more.
- **Not hard real-time.** The big kernel lock masks local interrupts while held; the measured figures are bounded-response gates, not guarantees.

---

## RP2040 safety rules

These are part of the qualification boundary, not optional tuning advice.

1. **SIO hardware spinlocks 0-2 are kernel-reserved** - 0 for kernel-lock word transitions, 1 for architecture atomics, 2 for IPI pending masks. Applications and imported SDK code must not claim them.
2. **A shared peripheral IRQ must have one owner core**, unless its driver is explicitly designed and locked for delivery to both NVICs. SysTick runs only on core 0. Each core owns its corresponding SIO FIFO IRQ for IPIs.
3. **Publish shared data before making another core observe** the associated ready flag, queue entry, or IPI. Use the port's atomic/barrier primitives; `volatile` alone is not inter-core synchronization.
4. **Never rebuild, free, or reuse a remotely running task's stack** until its processor has completed the final context save and released ownership.
5. **Any flash operation that disables or disrupts XIP must first park the other core** in SRAM-safe code. Flash erase/program was not exercised by this qualification and must not be added without that protocol.
6. **Treat serial, I2C, ADC, DMA, PWM and other shared blocks as single-owner resources** unless an SMP-safe driver supplies its own serialization and interrupt-ownership policy. The UART qualification covers the synchronous monitor path used by this release, not arbitrary concurrent driver use.
7. **Preserve the RP2040 core-1 NMI-mask/watchdog caution** from the datasheet: do not assume core 1 has the same NMI behaviour as core 0, and coordinate watchdog/reset actions across both processors.
8. **On Pico W the CYW43439 remains powered down** in the qualified profiles. GP23, GP24, GP25 and GP29 are radio-connected and reserved; the external liveness LED defaults to GP16.
9. **TinyUSB state and USBCTRL interrupt delivery are owned by processor 1** (physical core 0). Producers on either processor may use `tm_printf`, but must not call TinyUSB directly. The whole-call console lock masks local interrupts, making `tm_printf` diagnostic output rather than hard-real-time.

---

## Driver and resource ownership

"Processor 1" is the µT-Kernel processor number returned by `tk_get_prc()`; it
is physical RP2040 core 0.

| Resource | Owner | Required serialization |
|---|---|---|
| USBCTRL IRQ and TinyUSB device state | Processor 1 | Service task pinned `TP_PRC1`; USBCTRL enabled only in core 0's NVIC |
| USB transmit ring | Consumed by the USB service; produced from either processor | Whole `tm_printf` call lock, then the interrupt-preserving ring lock |
| UART0 T-Monitor mirror | Processor 1 owns configuration and direct access | The same whole-call lock serializes the UART mirror |
| Sample UART / I2C / ADC driver IRQs | Processor 1 | Atomic mSDI lock serializes requests; per-unit interrupt-preserving spinlock protects task/IRQ handoff |
| CYW43439 PIO-SPI, DMA, pins and state | Processor 1 | Service task pinned `TP_PRC1`; other tasks may only read its published status |
| SysTick / timekeeper | Processor 1 | Core 1 SysTick remains disabled |
| SIO FIFO IRQs | The corresponding processor | Core 0 owns `SIO_IRQ_PROC0`; core 1 owns `SIO_IRQ_PROC1` |
| Flash / XIP control | System-wide | Other core must complete an acknowledged SRAM-safe park protocol first |
| Watchdog / reset | System-wide | One designated control task; coordinate both processors |

---

## Idle and exception model

Each processor owns a private idle TCB and stack, outside the public task
table and ready queue. An empty processor restores that context normally and
issues `WFI` in Thread mode. SysTick and PendSV share the lowest implemented
priority (`SHPR3 = c0c00000`), so SysTick cannot interrupt a half-saved
context.

Core 0 is the sole timekeeper. Core 1 disables its local SysTick and is woken
for runnable work by the coalesced SIO FIFO IPI transport.

---

## Validation history

| Phase | Tag | Result |
|---|---|---|
| 0 - baseline import, Pico W, `.data` alignment HardFault | `phase0-baseline` | done |
| 1 - USB-CDC console | `phase1-usbcdc` | 5/5 |
| 2 - per-core kernel state | `phase2-percore` | 7/7, both profiles |
| 3 - core 1 bring-up and IPI | `phase3-core1` | 12/12 |
| 4 - atomics and big kernel lock | `phase4-bkl` | 18/18 |
| 5 - kernel critical sections | `phase5-critsec` | 22/22 |
| 6 - two-core dispatcher | `phase6-dispatch` | 24/24 |
| 7 - global ready queue and affinity | `phase7-readyq` | 27/27 |
| 8 - kernel objects under SMP | `phase8-objects` | 34/34, both profiles |
| 9 - time management | `phase9-time` | 40/40 |
| 10 - remote task management | `phase10-remotetask` | 50/50 |
| 11 - USB and driver ownership | `phase11-drivers` | optional USB profile 71/71 |
| 12 - SMP + UART qualification | `phase12-qualified` | **SMP 58/58; single-core 11/11** |
| 13 - multitask single/dual-core scaling | `phase13-multitask-scaling` | 65/65; 1.799x median speedup |
| CYW43439 radio (development) | - | 72/72 Pico W gate, 2026-08-16 |

Baseline toolchain: `arm-none-eabi-gcc` 13.2.1. Embedded behaviour depends
heavily on board revision, toolchain version and usage pattern, so if you hit
something these runs did not cover, please open an issue with the console
capture, your toolchain version, and your board.
