# RaceChrono DIY CAN-bus ESP32 Device — Yamaha R9 fork

A fork of [joeroback/racechrono-canbus](https://github.com/joeroback/racechrono-canbus),
retargeted from a BMW G8x to a **2026 Yamaha R9**, and turned into something that can be
read while it runs: a bench sniffer with a display, a run-time filter driven by the app,
and counters for everything it throws away.

The device reads a vehicle's CAN bus and streams raw frames over Bluetooth LE to
[RaceChrono](https://racechrono.com) on iOS/Android, which decodes them into channels.

Upstream's structure — register-level TWAI ISR, a decoder per vehicle, a
singleton-per-subsystem layout — is intact. What changed is the receive path's honesty
about loss, the BLE path's cost per frame, and the addition of a decoder that takes its
filter from the app instead of from a table.

## Disclaimer

CAN-bus is the central nervous system of a car. Modifying it in anyway can harm the car, cause things like
"check engine" lights, electrical or mechanical damage of the car or components, loss of control, injuries, or
even death.

By using any information, hardware designs or code in this project you assume any and all risk, and release any
liability from the author(s) and contributors to this project.

**Additionally**: nothing here has been connected to a motorcycle yet. Every number below
was measured on a bench, and the R9 decoder forwards frames without claiming to know what
any of them mean.

## Hardware this fork runs on

| | |
|---|---|
| Board | Adafruit Feather ESP32-S3 Reverse TFT (#5691) — 240×135 ST7789, three buttons |
| Transceiver | SN65HVD230 breakout, `CAN_RX` = GPIO18 (A0), `CAN_TX` = GPIO17 (A1) |
| Bus | 500 kbps, **listen-only**: the controller never transmits and never ACKs |
| Phone | Pixel 6, RaceChrono Pro for Android |
| Bench partner | CANable 2.0 Pro (candleLight) over a twisted pair, driven by `cangen` |

Upstream's other boards (Huzzah32, Feather S3, ESP32 Dev Module) still build; the display
compiles in only for the Reverse TFT.

## What is different from upstream

### The CAN receive path

- **Frame queue 8 → 2000 entries**, statically allocated: ~1.9 s of the R9's ~1050 msg/s,
  ~0.5 s of a saturated 500 kbps bus, so a stalled radio costs latency instead of frames.
  Upstream's 8 was about 8 ms of R9 traffic.
- **A full queue is now a drop, not a delivery.** Upstream incremented the received
  counter whether or not `xQueueSendToBackFromISR()` succeeded, so loss was invisible. The
  ISR now counts drops separately and tracks the queue's high-water mark.
- **The discarded classes are counted.** 29-bit frames and remote-transmission requests
  are still thrown away, but with tallies — a sniffer that silently discards a class of
  traffic cannot tell you afterwards that the class was empty.
- **Frames are counted per frame, not per interrupt.** One RX interrupt can carry several
  messages, which under-reported the bus exactly when it mattered.
- **The bit-rate prescaler is derived**, not taken from `TWAI_TIMING_CONFIG_*`. On ESP32
  core 3.x those macros stopped carrying a usable `brp` for the S3, which silently put the
  controller at the wrong bit rate.
- **Listen-only is set through the core 3.x API**, which takes three flags rather than a
  `twai_mode_t`, and the behaviour is demonstrated on the bench rather than assumed: the
  CANable drops to ERROR-PASSIVE with TEC 128 because nothing here ever ACKs it. The mode
  register is not read back — the registers that were read back, once, are the bit-timing
  ones.
- **The TX pad is held recessive** instead of left floating.
- **DLC is clamped to 8 in the ISR.** The wire can say 9–15; CAN 2.0 requires a receiver to
  treat that as eight bytes, and a bit error in that nibble on a vehicle harness produces
  the same thing.
- **The ISR's counters have two readers**, so `stats()` no longer zeroes them: every count
  is cumulative and monotonic, and a caller derives rates by subtracting its own previous
  sample.

### The BLE path

- **The frame path calls `ble_gatts_notify_custom()` directly.**
  `BLECharacteristic::notify()` spent three heap allocations and a mutex per frame — into a
  `String`, into another `String`, then into the mbuf the host wanted — to move twelve
  bytes that were already flat in memory. The wrapper path is still selectable with
  `RC_WRAPPER_NOTIFY` so the two can be A/B'd on one connection.
- **The drain task is off `tskIDLE_PRIORITY`.** At idle priority it round-robins with core
  0's idle task and is preempted by the display on top of that. One priority level up, with
  the loop made to block rather than spin and a one-tick yield every 128 frames, or the
  idle-task watchdog resets the board.
- **Refusals are counted and attributed.** Frames offered, notifications the controller
  actually sent, refusals, and the `esp_err_t` behind the most recent one — because
  `notify()` returns `void` and the interesting failure arrives in a callback.
- **The connection interval is requested and then read back.** The phone has the final say,
  so the negotiated value comes from `onConnParamsUpdate()` and is displayed rather than
  assumed.
- **This builds against NimBLE, not Bluedroid.** Core 3.x ships `CONFIG_NIMBLE_ENABLED` for
  the ESP32-S3 and the Arduino `BLEDevice` classes are a thin wrapper over whichever host is
  configured: different callback signatures, a 0x2902 descriptor that is a silent no-op,
  and a different failure path out of `notify()`.

### The R9 decoder and its allow-list

`src/canbus/decoder_yamahar9.cpp` has **no static ID table**, because no ID on this bike is
confirmed. It has an allow-list filled in at run time from RaceChrono's own requests: the
app sends a deny-all and then one allow per ID it has a channel for, each with the notify
interval it wants. Both are honoured, in the CAN ISR, before an unwanted frame costs a
queue slot or an mbuf.

- 32 slots, `id` + `interval_us` + `last_us`, published with release ordering so the ISR
  never sees a half-written slot.
- Gating is by elapsed time (`esp_timer_get_time()`, IRAM, a counter read), not by a frame
  divider, because the native rate of an ID on this bike is exactly what is unknown.
- **Until an app says otherwise, everything is forwarded.** That is the boot state and the
  bench-sniffer case.
- An allow-list overflow is counted, not just logged: a release build has no console, and a
  channel the rider defined that silently never arrives is the worst way for this to fail.

### The display

Three pages on the TFT, on their own task on core 0 — bus health, the ID census, the last
raw frames — repainting a character cell at a time through a change-detection cache.
Buttons: **D0** cycles panel + pixel, pixel alone, dark; **D1** cycles pages; **D2**
freezes the glass without pausing the counting.

This puts dropped frames, queue peak, refusals and the discarded 29-bit/RTR classes in
front of the rider **in a release build**, where they previously existed only in a `DEBUG`
serial log. The BUS page's top row also carries the supply voltage, read off the board's
MAX17048 fuel gauge — note that it is the BAT rail, not the 5 V coming in over USB, which
nothing on this board can see. State of charge and a charge/discharge marker are behind
`CONFIG_CELL_FITTED`, because the gauge cannot tell whether a cell is in the jack and
models the charger's rail as one when it is not.

**The on-board NeoPixel is driven as a status light**, on the same task: hue for where the
chain is (booting / advertising / configuring / recording), blips at a rate that follows
the forwarded frames, a double flash when they stop, and a latching red flash over the top
when something is lost. It never shows vehicle data — this firmware decodes nothing.

### Logging

- `logger::log()` no longer wraps `Serial.write()`/`Serial.flush()` in
  `portENTER_CRITICAL`. On this board `Serial` is USB CDC: the write spins until the *host*
  reads, for up to 250 ms, with the interrupt that would end the wait switched off, against
  a 300 ms interrupt watchdog. It was not losing the port — it was panicking and rebooting,
  invisibly, because the console default sends Guru Meditations to UART0 while `Serial` is
  TinyUSB CDC.
- `Serial.setTxTimeoutMs(20)`, so a console that is *not* reading costs a truncated line
  rather than a backlog.
- `bootln`/`errorln`/`warnln` compile in unconditionally; only `info`/`verbose`/`debug`
  become no-ops in a release build. A laptop attached in the pits arms the same paths that
  run on the bike.

## Architecture

```
      core 1                                  core 0
 ┌───────────────────────┐            ┌────────────────────────────┐
 │ TWAI ISR              │            │ drain task  (prio IDLE+1)  │
 │  · count, classify    │  frame     │  · blocking recv, 20 ms    │
 │  · drop 29-bit / RTR  │  queue     │  · ble_gatts_notify_custom │
 │  · allow-list gate ───┼──2000 ────▶│  · yield 1 tick / 128      │
 │  · clamp DLC          │  entries   │                            │
 │  · enqueue or drop    │            │ display task (prio IDLE+1) │
 ├───────────────────────┤            │  · 3 pages, cell diffing   │
 │ loop()  → stats()     │            │ NimBLE host + controller   │
 └───────────────────────┘            └────────────────────────────┘
                                                   │
                                          BLE service 0x1ff8
                                       char 0x1  frames  (notify)
                                       char 0x2  PID requests (write)
```

**All CAN activity is pinned to core 1, all Bluetooth activity to core 0** — upstream's
rule, and the ISR is installed from `setup()` on core 1 so it is serviced there.

The drain task is deliberately *level with* the display task rather than above it: under
saturation its drain loop stops exiting, and the panel is the only instrument that still
reports while that lasts.

RaceChrono's side of the protocol is two characteristics under service `0x1ff8`: the device
notifies raw frames on `0x1` (4-byte ID + up to 8 data bytes), and the app writes filter
requests to `0x2` (`DENY all`, `ALLOW all + interval`, `ALLOW ID + interval`).

## Measured

Bench: CANable 2.0 Pro generating traffic onto a twisted pair, Feather listen-only,
Pixel 6 running RaceChrono Pro. Rates below are what the CAN ISR counted, not what was
asked for.

**The BLE link is the ceiling, and it is `BLE_HS_ENOMEM`.** Frames refused by the host, as
the bus rate rises:

| Bus msg/s | 196 | 321 | 476 | 589 | 718 | 840 | 914 |
|---|---|---|---|---|---|---|---|
| refused, drain task at idle priority | 0 | 2.5% | 6.5% | — | — | — | — |
| refused, after the priority fix | 0 | 0 | 0 | 0.1% | 0.6% | 7.8% | 22% |
| frames dropped by the queue | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

The two refusal rows are not one experiment: the phone negotiated **15.00 ms** in the
before-run and **12.50 ms** in the after-run. Android decides that and was not asked to
change it. It is worth ~20% more connection events a second — some of the difference in
the table, nowhere near the whole of it.

Going direct to the host instead of through `BLECharacteristic::notify()` halves the
refusals mid-ramp and thirds them at the end (0.35% vs 0.8% at ~590 msg/s, 12% vs 35% at
~846) — that pair *was* taken at a single negotiated interval, 15.00 ms, which is what
makes it an A/B at all — but it does not move the wall. Every refusal is still the host
running out of mbufs because the radio cannot drain them at the interval the phone chose.
Cheaper work upstream of a full queue only delays filling it.

**The filter is what makes the ceiling stop mattering.** Against a *recording* RaceChrono
at a 15.00 ms interval, with three allowed IDs at the 50 ms the app asked for, plus an
unasked-for ID generated at ~470 msg/s:

| | |
|---|---|
| CAN bus | 635.8 msg/s, census sees all 4 IDs |
| over the radio | 19.6 msg/s — the three allowed IDs, at 20 Hz |
| interval accuracy | 19.4–19.6 against a requested 20 Hz, within 3% |
| dropped / refused | 0 / 0 |

**Endurance.** 600 s at 919 msg/s with no phone: no reboot, 0 dropped, queue high-water 21
of 2000, census monotonic to 699 043 frames. A second 420 s run survived a phone
disconnect, reconnect and a session-type change with no reboots. Before the logging fix,
the same load with a console attached killed the board at 2.08 s and 4.58 s over two runs.

**First phone session**, before any of the throughput work: 8018 frames, 0 lost, 0 errors,
one subscribed peer, a channel on the phone tracking a synthetic sweep.

**The display costs nothing measurable.** An idle pass is ~420 µs and a busy one 2–6 ms,
against ~250 ms for a full repaint — which happens once, at boot, into an empty cache. The
frame counters do not move when it runs.

For scale: the Yamaha R1-family bus is documented at roughly **1050 msg/s across ~16
arbitration IDs**, which is what the unfiltered ceiling above should be read against.

## Something worth knowing if you are building one of these

**What RaceChrono sends depends on what it is doing, not on how the channels are set up.**
With three CAN-Bus channels defined throughout:

| App state | What arrives on the PID characteristic |
|---|---|
| Test connection / configuring channels | `ALLOW all`, interval 50 ms |
| Recording a session | `DENY all`, then one `ALLOW ID` per channel, each with its interval |

Which makes sense once stated: while a channel is being configured the app needs every ID
on the bus to populate its live-data row; once recording, it wants only what it has
channels for. Reading a single test connection and concluding the app never sends a per-ID
list is an easy mistake to make — it was made here first.

## Not done

- **No R9 CAN IDs are confirmed.** The decoder is a pass-through; mapping IDs to signals
  needs the bike, and the bike needs a diagnostic pigtail this project does not have yet.
- **The NimBLE mbuf pool is not raised.** `CONFIG_BT_NIMBLE_MSYS1_BLOCK_COUNT` is baked
  into the Arduino core's precompiled libraries; raising it means building ESP-IDF from
  source, and the allow-list makes it moot.
- **The connection-parameter request in `src/racechrono-canbus.hpp` is the Android
  window** (7.5–12.5 ms). **Change it before pairing an iPhone**: iOS rejects the request
  outright unless min ≥ 15 ms, max ≥ min + 15 ms, and a rejected request leaves whatever
  the phone picked on its own — worse than asking for less. The iOS-safe pair is 12/24
  (15–30 ms), legal on both platforms, and is noted in the header.
- **Nothing with a cell in the JST jack has been tested**: the fuel gauge's state of
  charge and its charge/discharge marker are written but sit behind `CONFIG_CELL_FITTED`,
  and only the voltage half has been read on a board.
- Upstream's open items: histograms in `candump-parse`, BLE 4.0 vs 5.0 comparison.
  (NeoPixel support on the S3 boards was one of them and is done here.)

## Debugging

Turn on debugging messages by uncommenting out `#define DEBUG` in `src/racechrono-canbus.hpp`.
That enables the 5-second stats blocks on the serial console and per-frame logging; boot,
warning and error lines print in a release build too.

## Outline

ESP32 CAN-bus device for [RaceChrono](https://racechrono.com) on iOS/Android.

* [ESP32 Device Assembly](docs/ESP32.md)
* [CAN-bus Hacking with Raspberry Pi](docs/CANbusHacking.md)
* Vehicles
  * [Wiring Supported Vehicles](docs/WiringVehicles.md)
  * [Adding New Vehicles](docs/AddingNewVehicles.md)
* [Arduino Setup](docs/Arduino.md)

## Credits

* https://github.com/joeroback/racechrono-canbus — this fork's upstream
* https://github.com/aollin/racechrono-ble-diy-device
* https://github.com/timurrrr/RaceChronoDiyBleDevice
* https://github.com/espressif/arduino-esp32
* https://thesecretingredient.neocities.org/bmw/can/g29/
