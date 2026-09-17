# ArduinoDriver

[![Driver CI](https://github.com/MADS-NET/ArduinoDriver/actions/workflows/driver-ci.yml/badge.svg)](https://github.com/MADS-NET/ArduinoDriver/actions/workflows/driver-ci.yml)

Turn an Arduino board into a USB vendor-class I/O peripheral — digital I/O,
analog input, PWM and DAC — and drive it from a C++20 host library built on
libusb. No serial port, no text protocol: the host talks to the board with USB
control transfers, the same way it would talk to a purpose-built USB device.

```
┌──────────────┐   libusb control transfers (EP0, vendor)   ┌──────────────────┐
│ host program │ ─────────────────────────────────────────▶ │ Arduino + UsbIo  │
│ ArduinoDriver│ ◀───────────────────────────────────────── │ sketch           │
└──────────────┘   GET_INFO / PIN_MODE / DIO_* / AI_* ...   └──────────────────┘
```

> See also [QUICKSTART.md](QUICKSTART.md) 

The repository *is* the Arduino library: `library.properties`, `src/` and
`examples/` sit at the root, so it installs and publishes like any other
library. Everything the Arduino IDE ignores lives under `extras/`.

| path | content |
|---|---|
| `src/` | the `UsbIo` Arduino library (firmware side), one folder per transport |
| `examples/UsbIoDevice/` | the example sketch: `UsbIo.begin()` / `UsbIo.poll()` |
| `src/usbio_protocol.h` | the wire protocol, shared verbatim by both sides |
| `extras/driver/` | `arduino_driver` C++20 host library, `arduino-io` CLI, Catch2 tests |
| `CMakeLists.txt` | root build: the host driver, plus `firmware-*` targets when `arduino-cli` is on PATH |
| `PLAN.md` | design record: findings in the Arduino cores, decisions, verification plan |

## How it works

Every supported core dispatches *vendor-type* control requests on endpoint 0
to code the sketch can provide, without patching the core:

| stack | hook | vendor interface |
|---|---|---|
| Renesas (UNO R4 Minima, Nano R4) | weak `tud_vendor_control_xfer_cb()` in the core's TinyUSB | no (descriptors are fixed by the core) |
| mbed (Portenta H7, GIGA R1, Nano 33 BLE, Nano RP2040 Connect) | `PluggableUSBModule::callback_request()` | yes |
| SAMD21 (Zero, MKR family, Nano 33 IoT) | `PluggableUSBModule::setup()` — the core forwards every non-standard SETUP | yes |

The USB callback runs in interrupt context on all of them, and on mbed
`analogRead()`/`analogWrite()` take a mutex that must not be used from an ISR.
The firmware therefore never touches a pin from the callback:

- **OUT requests** (`PIN_MODE`, `DIO_WRITE`, `PWM_WRITE`, `DAC_WRITE`, `RESET`)
  are validated in the ISR — pin range, capability, intended mode, value range —
  then queued; an invalid request is STALLed and the reason is kept for
  `GET_STATUS`. `UsbIo.poll()`, called from `loop()`, executes the queue.
- **IN requests** (`DIO_READ`, `AI_READ`, `*_READ_ALL`) are answered from a
  shadow copy of the pin state that `poll()` refreshes (every input pin per
  call, one analog pin per call round-robin). While queued writes are pending
  the reply says `BUSY` and the driver retries; in practice `loop()` drains the
  queue in microseconds, well inside the ≥1 ms spacing of control transfers.
- Pins are **unconfigured after boot**: the firmware only reconfigures a pin
  when the host asks (`PIN_MODE` or `RESET`), so a sketch's own use of Serial1,
  SPI or the LEDs keeps working until the host takes over.

Where the core lets the firmware add an interface, the device also exposes a
dedicated zero-endpoint vendor interface (class `FF`, subclass `49`, protocol
`4F`, string "UsbIo"). The host recognises the board from the descriptor alone,
and Windows gets a function to bind WinUSB to while the CDC serial port stays
a COM port. The CDC port is untouched everywhere, so `Serial.print()` remains
available for debugging.

## Quick start

### 1. Firmware

The whole sketch is three lines — the library does the rest:

```cpp
#include <UsbIo.h>
void setup() { UsbIo.begin(); }
void loop()  { UsbIo.poll(); }
```

First install the core for your board, once per machine — `arduino:mbed_portenta`
for a Portenta H7, `arduino:renesas_uno` for an UNO R4, `arduino:samd` for a
MKR/Zero/Nano 33 IoT (see [Board support](#board-support) for the FQBN of each):

```bash
arduino-cli core install arduino:mbed_portenta
```

Then pick one of three routes.

**Arduino IDE.** Install *UsbIo* from the Library Manager, or clone/symlink this
repository into your sketchbook `libraries/` folder, then open
*File ▸ Examples ▸ UsbIo ▸ UsbIoDevice*, select your board and press Upload.

**`arduino-cli`, from a clone.** Find the port, then compile and upload in one
step (`--library .` makes the repository itself the library):

```bash
arduino-cli board list          # -> /dev/cu.usbmodem1102, arduino:mbed_portenta:envie_m7
arduino-cli compile --fqbn arduino:mbed_portenta:envie_m7 --library . \
  --warnings all -u -p /dev/cu.usbmodem1102 examples/UsbIoDevice
```

Drop `-u -p <port>` to compile without uploading.

**CMake targets.** When `arduino-cli` is on `PATH`, the root `CMakeLists.txt`
adds one target per board — `firmware-portenta`, `firmware-giga`,
`firmware-nano33ble`, `firmware-nanorp2040`, `firmware-minima`,
`firmware-nanor4`, `firmware-mkrzero`, `firmware-nano33iot`, `firmware-zero`,
plus `firmware-all` for every core you have installed:

```bash
cmake -Bbuild -G Ninja
cmake --build build --target firmware-portenta      # -> build/firmware/portenta
```

Uploading needs the port in the CMake cache, so it is a two-step affair:

```bash
cmake -Bbuild -DUSBIO_UPLOAD_PORT=/dev/cu.usbmodem1102
cmake --build build --target upload-portenta
```

**If the upload fails to find the board**, put it in bootloader mode: on the
mbed boards (Portenta, GIGA, Nano 33 BLE) double-tap the reset button — the
LED breathes — and on SAMD boards double-tap too. The board enumerates under a
*different* port in bootloader mode, so re-run `arduino-cli board list` and use
the new one. The port also changes after a successful upload, when the board
re-enumerates running UsbIo.

**Check it worked** — this needs the host driver from step 2:

```bash
$ build/extras/driver/arduino-io info
board:            Portenta H7 (id 0x0201)
protocol version: 0x0001
pins:             26 (analog: 7)
resolution:       adc 16 bits, pwm 12 bits, dac 12 bits
voltages:         vref 3300 mV, io 3300 mV
flags:            0x000F vendor-interface pulldown streaming events
queue depth:      32
stream channels:  up to 8
stream period:    1 us or longer (up to 1000000 Hz, bounded by the sketch's loop() rate)
event pins:       up to 8
transport:        device recipient, interface 0 (claimed)
```

The `flags` line is the quickest way to confirm *which* firmware is on the
board: `streaming` and `events` appear only if it was built from a revision
that has them. The CDC serial port keeps working throughout, so `Serial.print()`
stays available for your own debugging.

### 2. Host driver

The host driver lives in `extras/driver`, where the Arduino toolchain ignores
it. Build it from the repository root:

```bash
cmake -Bbuild -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure   # unit tests, no hardware needed
build/extras/driver/arduino-io list
```

`cmake -S extras/driver -B build` configures the driver alone, without the
`firmware-*` targets.

To put `arduino-io` on your `PATH`, install it (into `/usr/local/bin` by
default; choose another prefix with `--prefix`):

```bash
cmake --install build                     # may need sudo for /usr/local
cmake --install build --prefix ~/.local   # or a user-writable prefix
```

The installed tool is self-contained: libusb is linked statically, unless
`-DARDUINODRIVER_SYSTEM_LIBUSB=ON` was given. On Linux, also install the udev
rule described in [Operating-system notes](#operating-system-notes) to use it
without root.

Dependencies (libusb via the official `libusb-cmake` wrapper, `fmt`, `cxxopts`,
Catch2) are fetched with `FetchContent` and pinned. `-DARDUINODRIVER_SYSTEM_LIBUSB=ON`
uses the system libusb-1.0 through pkg-config instead. Windows builds with
MSVC (Visual Studio 2022+) using the same CMake project.

To use the driver from another CMake project, point `FetchContent` at the
repository root — the tests and the CLI are then off by default, so a consumer
builds only the library and its two dependencies:

```cmake
FetchContent_Declare(ArduinoDriver
  GIT_REPOSITORY https://github.com/MADS-NET/ArduinoDriver.git
  GIT_TAG v0.4.0)
FetchContent_MakeAvailable(ArduinoDriver)
target_link_libraries(my_app PRIVATE ArduinoDriver::arduino_driver)
```

### 3. Talk to the board

```bash
arduino-io info                     # board, pins, resolutions, flags
arduino-io caps                     # per-pin capability table
arduino-io mode 13 output && arduino-io write 13 1
arduino-io mode 14 analog  && arduino-io aread 14 --volts
arduino-io mode 9 pwm      && arduino-io pwm 9 50%
arduino-io monitor --hz 10 14 15    # repeated control reads until Ctrl-C
arduino-io stream 15,16 --hz 1000   # continuous sampling over the bulk endpoint
arduino-io --help                   # full command list
```

`list` needs no board and also shows devices it could not probe (typically a
permissions problem) in a separate block; every other command opens the first
identified device unless `--serial` is given. Exit codes: 0 ok, 1 device or
protocol error (message on stderr), 2 usage.

From C++:

```cpp
#include <arduino_driver/Device.h>
#include <arduino_driver/Enumerator.h>

using namespace ArduinoDriver;

auto ctx = std::make_shared<Context>();
Device dev = open_first(ctx);                 // throws if no board is attached
dev.pin_mode(13, PinMode::Output);
dev.digital_write(13, true);
dev.pin_mode(dev.analog_pins().front(), PinMode::AnalogIn);
double v = dev.analog_read_volts(dev.analog_pins().front());
```

Everything is reported by exceptions derived from `ArduinoDriver::Error`
(`InvalidPin`, `InvalidMode`, `NotSupported`, `InvalidValue`, `DeviceBusy`,
`ProtocolError`, `UsbError` …). `Device` validates pins and values locally
before any USB traffic, so most mistakes fail fast with a precise message.

### Python and R

Both bindings wrap `arduino_driver_c` ([`extras/c_api`](extras/c_api)), a C
ABI over the driver above, rather than linking the C++ API directly — a
stable boundary that works across compilers/toolchains, which matters most
for R (whose Windows builds use a different compiler than this project's
usual MSVC/Clang). Both currently expose the same subset as this README:
enumeration, pin configuration, digital/analog/PWM/DAC I/O, and polling
continuous sampling; pin events and callback-based streaming are not part of
either binding yet.

Python ([`extras/python`](extras/python)):

```python
from arduino_driver import Device, PinMode

with Device.open_first() as dev:
    dev.pin_mode(13, PinMode.OUTPUT)
    dev.digital_write(13, True)
```

Install from a checkout with `pip install extras/python`, or from git with
`pip install "git+https://github.com/pbosetti/ArduinoDriver.git#subdirectory=extras/python"`.

R ([`extras/r/arduinodriver`](extras/r/arduinodriver)):

```r
dev <- arduinodriver::device_open_first()
dev$pin_mode(13, arduinodriver::PinMode$OUTPUT)
dev$digital_write(13, TRUE)
```

Install with `remotes::install_github("pbosetti/ArduinoDriver", subdir = "extras/r/arduinodriver")`.
Its `configure` script builds `arduino_driver_c` from source (CMake required)
and needs network access to fetch this repository unless
`ARDUINODRIVER_LOCAL_CHECKOUT` points at a local checkout (see
[`extras/r/arduinodriver/configure`](extras/r/arduinodriver/configure)).

## Protocol

Full specification: [`usbio_protocol.h`](src/usbio_protocol.h).
Requests are vendor control transfers with device recipient; OUT requests
carry the pin in `wIndex` and the argument in `wValue` and have no data stage.

| bRequest | dir | wIndex | wValue | data |
|---|---|---|---|---|
| `0x00 GET_INFO` | IN | – | – | 24-byte info: magic `UIO1`, protocol version, board id, pin counts, ADC/PWM/DAC bits, Vref, logic level, flags, stream channels, event pins, shortest stream period |
| `0x01 GET_PIN_CAPS` | IN | first pin | – | one capability byte per pin: DIO, AIN, PWM, DAC bits |
| `0x02 PIN_MODE` | OUT | pin | INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN, ANALOG_IN, PWM, DAC | – |
| `0x03 DIO_READ` | IN | pin | – | status, value |
| `0x04 DIO_WRITE` | OUT | pin | 0 / 1 | – |
| `0x05 AI_READ` | IN | pin | – | status, raw sample (u16) |
| `0x06 PWM_WRITE` | OUT | pin | duty, `pwm_bits` wide | – |
| `0x07 DAC_WRITE` | OUT | pin | code, `dac_bits` wide | – |
| `0x10 DIO_READ_ALL` | IN | – | – | status + bitmap, bit *i* = pin *i* |
| `0x11 AI_READ_ALL` | IN | – | – | status + u16 per analog pin, ascending pin order |
| `0x20 GET_STATUS` | IN | – | – | pending commands, reason of the last STALL |
| `0x21 GET_TIME` | IN | – | – | device `millis` and `micros` (u32 each) |
| `0x40 EVENT_CONFIG` | OUT | pin | debounce ms << 8 \| edge mode | – |
| `0x41 EVENT_POP` | IN | max events | – | header + queued edges `{pin, edge, seq, t_ms}` |
| `0x42 EVENT_COUNTS` | IN | – | – | per-watched-pin edge counters |
| `0x30 STREAM_SELECT` | OUT | pin | 0 remove / 1 add | – |
| `0x31 STREAM_START` | OUT | flags | period µs (0 = free running) | – |
| `0x32 STREAM_STOP` | OUT | – | – | – |
| `0x33 STREAM_STATUS` | IN | – | – | running, channels, period, last seq, device overruns |
| `0x7F RESET` | OUT | – | – | all pins to INPUT, queue cleared |

Status codes: `OK, BUSY, BAD_PIN, BAD_MODE, BAD_CMD, UNSUPPORTED, QUEUE_FULL,
BAD_VALUE`. An invalid OUT request STALLs (libusb reports `LIBUSB_ERROR_PIPE`)
and `GET_STATUS.last_error` tells why; IN requests never STALL, their status
byte carries the error. `GET_INFO` with `n_pins == 0` means the sketch has not
reached `UsbIo.begin()` yet — the driver waits up to 2 s for it.

An optional interface-recipient form (`bmRequestType 0x41/0xC1`,
`wIndex = pin << 8 | interface`) is accepted by boards that expose the vendor
interface; `arduino-io --interface-recipient` and
`LibusbTransportOptions::recipient` select it, for hosts that must route
through the interface.

## Continuous sampling

Control transfers cost one round trip per reading. For sustained sampling the
boards whose core lets the vendor interface own endpoints add one **bulk IN
endpoint** and push timestamped records to the host on their own. Control
transfers stay the command channel; the bulk endpoint carries samples only.

Available where `GET_INFO` reports the streaming flag — the mbed boards
(Portenta H7, GIGA R1, Nano 33 BLE, Nano RP2040 Connect) and the SAMD21 boards
(Zero, MKR, Nano 33 IoT). The Renesas boards cannot add an endpoint to their
fixed descriptors, so `arduino-io info` shows no streaming flag there and the
`STREAM_*` requests are refused; everything else works as before.

```bash
arduino-io mode 15 analog && arduino-io mode 16 analog
arduino-io stream 15,16 --hz 1000 --seconds 3 --volts --csv samples.csv
```

```
t_us,pin,volts
1043216,15,0.4121
1043216,16,1.0084
1044216,15,0.4150
…
stream: 3029 records (6058 samples) in 3.028 s (1000.4 Hz achieved per channel);
device overruns 0, seq gaps 0, host drops 0, resyncs 0, stale records 0
```

One row per sample (`t_us,pin,raw`, or `volts` with `--volts`); samples from
one record share a timestamp.

Pins must already be in `analog` or an input mode — `stream` selects channels,
it does not configure them. The summary goes to stderr, so `--csv` is optional
when redirecting stdout. From C++:

```cpp
Stream s = dev.start_stream({.pins = {15, 16}, .period = std::chrono::microseconds(1000)});
std::array<Sample, 256> buf;
while (running) {
  const std::size_t n = s.read(buf, std::chrono::milliseconds(100));
  for (std::size_t i = 0; i < n; ++i) use(buf[i].pin, buf[i].volts, buf[i].t_us);
}
// s.stats(): records_received, device_overruns, seq_gaps, host_drops, resyncs,
//            stale_records
// s.running() turns false if the stream dies on its own; s.error() says why
// the destructor stops the device stream and joins the worker
```

Each record carries the device's own `micros()` timestamp: sampling runs from
`loop()`, not from a timer interrupt, so `t_us` — not host arrival time — is
the timing reference, and the achieved rate is best-effort. Every record is
numbered, so nothing is lost silently: a device-side drop (its ring filled)
appears as a gap in `seq`, and `stats()` accounts for gaps, host-side drops and
resyncs separately. While a stream runs the `Device` refuses other calls with
`DeviceBusy`; `RESET`, a `pin_mode()` on a selected pin, and unplugging all
stop it.

**Sampling rate.** The period (`--hz` or `--period-us`, `StreamConfig::period`)
must be at least the board's `stream_min_period_us`, which `arduino-io info`
prints: 1 µs from UsbIo 0.4.0, so any rate can be requested. Firmware before
0.4.0 reports 0 there and only accepts 100 µs (10 kHz) or longer; the driver
says so instead of letting the device refuse the request. What the board
actually achieves is bounded by the sketch: it takes at most one record per
`UsbIo.poll()`, so a period shorter than one `loop()` iteration yields the
loop rate — the same as free running (`--period-us 0`), and the summary's
"achieved" rate shows it. A late record restarts the schedule from that
moment instead of being followed by a burst of catch-up records, so a hiccup
shows up as a longer gap in `t_us`.

Measured on a Portenta H7 behind a USB hub, running the plain `UsbIoDevice`
sketch, 10 s per run: free running reaches about 32 kHz with one channel and
23.5 kHz with two. A 20 kHz request is met with both (19,995 Hz on average;
records land on `loop()` iterations, so their spacing ranges from about 30 to
70 µs around the nominal 50 µs), 30 kHz is met with one channel, and a 30 kHz
request on two channels runs at the 23.5 kHz loop rate. Device overruns stayed
below 0.05 %, each run's in a single burst of a few milliseconds.

On the wire, the firmware packs whole records into each bulk packet (up to
511 bytes on a High Speed board, 63 on Full Speed) and sends a partly filled
packet at the latest 2 ms after its first record: at 10 kHz with two channels
that is ~500 transfers per second instead of 10 000, for at most 2 ms of added
latency. The host side is robust to what a real session leaves behind:

- `start_stream()` checks the device's own channel count and clears pins that
  an earlier session (another process, say) left selected; `STREAM_STOP`
  keeps the selection on the device.
- Records still waiting in the endpoint from an earlier session are
  recognised by their timestamp (older than the device clock read just before
  `STREAM_START`) and dropped, counted in `stale_records`.
- `start_stream()` also stops a stream that a session which never sent
  `STREAM_STOP` (a killed process) left running on the device.
- A failed bulk transfer ends the stream: `running()` turns false and
  `error()` says why, and `arduino-io stream` exits early printing the reason.
  Opening the device again starts a clean session.
- So does a device that stops sampling on its own: `STREAM_STOP`, `RESET` or
  `PIN_MODE` on a streamed pin from another session, an overrun with
  `StopOnOverrun`, or the firmware giving up on an endpoint the host does not
  drain. The worker sees `running == 0` in its periodic `GET_STREAM_STATUS`
  poll (every 200 ms), reads the records still in transit, then stops with
  `error()` saying so, instead of waiting for data that never comes.

Measured on a Portenta H7 behind a USB hub, once with the hub's power adapter
connected and once without: 2 channels at 10 kHz, 10 runs of 60 s each time,
about 6 million records per series, no transfer failures, device overruns
below 0.2 % per run (1.8 % in one run while a drive was being plugged into
the computer). The firmware samples at most one
record per `poll()` call, so the ceiling is the sketch's loop rate; see
[High Speed link quality](#high-speed-link-quality) if streams die with
`LIBUSB_ERROR_IO`.

## Pin events and device time

Both work on **every** board, Renesas included: they are plain control
transfers and need no endpoint.

```bash
arduino-io mode 5 pulldown
arduino-io watch 5 --edge change --debounce 20     # until Ctrl-C
arduino-io time                                     # device clock + host offset
```

```cpp
dev.configure_event(5, EdgeMode::Change, 20ms);
for (const PinEvent &e : dev.poll_events()) { ... }   // non-blocking drain
std::optional<PinEvent> ev = dev.wait_event(500ms);   // blocking with timeout

// or a callback on a worker thread; arms the pins, disarms them on destruction
EventWatcher watcher(dev, {.pins = {{5, EdgeMode::Change, 20ms}}},
                     [](const PinEvent &e) { /* runs on the worker */ });
```

Edges are found by **scanning**: `poll()` compares the digital shadow it just
refreshed against the previous one. That covers buttons and other human-scale
contacts on any DIO input pin, with no interrupt-capable-pin restriction — but
a pulse shorter than one `loop()` iteration can be missed, so it is not for
encoders or tachometers. Debounce (0–255 ms, "first edge wins") is applied on
the device; with `--debounce 0` a bouncing contact reports every edge the scan
catches.

The event queue is bounded, so a flood can drop events — but the per-pin
counters from `EVENT_COUNTS` never do. "How many presses happened" is always
answerable exactly; only "when exactly" degrades. A watcher's worker thread may
poll while your own thread drives pins: `Device` serialises control transfers
internally.

`GET_TIME` returns `millis` and `micros` together, answered in the USB
interrupt so it timestamps the request's arrival. Sending both matters:
`micros` wraps every ~71.6 minutes, but since `micros == millis × 1000 (mod
2³²)` the host rebuilds a 64-bit microsecond clock good for 49.7 days — which
is what anchors stream `t_us` and event `t_ms` to host time, to about ±RTT/2.

## Board support

| board | FQBN | status | notes |
|---|---|---|---|
| Portenta H7 (M7 core) | `arduino:mbed_portenta:envie_m7` | compile-verified; bring-up target | 26 pins: D0–D14, A0–A6 (15–21), pin 22 (A7, digital only), LEDs 23–25 (active-low). A0–A3 are ADC-only pads (no digital I/O). DAC on A6 (21). PWM on every pin the core can drive with `analogWrite()` (decided at run time from the mbed PWM pin map). ADC 16 bit, PWM/DAC 12 bit, 3.3 V |
| GIGA R1 WiFi | `arduino:mbed_giga:giga` | compile-verified | 103 pins, exactly the core's digital pin table: D0–D88 header + LEDs (86–88, active-low), then D89–D91 SPI header, D92 USB-host enable, D95–D99 Wi-Fi/BLE control, D100 BOOT0, D101–D102 SCL1/SDA1 — a `RESET` puts all of them in INPUT. AIN 76–85 (A0–A7, A12, A13), DACs on 84 and 85. ADC 16 bit, PWM/DAC 12 bit, 3.3 V |
| Nano 33 BLE | `arduino:mbed_nano:nano33ble` | compile-verified | 26 pins; AIN 14–21, LEDs 22–25 (RGB active-low, 25 = power LED), no DAC, every pin PWM-capable. ADC reported as 16 bit; the nRF52840 delivers 12 significant bits |
| Nano RP2040 Connect | `arduino:mbed_nano:nanorp2040connect` | compile-verified | RP2040 through the mbed core. 30 pins; AIN 14–17 (GPIO26–29); D24–D29 are the NINA reset/SPI/UART lines (a `RESET` puts them in INPUT); the RGB LED sits on the NINA and is not addressable; no DAC; every pin PWM-capable. ADC reported as 16 bit (12 significant) |
| UNO R4 Minima | `arduino:renesas_uno:minima` | compile-verified | 20 pins; AIN 14–19, DAC 14 (A0), PWM 0–13, 18, 19. ADC 14 bit, PWM/DAC 12 bit, 5 V. No INPUT_PULLDOWN. No vendor interface (see Windows) |
| Nano R4 | `arduino:renesas_uno:nanor4` | compile-verified | 26 pins; AIN 14–21, DAC 14, PWM 0–13, 18, 19, LEDs 22–25 |
| UNO R4 WiFi | – | **not supported** | its USB-C port belongs to the ESP32-S3; the RA4M1 USB is disabled (`-DNO_USB`) |
| Zero | `arduino:samd:arduino_zero_native` | compile-verified | 20 pins; AIN 14–19, DAC 14, PWM 3–13, 15, 16. ADC 12 bit, PWM/DAC 10 bit, 3.3 V |
| MKR Zero (and MKR family) | `arduino:samd:mkrzero` | compile-verified | 33 pins (LED = 32); AIN 15–21, DAC 15 (A0), PWM 0–8, 10, 18, 19. Pins 22–23 (USB) are excluded; the SD-card SPI lines 26–29 are addressable |
| Nano 33 IoT | `arduino:samd:nano_33_iot` | compile-verified | 31 pins; AIN 14–21, DAC 14, PWM 2–6, 9–12, 16, 17, 19, 29, 30. NINA SPI lines 22–26 are addressable |
| RP2040 / RP2350 (arduino-pico) | `rp2040:rp2040:*` | shim written, **unverified** | core not installed; see `transport/tinyusb_rp2040.cpp` |
| ESP32-S2 / S3 (arduino-esp32) | `esp32:esp32:*` | shim written, **unverified** | needs *USB Mode: USB-OTG (TinyUSB)*; see `transport/esp32_vendor.cpp` |
| Teensy 4.x, STM32duino | – | deferred | no vendor-request hook; would need a patched core |
| UNO R3, Mega, classic Nano | – | not possible | USB is a separate chip (16U2/CH340); the MCU that owns the pins has no USB |

"Compile-verified" means the example builds warning-free for that FQBN;
hardware verification is tracked per board in `PLAN.md`.

The addressable pin range of a board is the last *named* pin (digital header,
analog header, on-board LEDs) plus one, never the core's full pad table, so a
`RESET` can never reconfigure pads wired to flash or SDRAM. Where the core
itself counts radio or USB-host control lines as digital pins (GIGA D92–D102,
Nano RP2040 Connect D24–D29) they are addressable, exactly as a sketch could
`pinMode()` them.

## Operating-system notes

- **macOS** — no driver needed. `AppleUSBACM` holds the CDC interfaces, so
  libusb cannot open the device exclusively; its darwin backend tolerates
  that and still sends EP0 requests straight to the device (dfu-util programs
  the UNO R4 through the same path). Claiming the UsbIo interface is optional
  here; the driver does it by default for exclusivity between processes.
- **Linux** — copy `extras/driver/etc/99-arduino-usbio.rules` to
  `/etc/udev/rules.d/`, then `sudo udevadm control --reload && sudo udevadm trigger`,
  or run as root. The rule grants access for the Arduino, Raspberry Pi and
  Espressif vendor IDs.
- **Windows** — libusb needs WinUSB bound to a function of the device. On
  boards with the vendor interface (mbed, SAMD) use Zadig to install WinUSB on
  the interface named **UsbIo** only; the COM port keeps working. On the
  Renesas boards the core's descriptors cannot be extended, so there is no
  function to bind without breaking the serial port: Windows is not supported
  there. Automatic WinUSB binding via MS OS 2.0 descriptors is a planned
  follow-up.

## Portenta H7 bring-up checklist

1. `cmake --build build --target firmware-portenta` and upload (double-tap
   reset for the bootloader, or `upload-portenta` with `-DUSBIO_UPLOAD_PORT`).
2. `system_profiler SPUSBDataType` (macOS) or `lsusb -v` should list the board
   with an extra vendor-specific interface (class 255) besides CDC.
3. `build/extras/driver/arduino-io list` shows the board; `info` reports
   *Portenta H7*, 26 pins, 7 analog, 16/12/12 bits.
4. `arduino-io mode 23 output && arduino-io write 23 0` lights the red LED
   (active-low); `write 23 1` turns it off. Same for 24 (green) and 25 (blue).
5. `arduino-io mode 15 analog && arduino-io aread 15 --volts` on A0 with a
   known voltage (A0–A3 read up to 3.3 V through the on-board dividers; check
   the Portenta pinout for the ratio).
6. `arduino-io mode 6 pwm && arduino-io pwm 6 25%` on a header PWM pin with a
   scope or an LED.
7. `cmake -DARDUINODRIVER_HARDWARE_TESTS=ON build && ctest --test-dir build -R hardware --output-on-failure`
   runs the hidden hardware suite (set `ARDUINO_IO_LOOPBACK=out,in` with two
   wired pins for the loopback case).
8. `arduino-io info` lists the streaming flag and
   `arduino-io stream 15,16 --hz 1000 --seconds 3` then samples two analog
   pins over the bulk endpoint (set their modes first). The summary line must
   report the requested rate with zero gaps, drops and resyncs.
9. `arduino-io time` prints the device clock and the estimated host offset.
   `arduino-io mode 5 pulldown && arduino-io watch 5 --debounce 20` then
   reports one rising edge each time pin 5 is tied to 3V3 and one falling edge
   when it is released. Without `--debounce` a bouncing contact can report
   several edges per press: the scan only filters bounce shorter than one
   `loop()` iteration.

### High Speed link quality

The Portenta H7 enumerates at USB High Speed (480 Mbit/s), and its bulk
endpoint is only as reliable as the link. Plugged straight into a Mac's USB-C
port (two different cables tried), streams died every few seconds with
`bulk IN transfer failed: LIBUSB_ERROR_IO` — `device not responding`
(`0xe00002ed`) in libusb's warning log (`LIBUSB_DEBUG=2`) — and now and then
the board stopped answering control requests altogether until it was
replugged, while the sketch itself kept running. Failures scaled with the
number of bytes per packet, not with time or firmware activity: bit errors on
the link. The same board and firmware behind a **USB hub** ran 10 x 60 s at
10 kHz without a single failure, both with the hub's power adapter connected
and without it. A hub receives and retransmits every High Speed packet, so one
marginal link becomes two short ones; the link still runs at 480 Mbit/s. A
USB 3 hub works too, because the board attaches to the USB 2.0 hub built into
it. If you see these symptoms, put a USB 2.0 or USB 3 hub (external power not
needed) between the computer and the board, and prefer a short cable.

## Limitations and follow-ups

- Individual pin operations are control transfers: one per ~1 ms USB frame.
  Continuous *input* now bypasses that through the bulk endpoint (see
  [Continuous sampling](#continuous-sampling)); waveform *output* (DAC/PWM
  playback) would need a second, bulk OUT endpoint and is not implemented.
- Streaming needs an endpoint on the vendor interface, so it is unavailable on
  the Renesas boards without a patched core (`CFG_TUD_VENDOR`, fixed
  descriptors). Its rate ceiling is bounded by the sketch's loop rate, since
  the firmware emits at most one record per `poll()`; on SAMD the core's
  `USBDevice.send()` blocks until a 70 ms timeout when the host stops reading,
  so the firmware stops a stream after three consecutive failed sends rather
  than stalling `loop()` indefinitely.
- The optional per-record digital bitmap (`STREAM_START` flags bit 0) is
  carried and framed correctly but not yet surfaced through `Sample` or the
  CLI.
- `Device` is not thread-safe; use one instance per thread.
- SAMD replies are limited to 64 bytes by the core's EP0 buffer (today's
  largest SAMD reply is 33 bytes).
- Planned: MS OS 2.0 descriptors for automatic WinUSB binding; Teensy 4.x and
  STM32duino transports (patched core); RP2040 (arduino-pico) and ESP32
  verification once those cores are available.


# Author and License

Author is Paolo Bosetti, University of Trento. Distributed under the Apache 2.0 License. See [LICENSE](LICENSE) for details.
