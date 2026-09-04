# ArduinoDriver — USB vendor-class I/O device + libusb host driver

## Context

Goal: turn an Arduino board into a USB peripheral controlled by a userland libusb driver (no serial layer), exposing DIO read/write, analog read, PWM write (plus DAC where the board has one).

Findings from the installed cores (`~/Library/Arduino15/packages/arduino/hardware/`) that shape the design:

- **Vendor control requests on EP0 reach sketch code with zero core patches** on three stacks:
  - Renesas (Uno R4 Minima, Nano R4): `tinyusb/device/usbd.c:624` dispatches type=VENDOR to weak `tud_vendor_control_xfer_cb()` — undefined by the core. Descriptors are hardcoded (`usb/USB.cpp`), `bcdUSB=0x0200`, no vendor interface can be added.
  - SAMD21 (Zero, MKR*, Nano 33 IoT): `USB/USBCore.cpp:373` routes every non-standard setup to `PluggableUSB().setup()`; a `PluggableUSBModule` (`api/PluggableUSB.h:29`) gets it. `USBDevice.sendControl()` for IN data. Module may add its own interface.
  - mbed (Portenta H7 `envie_m7`; Giga R1 / Nano 33 BLE via `mbed_giga` / `mbed_nano`): `USB/PluggableUSBDevice.cpp:135` loops modules' `callback_request(setup,&result,&data)`; `PassThrough` continues, `Send`/`Success`/`Failure` completes. Modules plug in their constructor (before `main.cpp:40 PluggableUSBD().begin()`), and own a `configuration_desc()` → we can add a dedicated vendor interface.
- **All three callbacks run in ISR context** (Renesas `USB.cpp:296` calls `tud_task()` from the USB IRQ; SAMD `USB_Handler`; mbed USB IRQ). On mbed, `analogRead`/`analogWrite` lock a `PlatformMutex` (`AnalogIn`/`PwmOut`) → illegal in ISR. Therefore: **commands execute in `loop()`, the ISR only enqueues / serves cached state.**
- Not supportable: Uno R4 WiFi (`-DNO_USB`), boards with a separate USB-serial chip. Deferred (need patched core): Teensy 4.x, STM32duino.
- User hardware for bring-up: **Portenta H7** (mbed transport is verified on hardware first). No Minima yet.
- Toolchain: cmake 4.2, ninja 1.13, Apple clang 21, arduino-cli 1.0.2 (`core list` hangs on index refresh — use `--no-update`/local index). libusb 1.0.30 via Homebrew available as fallback.
- Cores to install (approved): `arduino:mbed_giga`, `arduino:mbed_nano`. RP2040 / ESP32 shims are written but **not compile-verified**.

## Repository layout

Restructured on 2026-09-03 so the repository root *is* the Arduino library
(`library.properties`, `src/`, `examples/` at the top), which is what the
Arduino Library Manager indexes. The host driver moved to `extras/`, a
directory the Arduino tooling ignores by specification, so it is never
compiled into a sketch. `CMakeLists.txt` stays at the root so the driver is
still consumable with `FetchContent(GIT_REPOSITORY ...)` unchanged. Sections
below that predate the move still name the old paths (`arduino/UsbIo/**` for
the library, `driver/**` for the host side); they are kept as written, since
they record what was done at the time.

```
ArduinoDriver/                   # this repository IS the UsbIo library
  library.properties
  CMakeLists.txt                 # root: add_subdirectory(extras/driver) + optional firmware-* targets (arduino-cli)
  README.md                      # protocol, per-board notes, bring-up checklist, udev/Windows notes
  src/usbio_protocol.h           # C-compatible, freestanding: request codes, structs, status codes, board ids — SINGLE SOURCE OF TRUTH
  src/UsbIo.h / UsbIo.cpp        # board-agnostic: command queue, shadow state, capability table, dispatcher
  src/UsbIoBoard.h               # per-arch capability discovery (pin counts, digitalPinHasPWM, DAC pins, adc/pwm bits, vref)
  src/transport/UsbIoTransport.h        # tiny internal interface: handle_setup(request) → {action, data, len}
  src/transport/tinyusb_renesas.cpp     # #if ARDUINO_ARCH_RENESAS_UNO   — tud_vendor_control_xfer_cb
  src/transport/tinyusb_rp2040.cpp      # #if ARDUINO_ARCH_RP2040        — same weak callback (UNVERIFIED)
  src/transport/esp32_vendor.cpp        # #if ARDUINO_ARCH_ESP32 && ARDUINO_USB_MODE — USBVendor::onRequest (UNVERIFIED)
  src/transport/pluggable_samd.cpp      # #if ARDUINO_ARCH_SAMD          — PluggableUSBModule w/ vendor interface + bulk IN
  src/transport/pluggable_mbed.cpp      # #if ARDUINO_ARCH_MBED          — internal::PluggableUSBModule w/ vendor interface + bulk IN
  examples/UsbIoDevice/UsbIoDevice.ino  # setup(){UsbIo.begin();} loop(){UsbIo.poll();}
  extras/driver/                 # host side; ignored by the Arduino toolchain
    CMakeLists.txt               # project ArduinoDriver, C++20, FetchContent: libusb-cmake, fmt, cxxopts, Catch2 v3
    include/arduino_driver/{Protocol.h,Errors.h,Transport.h,LibusbTransport.h,Enumerator.h,Device.h,Stream.h}
    src/{LibusbTransport.cpp,Enumerator.cpp,Device.cpp,Stream.cpp}
    tools/arduino-io.cpp         # CLI (cxxopts + fmt)
    tests/{CMakeLists.txt,FakeTransport.h,FakeTransport.cpp,test_*.cpp}
    etc/99-arduino-usbio.rules   # Linux udev
```

Naming per user style: namespace `ArduinoDriver`, CamelCase classes, snake_case methods, `_private` members, 2-space LLVM style. Firmware global instance `UsbIo` (Arduino idiom), class `UsbIoDevice`.

## Protocol (`usbio_protocol.h`)

`bmRequestType = 0x40` (OUT) / `0xC0` (IN): vendor, **device recipient** (works on bare EP0 for Renesas, and through the dedicated interface elsewhere). OUT requests carry everything in `wValue`/`wIndex`, `wLength=0` — no OUT data stage on any stack. Pin numbers are Arduino pin numbers.

| bRequest | dir | wIndex | wValue | data |
|---|---|---|---|---|
| `0x00 GET_INFO` | IN | – | – | `usbio_info_t` {magic "UIO1", proto_ver u16, board_id u16, n_pins u8, n_ain u8, adc_bits, pwm_bits, dac_bits, vref_mv u16, flags u16} |
| `0x01 GET_PIN_CAPS` | IN | first pin | – | 1 byte/pin: bit0 DIO, bit1 AIN, bit2 PWM, bit3 DAC |
| `0x02 PIN_MODE` | OUT | pin | mode: INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN, ANALOG_IN, PWM, DAC | – |
| `0x03 DIO_READ` | IN | pin | – | {status u8, value u8} |
| `0x04 DIO_WRITE` | OUT | pin | 0/1 | – |
| `0x05 AI_READ` | IN | pin | – | {status u8, raw u16} |
| `0x06 PWM_WRITE` | OUT | pin | duty (pwm_bits) | – |
| `0x07 DAC_WRITE` | OUT | pin | value (dac_bits) | – |
| `0x10 DIO_READ_ALL` | IN | – | – | {status, bitmap[(n_pins+7)/8]} |
| `0x11 AI_READ_ALL` | IN | – | – | {status, u16 × n_ain} in ascending pin order |
| `0x20 GET_STATUS` | IN | – | – | {status, queue_pending u8, last_error u8} — the sync primitive |
| `0x7F RESET` | OUT | – | – | all pins → INPUT, queue cleared |

Status codes: `OK, BUSY, BAD_PIN, BAD_MODE, BAD_CMD, UNSUPPORTED, QUEUE_FULL`. Semantics: OUT requests are validated in the ISR (pin range + capability bit); invalid → **STALL** (host sees `LIBUSB_ERROR_PIPE`), valid → queued + ACK. IN reads serve the **shadow state** refreshed by `poll()`; if writes are still pending they return `BUSY` and the driver retries with a bounded backoff (in practice never surfaces: `loop()` drains in µs, control transfers are ≥1 ms apart). Multi-packet IN (`GET_PIN_CAPS`, `AI_READ_ALL`) is handled by all three stacks' control-transfer helpers.

Dedicated vendor interface (SAMD, mbed, RP2040, ESP32): class `0xFF`, subclass `0x49`, protocol `0x4F`, zero endpoints. Purpose: lets the driver identify devices from the config descriptor without probing, and gives Windows a function to bind WinUSB to (Zadig now; MS OS 2.0 descriptors as follow-up) while leaving the CDC COM port intact. Renesas can't add one → identified by VID/PID allowlist + `GET_INFO` probe; **Windows unsupported on Renesas** without core patch (documented).

## Firmware design (`UsbIo.cpp`)

- `begin()`: computes capability table at boot from core facilities — `NUM_DIGITAL_PINS`, `NUM_ANALOG_INPUTS`, `digitalPinHasPWM()` (Renesas `pins_arduino.h:93`, SAMD), mbed `pinmap_peripheral(pin, PinMap_PWM) != NC`, `DAC`/`IS_DAC` macros (Renesas `MINIMA/pins_arduino.h:24`, mbed `DAC` = A6 on Portenta), then sets `analogReadResolution(adc_bits)` / `analogWriteResolution(pwm_bits)` (RA4M1 14/12, SAMD21 12/10 (verify), H7 16/12) and starts the transport.
- ISR path (`handle_setup`): decode → validate → for OUT push `{cmd,pin,value}` onto a SPSC ring buffer (power-of-two, `volatile` head/tail — safe on Cortex-M); for IN copy from shadow arrays into a static response buffer. No Arduino I/O calls in ISR.
- `poll()` (from `loop()`): drain queue executing `pinMode/digitalWrite/analogWrite`; refresh shadow: `digitalRead` for every non-output DIO pin, round-robin `analogRead` for pins in ANALOG_IN mode. Shadow entries are byte/halfword → atomic reads from ISR; `AI_READ_ALL` may mix samples of different ages (documented).
- Transport files are compiled conditionally by arch macro; each implements only descriptor glue + the setup hook and calls `UsbIo.handle_setup()`. Templates to follow: mbed `libraries/USBHID/src/PluggableUSBHID.h` (module ctor, `configuration_desc`, `string_iinterface_desc`, `callback_request` returning `Send`/`Success`/`PassThrough` with a static buffer), SAMD `api/PluggableUSB.h` (ctor `(numEps=0, numIfs=1, epType)`, `getInterface`, `setup`, `USBDevice.sendControl`).
- Optional `Serial` debug stays functional on every board (CDC untouched).

## Driver design (C++20, exceptions)

- `Errors.h`: `Error` (base, `std::runtime_error`) → `UsbError{libusb code}`, `ProtocolError`, `DeviceBusy`, `InvalidPin`, `NotSupported`.
- `Transport.h`: abstract `control_in(request, value, index, std::span<std::byte>, timeout) -> size_t` / `control_out(...)`. `LibusbTransport` wraps `libusb_device_handle` (RAII, claims the vendor interface when present, `LIBUSB_ERROR_PIPE` → `ProtocolError`/`InvalidPin`).
- `Enumerator.h`: `list_devices()` → `DeviceInfo{vid,pid,serial,bus,address,has_vendor_interface,board_id?}`; identifies by (a) config descriptor vendor interface `FF/49/4F`, else (b) VID allowlist (Arduino `0x2341`, RPi `0x2E8A`, Espressif `0x303A`) + `GET_INFO` probe; user-supplied VID/PID override.
- `Device.h`: `open(DeviceInfo|serial)`, `info()`, `pin_caps(pin)`, `pin_mode`, `digital_read/write`, `analog_read` (raw + `analog_read_volts` via `vref_mv`), `pwm_write` (raw + `pwm_write_fraction(0..1)`), `dac_write`, `read_all_digital`, `read_all_analog`, `sync()`, `reset()`. Validates pin against caps before touching USB. BUSY → bounded retry (configurable). Not thread-safe (documented).
- CLI `arduino-io`: `list | info | caps | mode <pin> <m> | read <pin> | write <pin> <v> | aread <pin> | pwm <pin> <duty> | dac <pin> <v> | monitor [pins] [--hz]`. Uses `fmt` for output, `cxxopts` for parsing; exits non-zero with the exception message.
- CMake: `ARDUINODRIVER_SYSTEM_LIBUSB` (default OFF → FetchContent `libusb/libusb-cmake`; ON → `pkg_check_modules(libusb-1.0)`), `ARDUINODRIVER_BUILD_TESTS` (ON), `ARDUINODRIVER_BUILD_CLI` (ON). Root `CMakeLists.txt` adds `firmware-<board>` custom targets when `arduino-cli` is found (`arduino-cli compile --fqbn … --libraries arduino --build-path build/firmware/<board> arduino/UsbIo/examples/UsbIoDevice`).

## Tests (Catch2 v3, no hardware)

`FakeTransport` implements the protocol in-process (mirrors firmware semantics: pin table with caps, mode enforcement, STALL on bad pin, queue + BUSY injection, multi-packet responses) so tests exercise real request encoding/decoding:
- `test_protocol_encoding`: bRequest/wValue/wIndex/direction/length for each call; little-endian struct decoding of `GET_INFO`.
- `test_device_api`: caps-based validation (`InvalidPin`, `NotSupported` for PWM on non-PWM pin), volts/fraction scaling for 10/12/14/16-bit boards, `read_all_*` packing, BUSY retry then success, BUSY exhaustion → `DeviceBusy`, STALL → `InvalidPin`.
- `test_enumeration`: descriptor-based identification vs probe fallback using a fake device list.
- `test_hardware` (`[.hardware]`, hidden by default): open first device, toggle a pin, read it back, ADC sanity — run by the user on the Portenta.

## Execution strategy (per user)

1. This plan is copied verbatim to `PLAN.md` in the project root.
2. I write the shared contract first, before any delegation: `arduino/UsbIo/src/usbio_protocol.h`, root `CMakeLists.txt`, `.gitignore`, README skeleton — so both agents build against one fixed protocol.
3. Two Fable sub-agents run in parallel, each on its own subtree:
   - **firmware agent** → `arduino/UsbIo/**` (steps 2–4 below)
   - **driver agent** → `driver/**` (steps 5–6 below)
   Each agent works in **incremental steps** and stops after every step with a report; I review the files it produced (style, correctness against the protocol, portability), send corrections/improvements via SendMessage, and only then release the next step. Neither agent edits `usbio_protocol.h` — protocol changes go through me and are applied to both sides.
4. I write step 7 (README/docs) myself at the end, from both agents' final reports.

## Implementation order

1. Scaffold + `usbio_protocol.h` (+ `.gitignore` additions, README skeleton).
2. Firmware core (`UsbIo.h/.cpp`, `UsbIoBoard.h`) + example sketch.
3. Transports: **mbed first** (user's hardware), then Renesas, SAMD; then RP2040 and ESP32 shims marked UNVERIFIED in-file.
4. Install `arduino:mbed_giga`, `arduino:mbed_nano`; compile-check FQBNs `arduino:mbed_portenta:envie_m7`, `arduino:mbed_giga:giga`, `arduino:mbed_nano:nano33ble`, `arduino:renesas_uno:minima`, `arduino:renesas_uno:nanor4`, `arduino:samd:mkrzero`, `arduino:samd:nano_33_iot`, `arduino:samd:arduino_zero_native`.
5. Driver library + CLI + CMake (build with Ninja/clang).
6. Tests + FakeTransport; `ctest`.
7. README: protocol table, per-board matrix (verified / compile-only / unverified / deferred), Linux udev rule, macOS notes, Windows (Zadig on the vendor interface; Renesas limitation), Portenta bring-up checklist (LEDs are active-low; `LEDR=23`), follow-ups (Teensy/STM32 patched cores, MS OS 2.0, bulk streaming).

## Verification

- `cmake -Bbuild -G Ninja && cmake --build build && ctest --test-dir build --output-on-failure` — all non-hardware Catch2 tests pass on macOS/clang.
- `cmake --build build --target firmware-portenta` (and the other Tier-1 FQBNs) — clean compile, no warnings from our library.
- Hardware (user, Portenta H7): flash example → `system_profiler SPUSBDataType` shows an extra vendor interface; `build/driver/arduino-io list` finds it; `info` prints board id/pins; `mode 23 output && write 23 0` lights the red LED; `aread 15` (A0) returns a plausible value; `pwm` on a PWM-capable pin; `ctest -R hardware` with `ARDUINO_IO_HW=1`.

## Status (2026-09-02)

- Firmware: complete. Nine FQBNs compile warning-free (Portenta H7, GIGA R1,
  Nano 33 BLE, Nano RP2040 Connect, UNO R4 Minima, Nano R4, Zero, MKR Zero,
  Nano 33 IoT); RP2040 (arduino-pico) and ESP32 shims are written but
  unverified; Teensy/STM32 deferred. Recipient policy (device vs interface
  form) lives in `UsbIoDevice::handle_setup()`; transports pass raw packets.
- Driver: complete. `Device` over an abstract `Transport`, `LibusbTransport`
  (+ interface-recipient option), `Enumerator` (descriptor match, then
  VID-allow-listed GET_INFO probe), `arduino-io` CLI, Catch2 suites on an
  in-process firmware model, hidden `[.hardware]` suite.
- Hardware verification: **Portenta H7 done** — the README bring-up checklist
  passes end to end on the user's board (enumeration with the extra vendor
  interface, `list`/`info`, LED writes, analog read, PWM, hardware test suite).
  Findings from further boards go here.
- Deviations from the plan above: pins are unconfigured at boot (not INPUT);
  `GET_INFO` with `n_pins == 0` means "sketch not started"; `last_error`
  records IN STALLs too; the mbed/SAMD vendor interface lands at interface 0
  with CDC at 1-2; GIGA exposes the core's full 103-pin digital table.
- Next: phase 2 below (bulk endpoints for continuous sampling) — implemented
  and hardware-verified, see its own status entry at the end.

## Phase 2 — bulk endpoints for continuous sampling

Goal: sustained, device-timestamped sampling of a selected pin set, streamed
over a bulk IN endpoint instead of one control transfer per reading (today's
ceiling: one operation per ~1 ms frame). Control transfers stay the command
channel; the bulk endpoint carries sample records only.

### Feasibility per transport

- **mbed** (Portenta H7 — verified hardware, GIGA, Nano 33 BLE): the module's
  `init(EndpointResolver &)` — today an empty override in
  `transport/pluggable_mbed.cpp:117` — claims
  `resolver.endpoint_in(USB_EP_TYPE_BULK, 64)`, and `configuration_desc()`
  grows one endpoint descriptor with `bNumEndpoints` 0 → 1. Writes happen from
  `poll()` through the module's non-blocking send, never from the setup
  callback. **Tier 1: implement and verify here first.**
- **SAMD21** (Zero, MKR*, Nano 33 IoT): module ctor becomes
  `PluggableUSBModule(1, 1, epType)` with `EP_TYPE_BULK_IN`, `getInterface()`
  emits the endpoint descriptor, data goes out via `USBDevice.send()`. One
  64-byte packet per frame → ~64 kB/s ceiling. Tier 2.
- **Renesas** (Uno R4 Minima, Nano R4): descriptors are hardcoded in the core's
  `usb/USB.cpp` and there is no vendor interface to hang an endpoint on →
  **not supportable without a patched core**. Firmware simply reports no
  streaming flag; the driver reports `NotSupported`.
- **RP2040 / ESP32**: transports are still unverified for control I/O;
  streaming deferred until those are validated on hardware.

### Protocol additions (no version bump)

Streaming only adds request codes, one info flag and one reserved byte, so
`USBIO_PROTOCOL_VERSION` stays `0x0001`: `Device.cpp:100` requires an exact
version match, so bumping it would break every existing firmware/driver pair
for a purely additive feature. Capability discovery goes through the flag, and
firmware without streaming STALLs the new requests with `BAD_CMD` anyway.

| bRequest | dir | wIndex | wValue | data |
|---|---|---|---|---|
| `0x30 STREAM_SELECT` | OUT | pin | 0 remove / 1 add | – |
| `0x31 STREAM_START` | OUT | flags | period_us (0 = free-run) | – |
| `0x32 STREAM_STOP` | OUT | – | – | – |
| `0x33 GET_STREAM_STATUS` | IN | – | – | `usbio_stream_status_t` {status, running u8, n_channels u8, period_us u16, seq u32, overruns u32} |

- The channel set is built one pin per request so that no request needs an OUT
  data stage — the constraint that shaped the whole protocol. Selected pins
  must already be in `ANALOG_IN` or a DIO input mode (else STALL `BAD_MODE`);
  `STREAM_SELECT` while running STALLs `BUSY`.
- `wValue = period_us` (1..65535 µs ≈ 15 Hz..1 MHz nominal). The rate is
  best-effort; what the device actually achieved is read back through
  `GET_STREAM_STATUS` and `t_us` deltas.
- `STREAM_START` flags: bit0 append the digital bitmap to every record,
  bit1 stop on overrun instead of dropping.
- New `USBIO_FLAG_STREAMING = 1u << 2`; `usbio_info_t.reserved[0]` becomes
  `stream_max_channels` (0 when the build has no streaming), keeping the
  struct at 24 bytes.

### Record format (bulk IN, little-endian)

```
usbio_stream_record_t {
  uint16_t magic;      /* 'US' — lets the host resync after a loss */
  uint16_t n_samples;  /* channels in this record                  */
  uint32_t seq;        /* record counter; gaps == device drops     */
  uint32_t t_us;       /* micros() at sample time                  */
  uint16_t samples[];  /* one per selected pin, in selection order */
}
```

The endpoint carries a continuous byte stream: the firmware writes whole
packets, records may straddle packet boundaries, and the host reassembles.
`seq` gaps are how the host detects device-side drops, `magic` allows resync
after a host-side loss. The digital bitmap (flags bit0) is appended after
`samples`.

### Firmware

- Sampling stays in `poll()` — the ISR rule is unchanged. A `micros()` deadline
  scheduler samples the selected pins, formats one record into an SPSC ring
  (~2 kB), then pushes as many whole packets as the endpoint accepts without
  blocking. Ring full → drop the newest record and `overruns++` (the seq gap
  tells the host what was lost).
- Consequence to document: jitter is bounded by the sketch's loop time, and
  `t_us` — not host arrival time — is the timing reference. Timer/DMA-driven
  sampling is explicitly out of scope for this phase.
- `RESET`, USB suspend and disconnect stop the stream and clear the selection.

### Driver

- `Transport` gains `bulk_in(std::span<std::byte>, timeout) -> size_t`;
  `LibusbTransport` locates the bulk IN endpoint in the vendor interface
  descriptor at open time (`Enumerator` already walks config descriptors) and
  records its address and max packet size.
- New `Stream.h`: `Device::start_stream(StreamConfig{pins, period, flags})`
  returns an RAII `Stream` that stops the device in its destructor. `Stream`
  runs a worker thread over the libusb **async** API — a ring of ~8 in-flight
  `libusb_submit_transfer` of 8–16 packets each, driven by
  `libusb_handle_events_timeout` — because synchronous reads leave the endpoint
  idle between calls and lose samples at rate.
- Consumer API: `read(std::span<Sample>, timeout) -> size_t` for decoded
  records plus an optional `on_records` callback; `Stream::stats()` reports
  device overruns, host-side drops and seq gaps. `Sample` carries pin, raw,
  volts and `t_us`.
- Threading: `Device` stays not-thread-safe. A running `Stream` owns the bulk
  path; while it runs only `GET_STREAM_STATUS`/`STREAM_STOP` are allowed
  (serialized inside `Device`), other calls throw `DeviceBusy`.
- CLI: `arduino-io stream <pins> [--hz N | --period-us N] [--seconds N]
  [--volts] [--csv FILE]`, CSV on stdout plus a final summary line with the
  achieved rate and the drop counts.

### Tests

- `FakeTransport` grows a bulk endpoint model: synthetic records with a
  programmable ramp, injectable seq gaps, short reads, records straddling
  packet boundaries, and zero-length packets.
- `test_stream_decoding`: framing, straddling reassembly, resync via `magic`
  after injected garbage, seq-gap accounting, digital-bitmap layout.
- `test_stream_api`: select/start/stop sequencing, `BAD_MODE` on an
  unconfigured pin, `DeviceBusy` for control calls during a stream,
  `NotSupported` when the info flag is clear, destructor stops the device.
- `[.hardware]`: 4 channels at 1 kHz for 10 s on the Portenta — zero seq gaps,
  `t_us` deltas within tolerance, achieved rate close to requested.

### Implementation order

1. Protocol header additions (written once, applied to both sides, as before).
2. mbed firmware: endpoint, ring, scheduler; confirm enumeration still shows
   CDC plus the vendor interface and that existing control I/O is unaffected.
3. Driver: `bulk_in`, endpoint discovery, `Stream` + async worker, CLI verb.
4. Tests against the fake bulk model, then the hardware run on the Portenta.
5. SAMD port; re-run the FQBN compile matrix; document the 64 kB/s ceiling.
6. README: streaming section, board-matrix column, Renesas exclusion.

### Open questions

- Actual rate ceiling on the H7 poll loop (cost of `analogRead` per channel) —
  measure before quoting a number in the README.
- A second bulk OUT endpoint for waveform output (DAC/PWM playback) would use
  the same framing in reverse; deliberately deferred until the IN path is
  verified on hardware.

### Status (2026-09-03)

Implemented as planned: the contract in `usbio_protocol.h` was frozen first,
then the two halves were written in parallel by sub-agents on disjoint
subtrees (firmware `arduino/UsbIo/**`, driver `driver/**`), neither touching
the protocol header, and both reviewed and corrected here.

- Firmware: streaming state machine, `micros()` deadline scheduler and a
  32-slot record ring in `poll()`; mbed bulk IN endpoint (Tier 1), SAMD
  (Tier 2). All nine FQBNs compile warning-free, the Nano RP2040 Connect
  (mbed, so it gains streaming too) checked separately. Portenta +1800 B flash /
  +1544 B RAM; UNO R4 Minima +32 B / +0 B, with `nm` confirming zero stream
  symbols in the Renesas image (the feature genuinely compiles out).
- Driver: `Transport::bulk_in`, bulk endpoint discovery, an 8-transfer
  always-armed libusb async ring, `Stream` + worker thread with resync-capable
  reassembly, the `stream` CLI verb. 60/60 tests pass (15 new) with
  warnings-as-errors, and clean under TSan and ASan/UBSan.
- Hardware (Portenta H7): 2 channels at 1 kHz for 3.028 s — 3029 records,
  6058 samples, 1000.4 Hz achieved per channel, zero device overruns, seq
  gaps, host drops and resyncs. First end-to-end proof of the bulk path.

Deviations from the phase 2 design above:

- Records never straddle packets in practice: the channel and pin limits bound
  one at 44 B, under the 64 B endpoint (compile-time asserted), so the device
  emits whole records and its ring holds records rather than raw bytes. The
  host reassembler still implements straddling, as the wire contract allows it.
- The transport hook returns SENT/BUSY/FAILED, not a bool: on mbed a refused
  write is ordinary backpressure, while on SAMD it means the core's blocking
  `send()` burned its 70 ms timeout. Conflating them would either kill healthy
  streams or stall `loop()` for ever, so the core counts only FAILED and stops
  the stream (dropping its backlog) after three consecutive ones.
- mbed raises the in-flight flag *before* arming the transfer and marks it
  `volatile`: a completion interrupt landing between arming and the assignment
  would otherwise leave the flag high with nothing in flight, silently killing
  the stream. `USBCDC` avoids this with `lock()`, which a `PluggableUSBModule`
  cannot take.
- `stream_max_channels` is 8, ring depth 32 records (~1.4 kB).
- The driver rejects duplicate pins in `StreamConfig`: the device dedupes them
  through `STREAM_SELECT` no-ops, which would desync the host's channel map.

Left open:

- Rate ceiling still unmeasured (`--period-us 0` free-run); the README says
  "bounded by the sketch's loop rate" without a number.
- The digital bitmap is framed and parsed but not surfaced through `Sample` or
  the CLI, so `USBIO_STREAM_FLAG_DIGITAL` is not usable from the host yet.
- `Stream::stats().seq_gaps` accumulates unsigned, so a backwards `seq` (a
  device restarting mid-stream) would add ~2^32 instead of 0.
- SAMD streaming compiles and is wired but has never run on hardware; Windows
  is unverified until CI builds the new worker-thread code under MSVC.
- A `STREAM_SELECT` remove is rejected with `BAD_MODE` when the pin's mode
  changed after selection, because the contract orders the mode check before
  the add/remove split. Harmless, but a reconfigured pin can then only be
  dropped by `RESET` or by restarting the selection.
- No `[.hardware]` streaming test: the run above was manual.

## Phase 3 — device time and pin events

Contract frozen 2026-09-03 (`a77aa6a`), both halves implemented by sub-agents
against it, same working split as phase 2 (firmware `src/**`, driver
`extras/driver/**`).

Two additions, deliberately both on plain control transfers so that every
board is covered — the Renesas boards have no endpoint to stream over, and
excluding them was not acceptable for this use case:

- **`GET_TIME` (0x21)** returns `millis` and `micros` together, answered from
  the setup callback rather than a `poll()` shadow so the values timestamp the
  moment the request arrives. Sending both is what makes it useful: `micros`
  wraps every ~71.6 min, but `micros == millis * 1000 (mod 2^32)`, so the host
  rebuilds a 64-bit microsecond clock with no device-side bookkeeping, good for
  49.7 days. That is the anchor the raw `t_us` of stream records and the `t_ms`
  of pin events were missing; with a round-trip estimate the host can place
  device timestamps on its own clock to about ±RTT/2.
- **`EVENT_CONFIG` / `EVENT_POP` / `EVENT_COUNTS` (0x40–0x42)** report debounced
  pin edges. Up to `USBIO_MAX_EVENT_PINS` pins are watched, each with an edge
  mode and a 0–255 ms debounce window applied on the device ("first edge wins").

Design decisions worth recording:

- **Edges are detected by scanning, not by `attachInterrupt()`.** `poll()`
  already refreshes the digital shadow every iteration, so comparing it against
  the previous one costs almost nothing and avoids the whole hardware-interrupt
  problem space: no IRQ-capable-pin restriction, no shared EXTI line conflicts
  (STM32H7 and SAMD21 both multiplex pins onto shared interrupt lines), no
  `InterruptIn` allocation from the mbed core, and no multi-producer ring — one
  detection context instead of several ISRs racing each other. The cost is that
  pulses shorter than one `loop()` iteration are missed, which is irrelevant for
  buttons (the intended use) and disqualifying for encoders or tachometers.
  The wire contract does not encode this choice, so a board could later detect
  the same edges with a real interrupt without any host-visible change.
- **Counters alongside the queue.** The event ring is bounded and can drop under
  a flood; the per-pin counters cannot. So "how many presses happened" is always
  answerable exactly, and only "when precisely did each one happen" degrades.
- **Reply sizes are capped by SAMD, not by us**: that core's EP0 buffer limits a
  reply to 64 bytes, so `EVENT_POP` returns at most 7 events (60 B) and
  `EVENT_COUNTS` at most 8 pins (36 B).
- **`Device` gains an internal mutex** around control transfers, so an
  `EventWatcher`'s worker thread can poll for button events while the main
  thread drives pins. The bulk `Stream` keeps its stricter `DeviceBusy` gate:
  watching buttons must not lock out normal I/O, whereas streaming legitimately
  owns the bulk path.

### Status (2026-09-03)

Both halves implemented, reviewed and building: 98 driver tests pass
(warnings-as-errors, clean under ThreadSanitizer), all nine FQBNs compile
warning-free, +1.2-1.4 kB flash and +392 B RAM on the firmware side.

Hardware (Portenta H7): pin 5 in `INPUT_PULLDOWN`, watched with
`arduino-io watch`, reports rising and falling edges as a wire is connected to
and disconnected from 3V3, and `--debounce 100` cleanly suppresses the bounce
of a hand-made contact. That exercises the whole path on real hardware -
arming, the `poll()` scan, both edge directions, device-side debounce,
`EVENT_POP` framing, and the host decode.

Two bugs found in review, both fixed before the hardware run:

- The event ring was cleared from `apply_reset()` (poll() context) by writing
  `_event_head` from a snapshot of `_event_tail` - but the ISR owns
  `_event_tail`, so an `EVENT_POP` landing between the read and the write left
  tail one *ahead* of head, making `(uint8_t)(head - tail)` read as 255: a
  permanently "full" ring that drops every later edge and never recovers. The
  clear now happens in `request_reset()`, where the ISR moves `_event_tail` up
  to a snapshot of `_event_head` - which can never overtake it.
- The two sub-agents resolved the same ambiguity in the contract text in
  *opposite* directions: the firmware validated the pin's mode before
  unwatching (STALL `BAD_MODE`), while the driver's `FakeTransport` accepted
  `EDGE_OFF` unconditionally. Both suites passed, yet a `configure_event(pin,
  Off)` after a mode change would have thrown against real firmware and
  succeeded in tests. Settled toward the permissive reading - unwatching now
  always succeeds for a valid pin - and the contract now says so explicitly
  instead of leaving it to inference. This also retires the equivalent wart
  noted for `STREAM_SELECT` below, which is still unfixed and now inconsistent
  with how events behave.

Still unverified on hardware: `GET_TIME` (`arduino-io time`), the
queue-overflow drop accounting, the 8-pin watch capacity, and `EventWatcher`'s
callback mode. Note the CLI's `--debounce` defaults to 0; the scan itself only
filters bounce shorter than one `loop()` iteration, so a real contact needs an
explicit window (100 ms verified above).

## Phase 4 — RPC into the sketch (design only, not implemented)

Goal: let the host call a function the *sketch* registered, so a board can be
extended with application-specific behaviour without touching the protocol.
Request codes `0x50..0x5F` are already reserved for this in
`usbio_protocol.h`; nothing else is implemented.

This fits the existing architecture almost exactly: an RPC is the OUT-command
pattern (validate in the ISR, execute in `poll()`) plus a payload and a result.

```cpp
// sketch
uint8_t set_speed(const uint8_t *in, uint8_t in_len, uint8_t *out, uint8_t *out_len);
void setup() { UsbIo.begin(); UsbIo.on(1, "set_speed", set_speed); }
```

```cpp
// host
std::vector<std::byte> result = dev.call(1, args, 200ms);
```

Handlers live in a fixed-size table (no heap), take and return raw bytes and
return a status code. The optional name costs a little flash and buys
`RPC_LIST` discovery, letting the driver resolve names to ids and validate a
call before making it.

### The hard part: getting arguments to the device

The constraint that shaped the whole protocol is that **OUT requests have no
data stage on any of the three stacks**, so a request carries 4 bytes at most,
in `wValue`/`wIndex`. Two ways out:

1. **Chunked staging (recommended).** `RPC_ARG` with `wIndex` = offset and
   `wValue` = two payload bytes, repeated, then `RPC_CALL` with the handler id.
   ~1 ms per 2 bytes, so a 16-byte argument costs ~8 ms. Portable to every
   board with no new per-stack work, and adequate for the realistic cases
   (a setpoint, a mode, a short config blob).
2. **A real OUT data stage.** TinyUSB, SAMD's `USBDevice.recv()` and mbed's
   read stage can all do it, but that is three separate implementations plus
   hardware testing, to optimise a path that is rarely hot. Worth it only if
   payloads beyond ~16 bytes become normal.

Staging first; the data stage can be added later behind the same request codes
if a payload need appears.

### Constraints to design around

- **Results are capped at ~56 bytes** by the SAMD core's 64-byte EP0 buffer,
  not by `USBIO_MAX_REPLY_LEN`.
- **The handler runs inside `poll()`**, so it must be short: it delays the
  shadow refresh, the event scan and the stream sampler for as long as it runs.
  This is the same rule as `loop()` itself, but it now applies to user code we
  are inviting in, so it needs stating loudly in the docs.
- **A handler must not call back into `UsbIo`** — re-entrancy would corrupt the
  command queue.
- **One call outstanding at a time** (a single result slot), with a call token
  so a stale result cannot be mistaken for a fresh one. The host-side mutex
  added in phase 3 serialises callers naturally.
- Error surface to define: unknown handler id, arguments too long, result too
  long, handler returned an error, result not ready yet, and a timeout that
  distinguishes "still running" from "never started".

### Sketch of the request block

| bRequest | dir | wIndex | wValue | data |
|---|---|---|---|---|
| `0x50 RPC_LIST` | IN | first id | – | registered ids (+ names when given) |
| `0x51 RPC_ARG` | OUT | byte offset | two argument bytes | – |
| `0x52 RPC_CALL` | OUT | handler id | argument length | – |
| `0x53 RPC_RESULT` | IN | – | – | status, token, result bytes |
