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

```
ArduinoDriver/
  CMakeLists.txt                 # thin root: add_subdirectory(driver) + optional firmware-* targets (arduino-cli)
  README.md                      # protocol, per-board notes, bring-up checklist, udev/Windows notes
  arduino/UsbIo/                 # Arduino LIBRARY (so the protocol header is shared + one example sketch)
    library.properties
    src/usbio_protocol.h         # C-compatible, freestanding: request codes, structs, status codes, board ids — SINGLE SOURCE OF TRUTH
    src/UsbIo.h / UsbIo.cpp      # board-agnostic: command queue, shadow state, capability table, dispatcher
    src/UsbIoBoard.h             # per-arch capability discovery (pin counts, digitalPinHasPWM, DAC pins, adc/pwm bits, vref)
    src/transport/UsbIoTransport.h        # tiny internal interface: handle_setup(request) → {action, data, len}
    src/transport/tinyusb_renesas.cpp     # #if ARDUINO_ARCH_RENESAS_UNO   — tud_vendor_control_xfer_cb
    src/transport/tinyusb_rp2040.cpp      # #if ARDUINO_ARCH_RP2040        — same weak callback (UNVERIFIED)
    src/transport/esp32_vendor.cpp        # #if ARDUINO_ARCH_ESP32 && ARDUINO_USB_MODE — USBVendor::onRequest (UNVERIFIED)
    src/transport/pluggable_samd.cpp      # #if ARDUINO_ARCH_SAMD          — PluggableUSBModule w/ vendor interface
    src/transport/pluggable_mbed.cpp      # #if ARDUINO_ARCH_MBED          — internal::PluggableUSBModule w/ vendor interface
    examples/UsbIoDevice/UsbIoDevice.ino  # setup(){UsbIo.begin();} loop(){UsbIo.poll();}
  driver/
    CMakeLists.txt               # project ArduinoDriver, C++20, FetchContent: libusb-cmake, fmt, cxxopts, Catch2 v3
    include/arduino_driver/{Protocol.h,Errors.h,Transport.h,LibusbTransport.h,Enumerator.h,Device.h}
    src/{LibusbTransport.cpp,Enumerator.cpp,Device.cpp}
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
