# ArduinoDriver

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

| directory | content |
|---|---|
| `arduino/UsbIo/` | Arduino library + `UsbIoDevice` example sketch (firmware side) |
| `driver/` | `arduino_driver` C++20 library, `arduino-io` CLI, Catch2 tests |
| `arduino/UsbIo/src/usbio_protocol.h` | the wire protocol, shared verbatim by both sides |
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

The library lives in `arduino/UsbIo`. Either symlink/copy it into your
sketchbook `libraries/` folder and open *File ▸ Examples ▸ UsbIo ▸ UsbIoDevice*
in the IDE, or use `arduino-cli` from this directory:

```bash
arduino-cli compile --fqbn arduino:mbed_portenta:envie_m7 --library arduino/UsbIo arduino/UsbIo/examples/UsbIoDevice
```

The whole sketch is:

```cpp
#include <UsbIo.h>
void setup() { UsbIo.begin(); }
void loop()  { UsbIo.poll(); }
```

The root `CMakeLists.txt` wraps the same commands as targets
(`firmware-portenta`, `firmware-minima`, `firmware-mkrzero`, … and
`upload-<board>` with `-DUSBIO_UPLOAD_PORT=/dev/cu.usbmodemXXXX`) when
`arduino-cli` is on `PATH`.

### 2. Host driver

```bash
cmake -Bbuild -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure   # unit tests, no hardware needed
build/driver/arduino-io list
```

Dependencies (libusb via the official `libusb-cmake` wrapper, `fmt`, `cxxopts`,
Catch2) are fetched with `FetchContent` and pinned. `-DARDUINODRIVER_SYSTEM_LIBUSB=ON`
uses the system libusb-1.0 through pkg-config instead. Windows builds with
MSVC (Visual Studio 2022+) using the same CMake project.

### 3. Talk to the board

```bash
arduino-io info                     # board, pins, resolutions, flags
arduino-io caps                     # per-pin capability table
arduino-io mode 13 output && arduino-io write 13 1
arduino-io mode 14 analog  && arduino-io aread 14 --volts
arduino-io mode 9 pwm      && arduino-io pwm 9 50%
arduino-io monitor --hz 10 14 15    # stream reads until Ctrl-C
```

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

## Protocol

Full specification: [`usbio_protocol.h`](arduino/UsbIo/src/usbio_protocol.h).
Requests are vendor control transfers with device recipient; OUT requests
carry the pin in `wIndex` and the argument in `wValue` and have no data stage.

| bRequest | dir | wIndex | wValue | data |
|---|---|---|---|---|
| `0x00 GET_INFO` | IN | – | – | 24-byte info: magic `UIO1`, protocol version, board id, pin counts, ADC/PWM/DAC bits, Vref, logic level, flags |
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

- **macOS** — no driver needed. libusb opens the composite device and issues
  EP0 vendor requests even though the CDC interfaces belong to `AppleUSBACM`
  (dfu-util relies on the same behaviour to program the UNO R4).
- **Linux** — copy `driver/etc/99-arduino-usbio.rules` to
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
3. `build/driver/arduino-io list` shows the board; `info` reports
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

## Limitations and follow-ups

- Control transfers only: one operation per ~1 ms USB frame. Fine for I/O
  control; streaming (continuous sampling, waveform output) would need bulk
  endpoints and a patched core on Renesas (`CFG_TUD_VENDOR 0`, fixed
  descriptors).
- `Device` is not thread-safe; use one instance per thread.
- SAMD replies are limited to 64 bytes by the core's EP0 buffer (today's
  largest SAMD reply is 33 bytes).
- Planned: MS OS 2.0 descriptors for automatic WinUSB binding; Teensy 4.x and
  STM32duino transports (patched core); RP2040 (arduino-pico) and ESP32
  verification once those cores are available.
