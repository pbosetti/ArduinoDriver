# ArduinoDriver and UsbIo quickstart

This guide shows how to control an Arduino board's pins from a program on your
computer: read buttons and sensors, drive LEDs and outputs, and record analog
signals at high speed. You need to know how to write and upload an Arduino
sketch. You do not need to know how USB works.

- [1. What it is, and why not just use Serial](#1-what-it-is-and-why-not-just-use-serial)
- [2. Supported boards](#2-supported-boards)
- [3. Set up the board](#3-set-up-the-board)
- [4. The `arduino-io` command line tool](#4-the-arduino-io-command-line-tool)
- [5. Use the library in your own program](#5-use-the-library-in-your-own-program)
- [6. Troubleshooting](#6-troubleshooting)

## 1. What it is, and why not just use Serial

The project has three parts:

| Part | Runs on | What it does |
|---|---|---|
| **UsbIo** | the board | An Arduino library. Your sketch calls `UsbIo.begin()` and `UsbIo.poll()`, and the board becomes a USB I/O device. |
| **`arduino-io`** | the computer | A command line tool to set pin modes, read and write pins, and record data, without writing any code. |
| **ArduinoDriver** | the computer | A C++ library to do the same from your own program. |

```
┌───────────────────────┐          USB cable          ┌────────────────────┐
│ your program          │ ──── commands ────────────▶ │ Arduino board      │
│  or arduino-io        │ ◀─── readings, samples ──── │ running UsbIo      │
└───────────────────────┘                             └────────────────────┘
```

The usual way to do this is to invent a text protocol over `Serial`: the
sketch prints readings, parses commands, and the computer does the opposite.
UsbIo replaces that with a ready-made protocol that is built into USB itself:

| | Serial with your own protocol | UsbIo |
|---|---|---|
| Firmware | You write and debug message parsing on both sides | A three-line sketch |
| Finding the board | Port names change (`COM3`, `/dev/ttyACM0`, `/dev/cu.usbmodem1102`) | Found automatically, or by its USB serial number |
| Errors | A garbled or lost line may go unnoticed | Every command is accepted or rejected with a reason ("pin 3 is not in OUTPUT mode") |
| Board details | You hard-code pin numbers, ADC resolution, voltages | The board reports which pins can do digital, analog, PWM or DAC, and its resolutions and voltages |
| Fast sampling | Text lines, no timing, silent losses | Binary stream: a Portenta H7 samples one analog pin at about 32 kHz; every sample carries the board's own timestamp, and lost samples are counted |
| `Serial` port | Busy with your protocol | Still free for `Serial.print()` debugging |

It also has limits worth knowing up front:

- The board's microcontroller must have **native USB**. The UNO R3, Mega 2560
  and classic Nano do not (a separate chip handles their USB).
- The feature set is fixed: digital I/O, analog input, PWM, DAC, pin change
  events and the board clock. You cannot add your own commands yet.
- A single pin operation takes about 1 ms. For fast input, use streaming
  (sections [4](#record-a-signal-stream) and [5.3](#53-streaming)).
- One program at a time can use a board.
- Linux and Windows need a one-time permission setup
  ([section 3.4](#34-prepare-the-computer)).

## 2. Supported boards

| Board | Core to install | FQBN | Analog in | PWM | DAC | Logic | Streaming |
|---|---|---|---|---|---|---|---|
| Portenta H7 | `arduino:mbed_portenta` | `arduino:mbed_portenta:envie_m7` | 16 bit | 12 bit | 12 bit | 3.3 V | yes |
| GIGA R1 WiFi | `arduino:mbed_giga` | `arduino:mbed_giga:giga` | 16 bit | 12 bit | 12 bit | 3.3 V | yes |
| Nano 33 BLE | `arduino:mbed_nano` | `arduino:mbed_nano:nano33ble` | 12 bit ¹ | 12 bit | – | 3.3 V | yes |
| Nano RP2040 Connect | `arduino:mbed_nano` | `arduino:mbed_nano:nanorp2040connect` | 12 bit ¹ | 12 bit | – | 3.3 V | yes |
| UNO R4 Minima | `arduino:renesas_uno` | `arduino:renesas_uno:minima` | 14 bit | 12 bit | 12 bit | 5 V | no |
| Nano R4 | `arduino:renesas_uno` | `arduino:renesas_uno:nanor4` | 14 bit | 12 bit | 12 bit | 5 V | no |
| Zero | `arduino:samd` | `arduino:samd:arduino_zero_native` | 12 bit | 10 bit | 10 bit | 3.3 V | yes |
| MKR family (e.g. MKR Zero) | `arduino:samd` | `arduino:samd:mkrzero` | 12 bit | 10 bit | 10 bit | 3.3 V | yes |
| Nano 33 IoT | `arduino:samd` | `arduino:samd:nano_33_iot` | 12 bit | 10 bit | 10 bit | 3.3 V | yes |

¹ Reported as 16 bit; the chip delivers 12 significant bits.

- **Tested on hardware:** the Portenta H7. The other boards build without
  warnings but have not been tested on hardware yet.
- **Every board:** digital input and output, pull-up inputs, analog input,
  PWM, pin change events with debouncing, and the board clock.
- **UNO R4 Minima and Nano R4:** no pull-down inputs, no streaming, and no
  Windows support.
- **Not supported:** UNO R3, Mega 2560 and classic Nano (no native USB), and
  UNO R4 WiFi (its USB port belongs to the ESP32 chip).
- **Experimental and untested:** boards using the Raspberry Pi Pico core
  (`rp2040:rp2040`) and ESP32-S2/S3.

**Pin numbers** are the Arduino pin numbers you use in `digitalRead()` and
`analogRead()` in a sketch. The analog pins have board-specific numbers: A0 is
pin 15 on a Portenta H7 and pin 14 on an UNO R4. `arduino-io caps` lists every
pin and what it can do (see [section 4](#explore-the-board)).

## 3. Set up the board

### 3.1 Install the core and the library

In the **Arduino IDE**:

1. *Tools ▸ Board ▸ Boards Manager*: install the core for your board (see the
   table above, e.g. *Arduino Mbed OS Portenta Boards*).
2. *Tools ▸ Manage Libraries*: search for **UsbIo** and install it.

With **`arduino-cli`**:

```bash
arduino-cli core install arduino:mbed_portenta
arduino-cli lib install UsbIo
```

### 3.2 Upload the sketch

Open *File ▸ Examples ▸ UsbIo ▸ UsbIoDevice*, select your board and port, and
upload. The whole sketch is:

```cpp
#include <UsbIo.h>

void setup() {
  UsbIo.begin();
}

void loop() {
  UsbIo.poll();
}
```

With `arduino-cli`, create a sketch, paste the code above into it, then
compile and upload (replace the port and FQBN with yours):

```bash
arduino-cli sketch new UsbIoDevice    # then paste the code into UsbIoDevice/UsbIoDevice.ino
arduino-cli board list                # shows the port and FQBN of the board
arduino-cli compile --upload -p /dev/cu.usbmodem1102 \
  --fqbn arduino:mbed_portenta:envie_m7 UsbIoDevice
```

**If the upload cannot find the board**, double-tap its reset button to enter
the bootloader (the LED pulses on mbed boards), check `arduino-cli board list`
again (the port name often changes), and upload to the new port.

### 3.3 Adding your own code to the sketch

The sketch can do other things too, as long as you follow a few rules:

```cpp
#include <UsbIo.h>

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  UsbIo.begin();
}

void loop() {
  UsbIo.poll(); // runs the commands from the computer: call it on every loop

  // Your own code: keep it short, and never call delay().
  static unsigned long last = 0;
  if (millis() - last >= 500) {
    last = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // heartbeat
  }
}
```

- **Keep `loop()` fast.** Commands from the computer wait until the next
  `UsbIo.poll()`, and streaming takes at most one sample per `poll()`. At
  1 kHz, `loop()` must take well under 1 ms.
- **Share the pins carefully.** UsbIo leaves every pin alone until the
  computer sets its mode, so your `Serial1`, SPI or LED code keeps working. But
  do not drive a pin from both the sketch and the computer. A reset from the
  computer (`arduino-io reset`, or `reset()` in the library) puts *every* pin
  back to INPUT, including yours.
- **`Serial` is still yours**, for `Serial.print()` debugging.

### 3.4 Prepare the computer

- **macOS:** nothing to do.
- **Linux:** allow normal users to access the board, once per machine, then
  replug the board:

  ```bash
  sudo cp extras/driver/etc/99-arduino-usbio.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```

  The file is in the ArduinoDriver repository (see
  [section 4.1](#41-install-it)). Without it, run the tools with `sudo`.
- **Windows:** use [Zadig](https://zadig.akeo.ie/) to install the **WinUSB**
  driver on the interface named **UsbIo**. In Zadig, enable *Options ▸ List
  All Devices*, pick *UsbIo* (not the serial port), and click *Install
  Driver*. The serial port keeps working. UNO R4 boards cannot be used on
  Windows.

**Portenta H7 tip:** it uses fast USB (High Speed, 480 Mbit/s). Some computer
ports cannot keep that link clean, and fast streams then stop with
`LIBUSB_ERROR_IO`; this was seen on a Mac's USB-C port. Plugging the board
into a USB hub (powered or not) solved it.

## 4. The `arduino-io` command line tool

### 4.1 Install it

`arduino-io` is built from source. You need:

- **git**
- **CMake** 3.24 or newer ([cmake.org](https://cmake.org/download/), or your
  package manager)
- **a C++ compiler:**
  - *macOS:* Xcode command line tools (`xcode-select --install`)
  - *Linux:* `g++` or `clang` (e.g. `sudo apt install build-essential`)
  - *Windows:* Visual Studio 2022 with the *Desktop development with C++*
    workload

Then, in a terminal:

```bash
git clone https://github.com/pbosetti/ArduinoDriver.git
cd ArduinoDriver
cmake -B build -DARDUINODRIVER_BUILD_TESTS=OFF
cmake --build build --config Release
cmake --install build --config Release --prefix ~/.local
```

The first build downloads what it needs, so it takes a few minutes.
`arduino-io` ends up in `~/.local/bin`: make sure that folder is in your
`PATH`, or run `sudo cmake --install build` to install into `/usr/local/bin`
instead. On Windows, choose any folder with `--prefix`.

### 4.2 Everyday commands

Pin modes set with `arduino-io` stay on the board until it is reset or
unplugged, so you can set a mode once and then read or write many times.

#### Explore the board

```bash
arduino-io list      # connected UsbIo boards
arduino-io info      # board name, pin count, resolutions, features
arduino-io caps      # every pin and what it can do
```

```
$ arduino-io info
board:            Portenta H7 (id 0x0201)
pins:             26 (analog: 7)
resolution:       adc 16 bits, pwm 12 bits, dac 12 bits
voltages:         vref 3300 mV, io 3300 mV
flags:            0x000F vendor-interface pulldown streaming events
stream period:    1 us or longer (up to 1000000 Hz, bounded by the sketch's loop() rate)
...
$ arduino-io caps
PIN  CAPS  NOTES
0    D-P-
...
15   -A--  analog #0 (analog-only pad)
...
21   DA-C  analog #6, DAC
D digital I/O, A analog input, P PWM, C DAC
```

`flags` tells you what the board supports: `streaming` and `events` appear only
on boards (and firmware versions) that have them.

#### Set pin modes

```bash
arduino-io mode <pin> <mode>
```

| Mode | Like `pinMode()` / use |
|---|---|
| `input` | `INPUT` |
| `pullup` | `INPUT_PULLUP` |
| `pulldown` | `INPUT_PULLDOWN` (not on UNO R4) |
| `output` | `OUTPUT` |
| `analog` | analog input, for `aread` and `stream` |
| `pwm` | PWM output, for `pwm` |
| `dac` | analog output, for `dac` |

#### Read and write

```bash
arduino-io mode 13 output && arduino-io write 13 1   # like digitalWrite(13, HIGH)
arduino-io mode 2 pullup  && arduino-io read 2       # prints 0 or 1
arduino-io mode 15 analog && arduino-io aread 15 --volts
arduino-io mode 5 pwm     && arduino-io pwm 5 25%    # 25% duty cycle
arduino-io mode 21 dac    && arduino-io dac 21 1.5V  # 1.5 V on the DAC pin
arduino-io areadall --volts                          # every analog pin at once
arduino-io monitor --hz 10 2 15                      # print pins 2 and 15, 10 times a second
arduino-io reset                                     # every pin back to INPUT
```

Without `--volts`, analog values are raw ADC codes (0 to 65535 at 16 bit).
`pwm` and `dac` also accept raw codes (`pwm 5 1024`, `dac 21 2048`).

#### Record a signal (stream)

On boards with streaming, the board samples the pins itself at a steady rate
and sends the samples in bulk:

```bash
arduino-io mode 15 analog && arduino-io mode 16 analog
arduino-io stream 15,16 --hz 1000 --seconds 10 --volts --csv data.csv
```

```
t_us,pin,volts
2780434980,15,0.2322
2780434980,16,0.2463
2780435982,15,0.2321
...
stream: 1002 records (2004 samples) in 1.001 s (1000.8 Hz achieved per channel); device overruns 0, seq gaps 0, host drops 0, resyncs 0, stale records 0
```

- **`t_us`** is the board's own clock in microseconds, so the timing is
  exact even if the computer is busy. Samples taken at the same instant share
  a timestamp.
- **Rates.** The board takes one sample per `loop()`, so the fastest rate
  depends on the board, the number of pins and your sketch. With the plain
  UsbIoDevice sketch, a Portenta H7 reaches about 32 kHz on one pin and
  23 kHz on two. Ask for more and you simply get that maximum; the summary
  shows the rate achieved. `--period-us 0` instead of `--hz` always samples
  as fast as the board can. Boards with UsbIo older than 0.4.0 stop at 10 kHz
  (`arduino-io info` shows the limit).
- **The summary line** reports losses: `device overruns` (the board could not
  send fast enough), `seq gaps` (samples lost on the way) and `host drops`
  (the computer did not read fast enough). All zeros means a complete
  recording.
- **Without `--csv`**, samples go to the terminal, and the summary goes to
  the error output (so `arduino-io stream ... > data.csv` works too).

#### Watch for pin changes (events)

```bash
arduino-io mode 2 pullup
arduino-io watch 2 --edge falling --debounce 20   # a button between pin 2 and GND
```

The board detects each change on the pin and filters contact bounce (here
20 ms), and `watch` prints one line per press until you press Ctrl-C. It suits
buttons and switches; very short pulses (shorter than one `loop()`) can be
missed.

#### Other useful commands

```bash
arduino-io time                       # the board's clock, and how well it lines up with the computer's
arduino-io status                     # the reason the last command was rejected
arduino-io --serial <serial> info     # choose a board by the serial number `list` shows
arduino-io --help                     # everything else
```

`arduino-io` exits with 0 on success, 1 when the board reports an error (the
message is printed), and 2 when the command line is wrong, so it is easy to
use in scripts.

## 5. Use the library in your own program

### 5.1 Project setup

Create a folder with the three files below: `CMakeLists.txt`, `io.cpp` and
`stream.cpp`. CMake downloads ArduinoDriver for you; nothing else needs to be
installed apart from the tools listed in [section 4.1](#41-install-it).

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.24)
project(my_board_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(ArduinoDriver
  GIT_REPOSITORY https://github.com/pbosetti/ArduinoDriver.git
  GIT_TAG v0.4.0)
FetchContent_MakeAvailable(ArduinoDriver)

add_executable(io io.cpp)
target_link_libraries(io PRIVATE ArduinoDriver::arduino_driver)

add_executable(stream stream.cpp)
target_link_libraries(stream PRIVATE ArduinoDriver::arduino_driver)
```

Build and run:

```bash
cmake -B build
cmake --build build --config Release
./build/io         # on Windows: build\Release\io.exe
./build/stream
```

### 5.2 Reading and writing pins

This program blinks pin 13, sets a PWM output, and reads a button and an
analog sensor twice a second. Wire a button between pin 2 and GND and a
sensor (or a potentiometer) on A0, or just watch the values change as you
touch the pins.

```cpp
// io.cpp - blink a pin, dim another, read a button and a sensor.
#include <arduino_driver/Device.h>
#include <arduino_driver/Enumerator.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>

using namespace ArduinoDriver;
using namespace std::chrono_literals;

int main() {
  const std::uint8_t led_pin = 13;   // digital output
  const std::uint8_t pwm_pin = 5;    // PWM output (check `arduino-io caps`)
  const std::uint8_t button_pin = 2; // button between this pin and GND

  try {
    auto usb = std::make_shared<Context>();
    Device board = open_first(usb); // the first UsbIo board found
    std::cout << "Connected to " << board_name(board.info().board_id) << "\n";

    // The first analog input, whatever its number on this board (A0).
    const std::uint8_t sensor_pin = board.analog_pins().front();

    board.pin_mode(led_pin, PinMode::Output);
    board.pin_mode(pwm_pin, PinMode::Pwm);
    board.pin_mode(button_pin, PinMode::InputPullup);
    board.pin_mode(sensor_pin, PinMode::AnalogIn);

    board.pwm_write_fraction(pwm_pin, 0.25); // 25% duty cycle

    for (int i = 0; i < 10; ++i) {
      board.digital_write(led_pin, i % 2 == 0);
      const bool pressed = !board.digital_read(button_pin); // LOW = pressed
      const double volts = board.analog_read_volts(sensor_pin);
      std::cout << "button " << (pressed ? "pressed " : "released")
                << "   A0 = " << volts << " V\n";
      std::this_thread::sleep_for(500ms);
    }

    board.reset(); // every pin back to INPUT
  } catch (const Error &e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
```

What to know:

- **Opening the board.** `open_first()` opens the first board it finds. To
  pick a specific one, use `open_by_serial(usb, "<serial>")` with the serial
  number `arduino-io list` shows.
- **Pin modes come first**, exactly like `pinMode()` in a sketch: reading a
  pin that has no mode, or writing a pin in the wrong mode, is an error.
- **Errors are exceptions.** Every problem (no board found, a pin that cannot
  do PWM, a value out of range, a USB failure) throws an `Error` whose
  `what()` explains it. The library checks pins and values before sending
  anything, so most mistakes are caught immediately.

The functions you will use most:

| On the board (sketch) | On the computer (`Device board`) |
|---|---|
| `pinMode(pin, OUTPUT)` | `board.pin_mode(pin, PinMode::Output)` (also `Input`, `InputPullup`, `InputPulldown`, `AnalogIn`, `Pwm`, `Dac`) |
| `digitalWrite(pin, HIGH)` | `board.digital_write(pin, true)` |
| `digitalRead(pin)` | `board.digital_read(pin)` returns `bool` |
| `analogRead(pin)` | `board.analog_read(pin)` (raw code) or `board.analog_read_volts(pin)` |
| `analogWrite(pin, value)` | `board.pwm_write(pin, raw)` or `board.pwm_write_fraction(pin, 0.0 to 1.0)` |
| `analogWrite(DAC, value)` | `board.dac_write(pin, raw)` or `board.dac_write_volts(pin, volts)` |
| – | `board.read_all_digital()`, `board.read_all_analog()`: every pin in one call |
| – | `board.info()`: resolutions (`adc_bits`, `pwm_bits`, `dac_bits`), voltages, `streaming()` |
| – | `board.analog_pins()`: the analog pin numbers, A0 first |
| – | `board.reset()`: every pin back to INPUT |

### 5.3 Streaming

This program records the first two analog inputs at 1 kHz for 3 seconds and
prints a pair of readings twice a second:

```cpp
// stream.cpp - sample two analog inputs at 1 kHz for 3 seconds.
#include <arduino_driver/Device.h>
#include <arduino_driver/Enumerator.h>
#include <arduino_driver/Stream.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

using namespace ArduinoDriver;
using namespace std::chrono_literals;

int main() {
  try {
    auto usb = std::make_shared<Context>();
    Device board = open_first(usb);
    if (!board.info().streaming()) {
      std::cerr << "This board cannot stream\n";
      return 1;
    }

    // Stream the first two analog inputs (A0 and A1).
    const std::uint8_t a0 = board.analog_pins()[0];
    const std::uint8_t a1 = board.analog_pins()[1];
    board.pin_mode(a0, PinMode::AnalogIn);
    board.pin_mode(a1, PinMode::AnalogIn);

    StreamConfig config;
    config.pins = {a0, a1};  // every record holds one sample per pin, in order
    config.period = 1000us;  // one record every 1000 microseconds = 1 kHz
    Stream stream = board.start_stream(config);

    std::vector<Sample> samples(200);
    std::uint64_t records = 0;
    const auto end = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < end && stream.running()) {
      // Waits up to 100 ms for data, returns how many samples it copied.
      const std::size_t n = stream.read(samples, 100ms);
      for (std::size_t i = 0; i + 1 < n; i += 2) {
        const Sample &first = samples[i];      // pin a0
        const Sample &second = samples[i + 1]; // pin a1
        if (records % 500 == 0) {              // print twice a second
          std::cout << "t = " << first.t_us << " us   A0 = " << first.volts
                    << " V   A1 = " << second.volts << " V\n";
        }
        ++records;
      }
    }

    if (!stream.running()) {
      std::cerr << "Stream stopped: " << stream.error() << "\n";
    }
    const StreamStats stats = stream.stats();
    std::cout << stats.records_received << " records received, "
              << stats.seq_gaps << " lost on the way\n";
  } catch (const Error &e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
```

Output on a Portenta H7:

```
t = 2770539776 us   A0 = 0.231128 V   A1 = 0.227604 V
t = 2771039776 us   A0 = 0.245983 V   A1 = 0.233344 V
...
3003 records received, 0 lost on the way
```

What to know:

- **Records and samples.** Every *record* holds one `Sample` per streamed pin,
  all taken at the same instant, in the order of `config.pins`. `read()`
  always copies whole records.
- **Each `Sample`** has `pin`, `raw` (the ADC code), `volts`, and `t_us`, the
  board's clock in microseconds when the record was taken.
- **Rate.** `config.period` sets the time between records, in whole
  microseconds (`50us` is 20 kHz). A period shorter than the board can manage
  gives its maximum rate (see [section 4](#record-a-signal-stream)); `0us`
  samples as fast as the board can. Boards with UsbIo older than 0.4.0 refuse
  periods under 100 µs.
- **Keep reading.** Samples wait in a queue on the computer (1024 records by
  default, `config.queue_capacity`). If your program falls behind, the oldest
  are dropped and counted in `stats.host_drops`.
- **Losses are counted, never hidden.** `stats.seq_gaps` counts records lost
  before reaching the computer, and `stats.device_overruns` counts the ones
  the board could not send in time (they are included in `seq_gaps`).
- **When a stream stops by itself** (board unplugged, a USB error),
  `stream.running()` becomes `false` and `stream.error()` says why. Open the
  board again to start over.
- **One thing at a time.** While a stream runs, every other call on `board`
  throws `DeviceBusy`. Stop the stream first with `stream.stop()`, or let
  `stream` go out of scope.

## 6. Troubleshooting

| Symptom | What to check |
|---|---|
| `no UsbIo device found` | Is the UsbIo sketch running? Does `arduino-io list` show the board under "Not probed"? Then it is a permissions problem: see [section 3.4](#34-prepare-the-computer). |
| `cannot claim UsbIo interface ... LIBUSB_ERROR_ACCESS` | Another program is using the board (e.g. another `arduino-io stream`), or on Linux/Windows the setup in section 3.4 is missing. |
| `rejected by the device: BAD_MODE` | Set the pin mode first (`arduino-io mode ...` or `pin_mode()`). Modes are lost when the board is reset or unplugged. |
| `does not support mode ...` or `has no PWM capability` | The pin cannot do that: check `arduino-io caps`. For example, A0–A3 on a Portenta H7 are analog-only. |
| Streams stop with `LIBUSB_ERROR_IO` (Portenta H7) | Connect the board through a USB hub (see [section 3.4](#34-prepare-the-computer)). |
| Stream reports overruns | `loop()` is too slow for the sampling rate: remove `delay()` and slow code from the sketch, or lower the rate. |
| Upload fails | Double-tap the reset button and upload to the port that appears. |

For the details (protocol, internals, per-board pin notes), see the
[README](README.md).
