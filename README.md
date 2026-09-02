# ArduinoDriver

Turn an Arduino board into a USB vendor-class I/O peripheral (DIO, analog in,
PWM, DAC) and drive it from a C++20 host library built on libusb — no serial
port, no protocol-over-UART.

- `arduino/UsbIo/` — Arduino library + example sketch (firmware side)
- `driver/` — host library, `arduino-io` CLI, Catch2 tests
- `PLAN.md` — design, protocol, per-board notes, verification

Status: under construction. Full documentation lands here at the end of the
implementation; the wire protocol is specified in
`arduino/UsbIo/src/usbio_protocol.h`.
