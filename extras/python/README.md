# arduino-driver

Python bindings for [ArduinoDriver](https://github.com/pbosetti/ArduinoDriver): drive
an Arduino board running the UsbIo firmware (digital I/O, analog input, PWM, DAC and
continuous sampling) over USB, without a serial port.

```python
from arduino_driver import Device, PinMode

with Device.open_first() as dev:
    dev.pin_mode(13, PinMode.OUTPUT)
    dev.digital_write(13, True)
    print(dev.info)
```

This package wraps `arduino_driver_c`, the C ABI built from
[`extras/c_api`](../c_api), via `ctypes`; no compiler is needed at import time; a
scikit-build-core step compiles the native library when the wheel itself is built.

See the main repository's [`README.md`](../../README.md) for board setup (udev rules on
Linux, WinUSB/Zadig on Windows).
