"""Python bindings for ArduinoDriver: digital I/O, analog input, PWM, DAC
and continuous sampling over USB, driving a board running the UsbIo
firmware. See https://github.com/pbosetti/ArduinoDriver.
"""

from .device import Device, Info, PinMode
from .errors import (
    ArduinoDriverError,
    DeviceBusyError,
    DeviceNotFoundError,
    InvalidArgumentError,
    InvalidModeError,
    InvalidPinError,
    InvalidValueError,
    NotReadyError,
    NotSupportedError,
    ProtocolError,
    QueueFullError,
    StallError,
    TimeoutError,
    UsbError,
)
from .stream import Sample, Stream, StreamStats

__version__ = "0.4.0"

__all__ = [
    "Device",
    "Info",
    "PinMode",
    "Sample",
    "Stream",
    "StreamStats",
    "ArduinoDriverError",
    "UsbError",
    "StallError",
    "TimeoutError",
    "ProtocolError",
    "DeviceBusyError",
    "InvalidPinError",
    "InvalidModeError",
    "InvalidValueError",
    "NotSupportedError",
    "QueueFullError",
    "NotReadyError",
    "DeviceNotFoundError",
    "InvalidArgumentError",
]
