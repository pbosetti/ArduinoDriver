"""Exceptions mirroring the adrv_status_t hierarchy (see arduino_driver_c.h),
which in turn mirrors ArduinoDriver's C++ exception hierarchy (Errors.h).
"""

from __future__ import annotations

import builtins

from . import _ffi


class ArduinoDriverError(RuntimeError):
    """Base class of every exception this package raises for a failing
    arduino_driver_c call."""


class UsbError(ArduinoDriverError):
    """A USB transfer failed at the libusb level."""


class StallError(UsbError):
    """The device rejected the request (STALL)."""


class TimeoutError(UsbError, builtins.TimeoutError):
    """The control transfer did not complete before its timeout."""


class ProtocolError(ArduinoDriverError):
    """The device answered something the protocol does not allow."""


class DeviceBusyError(ArduinoDriverError):
    """A Stream is running, or BUSY retries ran out."""


class InvalidPinError(ArduinoDriverError):
    """Pin index outside the device's valid range."""


class InvalidModeError(ArduinoDriverError):
    """The pin is not in the mode this call needs."""


class InvalidValueError(ArduinoDriverError):
    """A value was outside the accepted range."""


class NotSupportedError(ArduinoDriverError):
    """The pin, or the board, lacks the capability this call needs."""


class QueueFullError(ArduinoDriverError):
    """The firmware command queue is full; call Device.sync() and retry."""


class NotReadyError(ArduinoDriverError):
    """The device enumerated but the sketch has not called UsbIo.begin() yet."""


class DeviceNotFoundError(ArduinoDriverError):
    """No USB device matched the enumeration filter / serial number."""


class InvalidArgumentError(ArduinoDriverError, ValueError):
    """A bad argument was passed to this Python API (e.g. a null handle)."""


_EXCEPTION_BY_STATUS = {
    _ffi.ADRV_ERR_USB: UsbError,
    _ffi.ADRV_ERR_STALL: StallError,
    _ffi.ADRV_ERR_TIMEOUT: TimeoutError,
    _ffi.ADRV_ERR_PROTOCOL: ProtocolError,
    _ffi.ADRV_ERR_DEVICE_BUSY: DeviceBusyError,
    _ffi.ADRV_ERR_INVALID_PIN: InvalidPinError,
    _ffi.ADRV_ERR_INVALID_MODE: InvalidModeError,
    _ffi.ADRV_ERR_INVALID_VALUE: InvalidValueError,
    _ffi.ADRV_ERR_NOT_SUPPORTED: NotSupportedError,
    _ffi.ADRV_ERR_QUEUE_FULL: QueueFullError,
    _ffi.ADRV_ERR_NOT_READY: NotReadyError,
    _ffi.ADRV_ERR_DEVICE_NOT_FOUND: DeviceNotFoundError,
    _ffi.ADRV_ERR_INVALID_ARGUMENT: InvalidArgumentError,
    _ffi.ADRV_ERR_OTHER: ArduinoDriverError,
}


def check(status: int) -> None:
    """Raises the exception matching `status` (an adrv_status_t); a no-op
    for ADRV_OK."""
    if status == _ffi.ADRV_OK:
        return
    message = _ffi.adrv_last_error_message().decode("utf-8", "replace")
    exception_type = _EXCEPTION_BY_STATUS.get(status, ArduinoDriverError)
    raise exception_type(message)
