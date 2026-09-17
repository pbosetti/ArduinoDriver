"""errors.check(): every adrv_status_t maps to the right exception type, and
ADRV_OK is a no-op. Pure logic -- no native calls, no hardware needed."""

import pytest

from arduino_driver import _ffi
from arduino_driver.errors import (
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
    check,
)

STATUS_TO_EXCEPTION = [
    (_ffi.ADRV_ERR_USB, UsbError),
    (_ffi.ADRV_ERR_STALL, StallError),
    (_ffi.ADRV_ERR_TIMEOUT, TimeoutError),
    (_ffi.ADRV_ERR_PROTOCOL, ProtocolError),
    (_ffi.ADRV_ERR_DEVICE_BUSY, DeviceBusyError),
    (_ffi.ADRV_ERR_INVALID_PIN, InvalidPinError),
    (_ffi.ADRV_ERR_INVALID_MODE, InvalidModeError),
    (_ffi.ADRV_ERR_INVALID_VALUE, InvalidValueError),
    (_ffi.ADRV_ERR_NOT_SUPPORTED, NotSupportedError),
    (_ffi.ADRV_ERR_QUEUE_FULL, QueueFullError),
    (_ffi.ADRV_ERR_NOT_READY, NotReadyError),
    (_ffi.ADRV_ERR_DEVICE_NOT_FOUND, DeviceNotFoundError),
    (_ffi.ADRV_ERR_INVALID_ARGUMENT, InvalidArgumentError),
    (_ffi.ADRV_ERR_OTHER, ArduinoDriverError),
]


def test_ok_is_a_no_op():
    check(_ffi.ADRV_OK)  # must not raise


@pytest.mark.parametrize("status,expected", STATUS_TO_EXCEPTION)
def test_status_maps_to_expected_exception(status, expected):
    with pytest.raises(expected) as exc_info:
        check(status)
    assert isinstance(exc_info.value, ArduinoDriverError)


def test_stall_is_a_usb_error():
    with pytest.raises(UsbError):
        check(_ffi.ADRV_ERR_STALL)


def test_timeout_is_also_the_builtin_timeout_error():
    with pytest.raises(TimeoutError):
        check(_ffi.ADRV_ERR_TIMEOUT)


def test_invalid_argument_is_also_a_value_error():
    with pytest.raises(ValueError):
        check(_ffi.ADRV_ERR_INVALID_ARGUMENT)


def test_unknown_status_falls_back_to_the_base_error():
    with pytest.raises(ArduinoDriverError):
        check(999)


def test_every_status_code_except_ok_is_covered():
    covered = {status for status, _ in STATUS_TO_EXCEPTION}
    all_status_codes = {
        value
        for name, value in vars(_ffi).items()
        if name.startswith("ADRV_ERR_")
    }
    assert covered == all_status_codes
