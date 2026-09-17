"""Exercises the real native library (ctypes -> arduino_driver_c ->
ArduinoDriver -> libusb). Tests that need a board skip themselves (via the
`device` fixture) when none is attached, so this file runs meaningfully both
in CI (skips) and on a dev machine with a board plugged in (real hardware
round-trip)."""

import pytest

from arduino_driver import Device, DeviceNotFoundError, PinMode


@pytest.fixture
def device():
    try:
        dev = Device.open_first()
    except DeviceNotFoundError:
        pytest.skip("no UsbIo board attached")
    yield dev
    dev.close()


def test_open_by_serial_of_a_nonexistent_board_raises_device_not_found():
    # Independent of whether some other board is attached: this serial can
    # never match.
    with pytest.raises(DeviceNotFoundError):
        Device.open_by_serial("nonexistent-serial-should-never-match")


def test_board_name_of_unknown_id_does_not_crash():
    from arduino_driver import _ffi
    from arduino_driver.device import Info

    raw = _ffi.Info()
    raw.board_id = 0  # USBIO_BOARD_UNKNOWN
    assert Info(raw).board_name == "unknown"


def test_info_reports_a_sane_board(device):
    info = device.info
    assert info.n_pins > 0
    assert info.board_name != ""
    assert device.pin_count == info.n_pins


def test_digital_write_read_round_trip(device):
    device.pin_mode(0, PinMode.OUTPUT)
    device.digital_write(0, True)
    assert device.digital_read(0) is True
    device.digital_write(0, False)
    assert device.digital_read(0) is False


def test_reset_returns_pins_to_input(device):
    device.pin_mode(0, PinMode.OUTPUT)
    device.reset()
    # After reset(), the pin is back to INPUT: writing to it without setting
    # OUTPUT again must fail.
    from arduino_driver import InvalidModeError

    with pytest.raises(InvalidModeError):
        device.digital_write(0, True)
