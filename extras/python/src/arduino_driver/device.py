"""Pythonic wrapper around one UsbIo device (see arduino_driver_c.h's
adrv_device_t entry points)."""

from __future__ import annotations

import ctypes
import enum
from typing import List

from . import _ffi
from .errors import check


class PinMode(enum.IntEnum):
    """Mirrors adrv_pin_mode_t."""

    INPUT = _ffi.ADRV_PIN_INPUT
    OUTPUT = _ffi.ADRV_PIN_OUTPUT
    INPUT_PULLUP = _ffi.ADRV_PIN_INPUT_PULLUP
    INPUT_PULLDOWN = _ffi.ADRV_PIN_INPUT_PULLDOWN
    ANALOG_IN = _ffi.ADRV_PIN_ANALOG_IN
    PWM = _ffi.ADRV_PIN_PWM
    DAC = _ffi.ADRV_PIN_DAC


class Info:
    """Snapshot of adrv_info_t (Device.info)."""

    __slots__ = ("_raw",)

    def __init__(self, raw: _ffi.Info) -> None:
        self._raw = raw

    @property
    def board_id(self) -> int:
        return self._raw.board_id

    @property
    def board_name(self) -> str:
        return _ffi.adrv_board_name(self._raw.board_id).decode()

    @property
    def n_pins(self) -> int:
        return self._raw.n_pins

    @property
    def n_analog_pins(self) -> int:
        return self._raw.n_ain

    @property
    def adc_bits(self) -> int:
        return self._raw.adc_bits

    @property
    def pwm_bits(self) -> int:
        return self._raw.pwm_bits

    @property
    def dac_bits(self) -> int:
        return self._raw.dac_bits

    @property
    def vref_volts(self) -> float:
        return self._raw.vref_mv / 1000.0

    @property
    def io_volts(self) -> float:
        return self._raw.io_mv / 1000.0

    @property
    def streaming(self) -> bool:
        return bool(self._raw.flags & _ffi.ADRV_FLAG_STREAMING)

    @property
    def events(self) -> bool:
        return bool(self._raw.flags & _ffi.ADRV_FLAG_EVENTS)

    def __repr__(self) -> str:
        return (
            f"Info(board_name={self.board_name!r}, n_pins={self.n_pins}, "
            f"n_analog_pins={self.n_analog_pins})"
        )


class Device:
    """One open UsbIo board. Build one with `Device.open_first()` or
    `Device.open_by_serial(serial)`, and either call `close()` explicitly or
    use it as a context manager:

        with Device.open_first() as dev:
            dev.pin_mode(13, PinMode.OUTPUT)
            dev.digital_write(13, True)
    """

    def __init__(
        self, context: "_ffi.ContextHandle", handle: "_ffi.DeviceHandle"
    ) -> None:
        # Not a public constructor: use open_first() / open_by_serial().
        self._context = context
        self._handle = handle

    @classmethod
    def open_first(cls) -> "Device":
        """Opens the first identified UsbIo device."""
        context = _ffi.adrv_context_create()
        if not context:
            check(_ffi.ADRV_ERR_OTHER)
        handle = _ffi.DeviceHandle()
        status = _ffi.adrv_device_open_first(context, ctypes.byref(handle))
        if status != _ffi.ADRV_OK:
            _ffi.adrv_context_destroy(context)
            check(status)
        return cls(context, handle)

    @classmethod
    def open_by_serial(cls, serial: str) -> "Device":
        """Opens the identified device with this USB serial number."""
        context = _ffi.adrv_context_create()
        if not context:
            check(_ffi.ADRV_ERR_OTHER)
        handle = _ffi.DeviceHandle()
        status = _ffi.adrv_device_open_by_serial(
            context, serial.encode(), ctypes.byref(handle)
        )
        if status != _ffi.ADRV_OK:
            _ffi.adrv_context_destroy(context)
            check(status)
        return cls(context, handle)

    def close(self) -> None:
        """Closes the device. Idempotent; also called by __exit__/__del__."""
        if self._handle:
            _ffi.adrv_device_close(self._handle)
            self._handle = _ffi.DeviceHandle()
        if self._context:
            _ffi.adrv_context_destroy(self._context)
            self._context = _ffi.ContextHandle()

    def __enter__(self) -> "Device":
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    def __del__(self) -> None:
        self.close()

    # ---- static information (no USB traffic) ----------------------------

    @property
    def info(self) -> Info:
        raw = _ffi.Info()
        check(_ffi.adrv_device_get_info(self._handle, ctypes.byref(raw)))
        return Info(raw)

    @property
    def pin_count(self) -> int:
        return _ffi.adrv_pin_count(self._handle)

    # ---- configuration ----------------------------------------------------

    def pin_mode(self, pin: int, mode: PinMode) -> None:
        check(_ffi.adrv_pin_mode(self._handle, pin, int(mode)))

    # ---- digital I/O --------------------------------------------------------

    def digital_write(self, pin: int, high: bool) -> None:
        check(_ffi.adrv_digital_write(self._handle, pin, high))

    def digital_read(self, pin: int) -> bool:
        value = ctypes.c_bool()
        check(_ffi.adrv_digital_read(self._handle, pin, ctypes.byref(value)))
        return value.value

    def read_all_digital(self) -> List[bool]:
        n = self.pin_count
        buffer = (ctypes.c_bool * n)()
        written = ctypes.c_size_t()
        check(
            _ffi.adrv_read_all_digital(self._handle, buffer, n, ctypes.byref(written))
        )
        return list(buffer[: written.value])

    # ---- analog input -----------------------------------------------------

    def analog_read(self, pin: int) -> int:
        value = ctypes.c_uint16()
        check(_ffi.adrv_analog_read(self._handle, pin, ctypes.byref(value)))
        return value.value

    def analog_read_volts(self, pin: int) -> float:
        value = ctypes.c_double()
        check(_ffi.adrv_analog_read_volts(self._handle, pin, ctypes.byref(value)))
        return value.value

    def read_all_analog(self) -> List[int]:
        n = self.info.n_analog_pins
        buffer = (ctypes.c_uint16 * n)()
        written = ctypes.c_size_t()
        check(
            _ffi.adrv_read_all_analog(self._handle, buffer, n, ctypes.byref(written))
        )
        return list(buffer[: written.value])

    def to_volts(self, raw: int) -> float:
        value = ctypes.c_double()
        check(_ffi.adrv_to_volts(self._handle, raw, ctypes.byref(value)))
        return value.value

    # ---- PWM / DAC output -----------------------------------------------------

    def pwm_write(self, pin: int, duty: int) -> None:
        check(_ffi.adrv_pwm_write(self._handle, pin, duty))

    def pwm_write_fraction(self, pin: int, fraction: float) -> None:
        check(_ffi.adrv_pwm_write_fraction(self._handle, pin, fraction))

    def dac_write(self, pin: int, value: int) -> None:
        check(_ffi.adrv_dac_write(self._handle, pin, value))

    def dac_write_volts(self, pin: int, volts: float) -> None:
        check(_ffi.adrv_dac_write_volts(self._handle, pin, volts))

    # ---- control ----------------------------------------------------------------

    def status(self) -> "tuple[int, int]":
        """Returns (last_error, queue_pending); see arduino_driver_c.h's
        adrv_device_status()."""
        last_error = ctypes.c_uint8()
        queue_pending = ctypes.c_uint8()
        check(
            _ffi.adrv_device_status(
                self._handle, ctypes.byref(last_error), ctypes.byref(queue_pending)
            )
        )
        return last_error.value, queue_pending.value

    def sync(self) -> None:
        check(_ffi.adrv_sync(self._handle))

    def reset(self) -> None:
        check(_ffi.adrv_reset(self._handle))

    # ---- streaming ----------------------------------------------------------------

    def start_stream(
        self,
        pins: List[int],
        period_us: int = 0,
        *,
        digital: bool = False,
        queue_capacity: int = 0,
    ) -> "Stream":
        """Starts continuous sampling of `pins`; see Stream for reading it."""
        from .stream import Stream  # local import: avoids a device/stream cycle

        return Stream._start(
            self, pins, period_us, digital=digital, queue_capacity=queue_capacity
        )
