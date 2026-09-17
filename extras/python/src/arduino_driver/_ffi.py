"""Low-level ctypes bindings to arduino_driver_c (see extras/c_api's
arduino_driver_c.h, which this module mirrors 1:1). Internal: use
arduino_driver.Device / arduino_driver.Stream instead of this module.
"""

from __future__ import annotations

import ctypes
import pathlib
import platform


def _library_path() -> pathlib.Path:
    native_dir = pathlib.Path(__file__).parent / "_native"
    system = platform.system()
    pattern = {"Windows": "*.dll", "Darwin": "*.dylib"}.get(system, "*.so")
    for path in sorted(native_dir.glob(pattern)):
        return path
    raise OSError(
        f"arduino_driver_c native library not found in {native_dir}; "
        "was this package built correctly (pip install / wheel)?"
    )


_lib = ctypes.CDLL(str(_library_path()))


# ---- opaque handles (mirror adrv_context_t / adrv_device_t / adrv_stream_t) --


class _Context(ctypes.Structure):
    pass


class _Device(ctypes.Structure):
    pass


class _Stream(ctypes.Structure):
    pass


ContextHandle = ctypes.POINTER(_Context)
DeviceHandle = ctypes.POINTER(_Device)
StreamHandle = ctypes.POINTER(_Stream)


# ---- status codes (mirror adrv_status_t) --------------------------------------

ADRV_OK = 0
ADRV_ERR_USB = 1
ADRV_ERR_STALL = 2
ADRV_ERR_TIMEOUT = 3
ADRV_ERR_PROTOCOL = 4
ADRV_ERR_DEVICE_BUSY = 5
ADRV_ERR_INVALID_PIN = 6
ADRV_ERR_INVALID_MODE = 7
ADRV_ERR_INVALID_VALUE = 8
ADRV_ERR_NOT_SUPPORTED = 9
ADRV_ERR_QUEUE_FULL = 10
ADRV_ERR_NOT_READY = 11
ADRV_ERR_DEVICE_NOT_FOUND = 12
ADRV_ERR_INVALID_ARGUMENT = 13
ADRV_ERR_OTHER = 14

# ---- pin modes (mirror adrv_pin_mode_t) ---------------------------------------

ADRV_PIN_INPUT = 0
ADRV_PIN_OUTPUT = 1
ADRV_PIN_INPUT_PULLUP = 2
ADRV_PIN_INPUT_PULLDOWN = 3
ADRV_PIN_ANALOG_IN = 4
ADRV_PIN_PWM = 5
ADRV_PIN_DAC = 6

# ---- adrv_info_t.flags bits ----------------------------------------------------

ADRV_FLAG_VENDOR_INTERFACE = 1 << 0
ADRV_FLAG_PULLDOWN = 1 << 1
ADRV_FLAG_STREAMING = 1 << 2
ADRV_FLAG_EVENTS = 1 << 3

# ---- adrv_stream_config_t.flags bits -------------------------------------------

ADRV_STREAM_FLAG_DIGITAL = 1 << 0
ADRV_STREAM_FLAG_STOP_ON_OVERRUN = 1 << 1


# ---- plain structs (mirror the C structs of the same name) --------------------


class Info(ctypes.Structure):
    _fields_ = [
        ("protocol_version", ctypes.c_uint16),
        ("board_id", ctypes.c_uint16),
        ("n_pins", ctypes.c_uint8),
        ("n_ain", ctypes.c_uint8),
        ("adc_bits", ctypes.c_uint8),
        ("pwm_bits", ctypes.c_uint8),
        ("dac_bits", ctypes.c_uint8),
        ("queue_depth", ctypes.c_uint8),
        ("vref_mv", ctypes.c_uint16),
        ("io_mv", ctypes.c_uint16),
        ("flags", ctypes.c_uint16),
        ("stream_max_channels", ctypes.c_uint8),
        ("event_max_pins", ctypes.c_uint8),
        ("stream_min_period_us", ctypes.c_uint16),
    ]


class StreamConfig(ctypes.Structure):
    _fields_ = [
        ("pins", ctypes.POINTER(ctypes.c_uint8)),
        ("n_pins", ctypes.c_size_t),
        ("period_us", ctypes.c_uint32),
        ("flags", ctypes.c_uint8),
        ("queue_capacity", ctypes.c_size_t),
    ]


class Sample(ctypes.Structure):
    _fields_ = [
        ("pin", ctypes.c_uint8),
        ("raw", ctypes.c_uint16),
        ("volts", ctypes.c_double),
        ("t_us", ctypes.c_uint32),
    ]


class StreamStats(ctypes.Structure):
    _fields_ = [
        ("device_overruns", ctypes.c_uint32),
        ("records_received", ctypes.c_uint64),
        ("seq_gaps", ctypes.c_uint64),
        ("host_drops", ctypes.c_uint64),
        ("resyncs", ctypes.c_uint64),
        ("stale_records", ctypes.c_uint64),
    ]


def _declare(name: str, restype, argtypes: list):
    func = getattr(_lib, name)
    func.restype = restype
    func.argtypes = argtypes
    return func


adrv_last_error_message = _declare("adrv_last_error_message", ctypes.c_char_p, [])

adrv_context_create = _declare("adrv_context_create", ContextHandle, [])
adrv_context_destroy = _declare("adrv_context_destroy", None, [ContextHandle])

adrv_device_open_first = _declare(
    "adrv_device_open_first",
    ctypes.c_int,
    [ContextHandle, ctypes.POINTER(DeviceHandle)],
)
adrv_device_open_by_serial = _declare(
    "adrv_device_open_by_serial",
    ctypes.c_int,
    [ContextHandle, ctypes.c_char_p, ctypes.POINTER(DeviceHandle)],
)
adrv_device_close = _declare("adrv_device_close", None, [DeviceHandle])

adrv_device_get_info = _declare(
    "adrv_device_get_info", ctypes.c_int, [DeviceHandle, ctypes.POINTER(Info)]
)
adrv_pin_count = _declare("adrv_pin_count", ctypes.c_size_t, [DeviceHandle])
adrv_board_name = _declare("adrv_board_name", ctypes.c_char_p, [ctypes.c_uint16])

adrv_pin_mode = _declare(
    "adrv_pin_mode", ctypes.c_int, [DeviceHandle, ctypes.c_uint8, ctypes.c_int]
)
adrv_digital_write = _declare(
    "adrv_digital_write",
    ctypes.c_int,
    [DeviceHandle, ctypes.c_uint8, ctypes.c_bool],
)
adrv_digital_read = _declare(
    "adrv_digital_read",
    ctypes.c_int,
    [DeviceHandle, ctypes.c_uint8, ctypes.POINTER(ctypes.c_bool)],
)
adrv_read_all_digital = _declare(
    "adrv_read_all_digital",
    ctypes.c_int,
    [
        DeviceHandle,
        ctypes.POINTER(ctypes.c_bool),
        ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_size_t),
    ],
)
adrv_analog_read = _declare(
    "adrv_analog_read",
    ctypes.c_int,
    [DeviceHandle, ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint16)],
)
adrv_analog_read_volts = _declare(
    "adrv_analog_read_volts",
    ctypes.c_int,
    [DeviceHandle, ctypes.c_uint8, ctypes.POINTER(ctypes.c_double)],
)
adrv_read_all_analog = _declare(
    "adrv_read_all_analog",
    ctypes.c_int,
    [
        DeviceHandle,
        ctypes.POINTER(ctypes.c_uint16),
        ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_size_t),
    ],
)
adrv_to_volts = _declare(
    "adrv_to_volts",
    ctypes.c_int,
    [DeviceHandle, ctypes.c_uint16, ctypes.POINTER(ctypes.c_double)],
)
adrv_pwm_write = _declare(
    "adrv_pwm_write", ctypes.c_int, [DeviceHandle, ctypes.c_uint8, ctypes.c_uint16]
)
adrv_pwm_write_fraction = _declare(
    "adrv_pwm_write_fraction",
    ctypes.c_int,
    [DeviceHandle, ctypes.c_uint8, ctypes.c_double],
)
adrv_dac_write = _declare(
    "adrv_dac_write", ctypes.c_int, [DeviceHandle, ctypes.c_uint8, ctypes.c_uint16]
)
adrv_dac_write_volts = _declare(
    "adrv_dac_write_volts",
    ctypes.c_int,
    [DeviceHandle, ctypes.c_uint8, ctypes.c_double],
)
adrv_device_status = _declare(
    "adrv_device_status",
    ctypes.c_int,
    [DeviceHandle, ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint8)],
)
adrv_sync = _declare("adrv_sync", ctypes.c_int, [DeviceHandle])
adrv_reset = _declare("adrv_reset", ctypes.c_int, [DeviceHandle])

adrv_start_stream = _declare(
    "adrv_start_stream",
    ctypes.c_int,
    [DeviceHandle, ctypes.POINTER(StreamConfig), ctypes.POINTER(StreamHandle)],
)
adrv_stream_read = _declare(
    "adrv_stream_read",
    ctypes.c_int,
    [
        StreamHandle,
        ctypes.POINTER(Sample),
        ctypes.c_size_t,
        ctypes.c_uint32,
        ctypes.POINTER(ctypes.c_size_t),
    ],
)
adrv_stream_stats = _declare(
    "adrv_stream_stats", ctypes.c_int, [StreamHandle, ctypes.POINTER(StreamStats)]
)
adrv_stream_running = _declare("adrv_stream_running", ctypes.c_bool, [StreamHandle])
adrv_stream_error = _declare("adrv_stream_error", ctypes.c_char_p, [StreamHandle])
adrv_stream_stop = _declare("adrv_stream_stop", None, [StreamHandle])
adrv_stream_destroy = _declare("adrv_stream_destroy", None, [StreamHandle])
