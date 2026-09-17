"""Continuous sampling (Device.start_stream()); polling only, mirroring
arduino_driver_c.h's adrv_stream_* entry points."""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from typing import TYPE_CHECKING, List

from . import _ffi
from .errors import check

if TYPE_CHECKING:
    from .device import Device


@dataclass(frozen=True)
class Sample:
    """One decoded channel reading (mirrors adrv_sample_t)."""

    pin: int
    raw: int
    volts: float
    t_us: int


@dataclass(frozen=True)
class StreamStats:
    """Counters accumulated by a Stream since it started (mirrors
    adrv_stream_stats_t)."""

    device_overruns: int
    records_received: int
    seq_gaps: int
    host_drops: int
    resyncs: int
    stale_records: int


class Stream:
    """Handle around one continuous-sampling session. Call stop()/close(), or
    use it as a context manager; read() polls, matching the C API."""

    def __init__(self, handle: "_ffi.StreamHandle") -> None:
        self._handle = handle

    @classmethod
    def _start(
        cls,
        device: "Device",
        pins: List[int],
        period_us: int,
        *,
        digital: bool,
        queue_capacity: int,
    ) -> "Stream":
        pins_array = (ctypes.c_uint8 * len(pins))(*pins)
        flags = _ffi.ADRV_STREAM_FLAG_DIGITAL if digital else 0
        config = _ffi.StreamConfig(
            pins=pins_array,
            n_pins=len(pins),
            period_us=period_us,
            flags=flags,
            queue_capacity=queue_capacity,
        )
        handle = _ffi.StreamHandle()
        check(
            _ffi.adrv_start_stream(
                device._handle, ctypes.byref(config), ctypes.byref(handle)
            )
        )
        return cls(handle)

    def read(self, max_samples: int, timeout_ms: int) -> List[Sample]:
        """Blocks until at least one sample is available or timeout_ms
        elapses; returns up to max_samples samples (empty on timeout)."""
        buffer = (_ffi.Sample * max_samples)()
        n_read = ctypes.c_size_t()
        check(
            _ffi.adrv_stream_read(
                self._handle, buffer, max_samples, timeout_ms, ctypes.byref(n_read)
            )
        )
        return [
            Sample(pin=s.pin, raw=s.raw, volts=s.volts, t_us=s.t_us)
            for s in buffer[: n_read.value]
        ]

    @property
    def stats(self) -> StreamStats:
        raw = _ffi.StreamStats()
        check(_ffi.adrv_stream_stats(self._handle, ctypes.byref(raw)))
        return StreamStats(
            device_overruns=raw.device_overruns,
            records_received=raw.records_received,
            seq_gaps=raw.seq_gaps,
            host_drops=raw.host_drops,
            resyncs=raw.resyncs,
            stale_records=raw.stale_records,
        )

    @property
    def running(self) -> bool:
        return bool(_ffi.adrv_stream_running(self._handle))

    @property
    def error(self) -> str:
        """Why the worker stopped on its own; empty while running normally."""
        return _ffi.adrv_stream_error(self._handle).decode("utf-8", "replace")

    def stop(self) -> None:
        """Stops the device stream and joins the worker thread. Idempotent."""
        if self._handle:
            _ffi.adrv_stream_stop(self._handle)

    def close(self) -> None:
        """Stops (if still running) and releases the stream. Idempotent."""
        if self._handle:
            _ffi.adrv_stream_destroy(self._handle)
            self._handle = _ffi.StreamHandle()

    def __enter__(self) -> "Stream":
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    def __del__(self) -> None:
        self.close()
