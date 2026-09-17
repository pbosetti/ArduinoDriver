"""Stream (continuous sampling): needs a board with the streaming
capability and an analog-capable pin, so it skips itself when neither is
available."""

import pytest

from arduino_driver import Device, DeviceNotFoundError, NotSupportedError, PinMode


@pytest.fixture
def streaming_device():
    try:
        dev = Device.open_first()
    except DeviceNotFoundError:
        pytest.skip("no UsbIo board attached")
    if not dev.info.streaming:
        dev.close()
        pytest.skip("attached board does not support streaming")
    yield dev
    dev.close()


@pytest.fixture
def analog_pin(streaming_device):
    for pin in range(streaming_device.info.n_pins):
        try:
            streaming_device.pin_mode(pin, PinMode.ANALOG_IN)
            return pin
        except NotSupportedError:
            continue
    pytest.skip("attached board has no analog-capable pin")


def test_start_stream_read_stop(streaming_device, analog_pin):
    with streaming_device.start_stream([analog_pin], period_us=1000) as stream:
        assert stream.running
        samples = stream.read(max_samples=16, timeout_ms=1000)
        assert samples, "expected at least one sample before the timeout"
        assert all(sample.pin == analog_pin for sample in samples)
        stats = stream.stats
        assert stats.records_received >= len(samples)
        stream.stop()
        assert not stream.running
