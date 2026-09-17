// test_stream_api.cpp - Device::start_stream() / Stream control-path
// behaviour against the fake firmware: select/start/stop sequencing,
// validation, the DeviceBusy contract while a Stream runs, and RAII
// teardown. Byte-stream decoding itself is covered by
// test_stream_decoding.cpp.
#include "TestRig.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace ArduinoDriver;
using ArduinoDriver::Testing::FakeBoard;
using ArduinoDriver::Testing::FakeTransport;
using ArduinoDriver::Testing::fast_options;
using ArduinoDriver::Testing::Rig;

using namespace std::chrono_literals;

namespace {

FakeBoard streaming_board() {
  FakeBoard board = FakeBoard::portenta_h7();
  board.flags |= USBIO_FLAG_STREAMING;
  board.stream_max_channels = 4;
  return board;
}

} // namespace

// ---- select/start/stop sequencing --------------------------------------

TEST_CASE("start_stream selects the pins in order and starts the device",
          "[stream][api]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(19, PinMode::AnalogIn); // A4
  rig.device.pin_mode(20, PinMode::AnalogIn); // A5
  rig.fake.clear_log();

  StreamConfig config;
  config.pins = {19, 20};
  config.period = std::chrono::microseconds{1000};
  Stream stream = rig.device.start_stream(config);

  CHECK(rig.fake.stream_running());
  CHECK(rig.fake.stream_selected() == std::vector<std::uint8_t>{19, 20});
  CHECK(rig.fake.stream_period_us() == 1000);
  CHECK(stream.running());
  CHECK(stream.pins() == std::vector<std::uint8_t>{19, 20});

  // GET_STREAM_STATUS (is a stream left running?), STREAM_SELECT once per
  // pin (in order), GET_STREAM_STATUS to confirm the device's channel count
  // matches, GET_TIME to anchor stale-record detection, then STREAM_START.
  // (The worker's own status polls may follow, so only the first six
  // requests are checked.)
  const auto log = rig.fake.log();
  REQUIRE(log.size() >= 6);
  CHECK(log[0].request == USBIO_REQ_STREAM_STATUS);
  CHECK(log[1].request == USBIO_REQ_STREAM_SELECT);
  CHECK(log[1].index == 19);
  CHECK(log[1].value == 1);
  CHECK(log[2].request == USBIO_REQ_STREAM_SELECT);
  CHECK(log[2].index == 20);
  CHECK(log[2].value == 1);
  CHECK(log[3].request == USBIO_REQ_STREAM_STATUS);
  CHECK(log[4].request == USBIO_REQ_GET_TIME);
  CHECK(log[5].request == USBIO_REQ_STREAM_START);
  CHECK(log[5].value == 1000);

  stream.stop();
  CHECK_FALSE(rig.fake.stream_running());
  CHECK_FALSE(stream.running());
  // The selection survives STOP (kept until RESET or a different config).
  CHECK(rig.fake.stream_selected() == std::vector<std::uint8_t>{19, 20});

  // stop() is idempotent.
  CHECK_NOTHROW(stream.stop());
}

TEST_CASE("a second start_stream() replaces the pin selection", "[stream][api]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(19, PinMode::AnalogIn);
  rig.device.pin_mode(20, PinMode::AnalogIn);
  rig.device.pin_mode(0, PinMode::Input);

  StreamConfig first;
  first.pins = {19, 20};
  { Stream s = rig.device.start_stream(first); s.stop(); }

  StreamConfig second;
  second.pins = {0};
  Stream s2 = rig.device.start_stream(second);
  // 19 and 20 are deselected (no longer wanted), 0 is added.
  CHECK(rig.fake.stream_selected() == std::vector<std::uint8_t>{0});
  s2.stop();
}

TEST_CASE("start_stream clears a stale selection left by another session",
          "[stream][api]") {
  // One device, two host sessions: the selection survives STREAM_STOP on the
  // device, and the second Device has no record of what the first selected.
  FakeTransport fake(streaming_board());
  auto borrowed = [&fake] {
    return std::make_unique<ArduinoDriver::Testing::BorrowedTransport>(fake);
  };
  {
    Device first(borrowed(), fast_options());
    first.pin_mode(19, PinMode::AnalogIn);
    first.pin_mode(20, PinMode::AnalogIn);
    first.pin_mode(0, PinMode::Input);
    StreamConfig config;
    config.pins = {0, 19};
    Stream s = first.start_stream(config);
    s.stop();
  }
  REQUIRE(fake.stream_selected() == std::vector<std::uint8_t>{0, 19});

  Device second(borrowed(), fast_options());
  StreamConfig config;
  config.pins = {20};
  Stream s = second.start_stream(config);
  CHECK(fake.stream_selected() == std::vector<std::uint8_t>{20});
  CHECK(fake.stream_running());
  s.stop();
}

TEST_CASE("start_stream stops a stream an ended session left running",
          "[stream][api]") {
  // A process killed mid-stream never sends STREAM_STOP: the device keeps
  // sampling, and would refuse the next session's STREAM_SELECT with BUSY.
  FakeTransport fake(streaming_board());
  auto borrowed = [&fake] {
    return std::make_unique<ArduinoDriver::Testing::BorrowedTransport>(fake);
  };
  Device device(borrowed(), fast_options());
  device.pin_mode(19, PinMode::AnalogIn);
  device.pin_mode(20, PinMode::AnalogIn);
  fake.control_out(USBIO_REQ_STREAM_SELECT, 1, 19, 100ms);
  fake.control_out(USBIO_REQ_STREAM_START, 0, 0, 100ms);
  REQUIRE(fake.stream_running());

  StreamConfig config;
  config.pins = {20};
  Stream s = device.start_stream(config);
  CHECK(fake.count(Request::StreamStop) == 1);
  CHECK(fake.stream_selected() == std::vector<std::uint8_t>{20});
  CHECK(fake.stream_running());
  s.stop();
}

// ---- Validation -----------------------------------------------------------

TEST_CASE("start_stream throws NotSupported when the info flag is clear",
          "[stream][api]") {
  Rig rig(FakeBoard::portenta_h7(), fast_options()); // no USBIO_FLAG_STREAMING
  CHECK_FALSE(rig.device.info().streaming());
  rig.device.pin_mode(19, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {19};
  CHECK_THROWS_AS(rig.device.start_stream(config), NotSupported);
  CHECK_FALSE(rig.fake.stream_running());
}

TEST_CASE("start_stream rejects a pin that is not in ANALOG_IN or an INPUT* "
          "mode with InvalidMode (device BAD_MODE)",
          "[stream][api]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(19, PinMode::AnalogIn); // ok
  // pin 20 is left unconfigured.
  StreamConfig config;
  config.pins = {19, 20};
  CHECK_THROWS_AS(rig.device.start_stream(config), InvalidMode);
  // Best-effort rollback: nothing stays selected, and the device is not
  // left "running" or busy.
  CHECK(rig.fake.stream_selected().empty());
  CHECK_FALSE(rig.fake.stream_running());
  CHECK_NOTHROW(rig.device.pin_mode(20, PinMode::Input));

  SECTION("an OUTPUT pin is rejected the same way") {
    Rig out_rig(streaming_board(), fast_options());
    out_rig.device.pin_mode(0, PinMode::Output);
    StreamConfig out_config;
    out_config.pins = {0};
    CHECK_THROWS_AS(out_rig.device.start_stream(out_config), InvalidMode);
  }
}

TEST_CASE("start_stream validates pins/period locally before any USB traffic",
          "[stream][api]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(19, PinMode::AnalogIn);
  rig.fake.clear_log();

  SECTION("empty pin list") {
    StreamConfig config;
    CHECK_THROWS_AS(rig.device.start_stream(config), InvalidValue);
    CHECK(rig.fake.log().empty()); // rejected before any USB traffic
  }
  SECTION("more pins than stream_max_channels") {
    StreamConfig config;
    config.pins = {19, 19, 19, 19, 19}; // 5 > stream_max_channels (4)
    CHECK_THROWS_AS(rig.device.start_stream(config), InvalidValue);
    CHECK(rig.fake.log().empty());
  }
  SECTION("pin out of range") {
    StreamConfig config;
    config.pins = {19, 99};
    CHECK_THROWS_AS(rig.device.start_stream(config), InvalidPin);
    CHECK(rig.fake.log().empty());
  }
  SECTION("a repeated pin") {
    rig.device.pin_mode(20, PinMode::AnalogIn);
    rig.fake.clear_log();
    StreamConfig config;
    config.pins = {19, 20, 19};
    CHECK_THROWS_AS(rig.device.start_stream(config), InvalidValue);
    CHECK(rig.fake.log().empty());
  }
  SECTION("period below the board's minimum") {
    StreamConfig config;
    config.pins = {19};
    config.period = std::chrono::microseconds{1}; // below the legacy 100 us
    CHECK_THROWS_AS(rig.device.start_stream(config), InvalidValue);
    CHECK(rig.fake.log().empty());
  }
  SECTION("period above 65535 us") {
    StreamConfig config;
    config.pins = {19};
    config.period = std::chrono::microseconds{100000};
    CHECK_THROWS_AS(rig.device.start_stream(config), InvalidValue);
    CHECK(rig.fake.log().empty());
  }
  SECTION("queue_capacity 0") {
    StreamConfig config;
    config.pins = {19};
    config.queue_capacity = 0;
    CHECK_THROWS_AS(rig.device.start_stream(config), InvalidValue);
    CHECK(rig.fake.log().empty());
  }
  SECTION("period 0 (free running) is fine") {
    StreamConfig config;
    config.pins = {19};
    config.period = std::chrono::microseconds{0};
    Stream stream = rig.device.start_stream(config);
    CHECK(rig.fake.stream_period_us() == 0);
    stream.stop();
  }
}

TEST_CASE("start_stream accepts periods down to the board's reported minimum",
          "[stream][api]") {
  SECTION("firmware before 0.4.0 reports 0: 100 us, with a hint to update") {
    Rig rig(streaming_board(), fast_options());
    rig.device.pin_mode(19, PinMode::AnalogIn);
    CHECK(rig.device.info().min_stream_period_us() == StreamLegacyMinPeriodUs);
    rig.fake.clear_log();
    StreamConfig config;
    config.pins = {19};
    config.period = 50us;
    try {
      rig.device.start_stream(config);
      FAIL("a 50 us period was accepted");
    } catch (const InvalidValue &e) {
      CHECK(std::string(e.what()).find("0.4.0") != std::string::npos);
    }
    CHECK(rig.fake.log().empty());
    config.period = 100us;
    Stream stream = rig.device.start_stream(config);
    CHECK(rig.fake.stream_period_us() == 100);
  }
  SECTION("current firmware reports 1 us: 20 kHz and beyond") {
    FakeBoard board = streaming_board();
    board.stream_min_period_us = StreamMinPeriodUs;
    Rig rig(board, fast_options());
    rig.device.pin_mode(19, PinMode::AnalogIn);
    StreamConfig config;
    config.pins = {19};
    config.period = 50us;
    Stream stream = rig.device.start_stream(config);
    CHECK(rig.fake.stream_period_us() == 50);
    stream.stop();
    config.period = 1us;
    Stream fastest = rig.device.start_stream(config);
    CHECK(rig.fake.stream_period_us() == 1);
  }
  SECTION("a board reporting a larger minimum is held to it") {
    FakeBoard board = streaming_board();
    board.stream_min_period_us = 500;
    Rig rig(board, fast_options());
    rig.device.pin_mode(19, PinMode::AnalogIn);
    rig.fake.clear_log();
    StreamConfig config;
    config.pins = {19};
    config.period = 499us;
    CHECK_THROWS_AS(rig.device.start_stream(config), InvalidValue);
    CHECK(rig.fake.log().empty());
    config.period = 500us;
    Stream stream = rig.device.start_stream(config);
    CHECK(rig.fake.stream_period_us() == 500);
  }
}

TEST_CASE("start_stream while a Stream is already running throws DeviceBusy",
          "[stream][api]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(19, PinMode::AnalogIn);
  rig.device.pin_mode(20, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {19};
  Stream stream = rig.device.start_stream(config);

  StreamConfig other;
  other.pins = {20};
  CHECK_THROWS_AS(rig.device.start_stream(other), DeviceBusy);
  stream.stop();
}

// ---- DeviceBusy while a Stream runs ----------------------------------------

TEST_CASE("every ordinary Device call throws DeviceBusy while a Stream runs",
          "[stream][api]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(19, PinMode::AnalogIn);
  rig.device.pin_mode(2, PinMode::Output);
  StreamConfig config;
  config.pins = {19};
  Stream stream = rig.device.start_stream(config);

  CHECK_THROWS_AS(rig.device.pin_mode(0, PinMode::Input), DeviceBusy);
  CHECK_THROWS_AS(rig.device.digital_write(2, true), DeviceBusy);
  CHECK_THROWS_AS(rig.device.digital_read(2), DeviceBusy);
  CHECK_THROWS_AS(rig.device.read_all_digital(), DeviceBusy);
  CHECK_THROWS_AS(rig.device.analog_read(19), DeviceBusy);
  CHECK_THROWS_AS(rig.device.read_all_analog(), DeviceBusy);
  CHECK_THROWS_AS(rig.device.pwm_write(0, 0), DeviceBusy);
  CHECK_THROWS_AS(rig.device.dac_write(21, 0), DeviceBusy); // A6, the DAC pin
  CHECK_THROWS_AS(rig.device.status(), DeviceBusy);
  CHECK_THROWS_AS(rig.device.sync(), DeviceBusy);
  CHECK_THROWS_AS(rig.device.reset(), DeviceBusy);
  CHECK_THROWS_AS(rig.device.start_stream(config), DeviceBusy);

  // Pure accessors (no USB traffic) stay usable.
  CHECK(rig.device.pin_count() == 26);
  CHECK_NOTHROW(rig.device.pin_caps(19));
  CHECK_NOTHROW(rig.device.info());

  // Stream::stats()/stop() are exactly the calls that ARE allowed through.
  CHECK_NOTHROW(stream.stats());
  CHECK_NOTHROW(stream.stop());

  // Once stopped, the Device is usable again.
  CHECK_NOTHROW(rig.device.digital_read(2));
  CHECK_NOTHROW(rig.device.status());
}

// ---- RAII teardown ----------------------------------------------------------

TEST_CASE("the Stream destructor stops the device", "[stream][api]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(19, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {19};

  {
    Stream stream = rig.device.start_stream(config);
    CHECK(rig.fake.stream_running());
  }
  CHECK_FALSE(rig.fake.stream_running());
  REQUIRE_FALSE(rig.fake.log().empty());
  CHECK(rig.fake.log().back().request == USBIO_REQ_STREAM_STOP);
  // The Device is unlocked again.
  CHECK_NOTHROW(rig.device.pin_mode(0, PinMode::Input));
}

TEST_CASE("Stream is move-only and moving transfers ownership",
          "[stream][api]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(19, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {19};

  Stream stream = rig.device.start_stream(config);
  Stream moved = std::move(stream);
  CHECK(moved.running());
  CHECK(moved.pins() == std::vector<std::uint8_t>{19});
  moved.stop();
  CHECK_FALSE(rig.fake.stream_running());
}

TEST_CASE("RESET stops a running stream and clears its selection",
          "[stream][api]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(19, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {19};
  Stream stream = rig.device.start_stream(config);
  stream.stop();
  rig.device.reset();
  CHECK(rig.fake.stream_selected().empty());
}
