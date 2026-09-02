// test_stream_api.cpp - Device::start_stream() / Stream control-path
// behaviour against the fake firmware: select/start/stop sequencing,
// validation, the DeviceBusy contract while a Stream runs, and RAII
// teardown. Byte-stream decoding itself is covered by
// test_stream_decoding.cpp.
#include "TestRig.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdint>
#include <utility>
#include <vector>

using namespace ArduinoDriver;
using ArduinoDriver::Testing::FakeBoard;
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

  // STREAM_SELECT once per pin (in order), then STREAM_START.
  REQUIRE(rig.fake.log().size() == 3);
  CHECK(rig.fake.log()[0].request == USBIO_REQ_STREAM_SELECT);
  CHECK(rig.fake.log()[0].index == 19);
  CHECK(rig.fake.log()[0].value == 1);
  CHECK(rig.fake.log()[1].request == USBIO_REQ_STREAM_SELECT);
  CHECK(rig.fake.log()[1].index == 20);
  CHECK(rig.fake.log()[1].value == 1);
  CHECK(rig.fake.log()[2].request == USBIO_REQ_STREAM_START);
  CHECK(rig.fake.log()[2].value == 1000);

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
  SECTION("period below StreamMinPeriodUs") {
    StreamConfig config;
    config.pins = {19};
    config.period = std::chrono::microseconds{1};
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
  SECTION("period 0 (free running) is fine") {
    StreamConfig config;
    config.pins = {19};
    config.period = std::chrono::microseconds{0};
    Stream stream = rig.device.start_stream(config);
    CHECK(rig.fake.stream_period_us() == 0);
    stream.stop();
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
