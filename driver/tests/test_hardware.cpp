// test_hardware.cpp - exercises a real board running the UsbIo firmware.
//
// Hidden behind the [.hardware] tag: run with `arduino_driver_tests
// "[hardware]"` or `ctest -R hardware` after configuring with
// -DARDUINODRIVER_HARDWARE_TESTS=ON. Environment:
//   ARDUINO_IO_SERIAL     pick the board by USB serial (default: first found)
//   ARDUINO_IO_LOOPBACK   "out,in": two pins wired together for the loopback
#include "arduino_driver/Enumerator.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <thread>

using namespace ArduinoDriver;
using namespace std::chrono_literals;

namespace {

std::optional<std::string> env(const char *name) {
  const char *value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return std::nullopt;
  }
  return std::string(value);
}

Device open_board() {
  auto context = std::make_shared<Context>();
  if (const auto serial = env("ARDUINO_IO_SERIAL")) {
    return open_by_serial(context, *serial);
  }
  return open_first(context);
}

} // namespace

TEST_CASE("hardware: identify, reset, read every pin, sync", "[.hardware]") {
  Device dev = open_board();
  const Info &info = dev.info();
  WARN("board: " << board_name(info.board_id) << " (0x" << std::hex
                 << static_cast<unsigned>(info.board_id) << std::dec << "), "
                 << unsigned{info.n_pins} << " pins, " << unsigned{info.n_ain}
                 << " analog, adc " << unsigned{info.adc_bits} << " bits, vref "
                 << info.vref_mv << " mV");
  REQUIRE(info.n_pins > 0);
  REQUIRE(dev.pin_count() == info.n_pins);
  REQUIRE(dev.analog_pins().size() == info.n_ain);

  // Every DIO pin to INPUT.
  dev.reset();
  dev.sync();
  const std::vector<bool> levels = dev.read_all_digital();
  REQUIRE(levels.size() == info.n_pins);
  for (std::uint8_t pin = 0; pin < info.n_pins; ++pin) {
    if (dev.pin_caps(pin).dio()) {
      CHECK_NOTHROW(dev.digital_read(pin));
    }
  }

  // Every AIN pin sampled.
  for (const std::uint8_t pin : dev.analog_pins()) {
    dev.pin_mode(pin, PinMode::AnalogIn);
  }
  dev.sync();
  std::this_thread::sleep_for(200ms); // round-robin sampling from loop()
  const std::vector<std::uint16_t> samples = dev.read_all_analog();
  REQUIRE(samples.size() == info.n_ain);
  const std::uint16_t full_scale = max_value(info.adc_bits);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    INFO("analog pin " << unsigned{dev.analog_pins()[i]});
    CHECK(samples[i] <= full_scale);
    const std::uint16_t single = dev.analog_read(dev.analog_pins()[i]);
    CHECK(single <= full_scale);
    WARN("A" << unsigned{dev.analog_pins()[i]} << " = " << single << " ("
             << dev.to_volts(single) << " V)");
  }

  CHECK(dev.status() == Status::Ok);
  dev.reset();
  dev.sync();
  for (const std::uint8_t pin : dev.analog_pins()) {
    if (dev.pin_caps(pin).dio()) {
      CHECK_NOTHROW(dev.digital_read(pin)); // back to INPUT
    } else {
      CHECK_THROWS_AS(dev.analog_read(pin), InvalidMode); // unconfigured
    }
  }
}

TEST_CASE("hardware: loopback between two wired pins", "[.hardware]") {
  const auto loopback = env("ARDUINO_IO_LOOPBACK");
  if (!loopback) {
    SKIP("ARDUINO_IO_LOOPBACK=\"out,in\" not set");
  }
  const auto comma = loopback->find(',');
  REQUIRE(comma != std::string::npos);
  const auto out =
      static_cast<std::uint8_t>(std::stoul(loopback->substr(0, comma)));
  const auto in =
      static_cast<std::uint8_t>(std::stoul(loopback->substr(comma + 1)));

  Device dev = open_board();
  dev.pin_mode(out, PinMode::Output);
  dev.pin_mode(in, PinMode::Input);
  dev.sync();
  for (const bool level : {true, false, true, false}) {
    dev.digital_write(out, level);
    dev.sync();
    std::this_thread::sleep_for(10ms); // let loop() resample the input
    CHECK(dev.digital_read(in) == level);
  }
  dev.reset();
  dev.sync();
}
