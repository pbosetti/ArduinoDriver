// test_c_api.cpp - the C ABI wrapper against the fake firmware (no
// hardware needed): argument checks, the happy path for each entry point,
// and exception -> adrv_status_t mapping.
#include "TestBackdoor.h"
#include "TestRig.h"

#include "arduino_driver_c.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using ArduinoDriver::Testing::FakeBoard;
using ArduinoDriver::Testing::fast_options;
using ArduinoDriver::Testing::Rig;
using Catch::Matchers::WithinAbs;

namespace {

// RAII around an adrv_device_t built on a fake UNO R4 Minima: pins 0..19
// are DIO, 3/5/6/9/10/11 are also PWM, 14..19 are also analog input, and 14
// is also DAC (see FakeTransport.cpp's uno_r4_minima()).
struct DeviceHandle {
  DeviceHandle()
      : rig(FakeBoard::uno_r4_minima(), fast_options()),
        handle(adrv_test_wrap_device(std::move(rig.device))) {}
  ~DeviceHandle() { adrv_device_close(handle); }
  DeviceHandle(const DeviceHandle &) = delete;

  Rig rig;
  adrv_device_t *handle;
};

} // namespace

TEST_CASE("null arguments return ADRV_ERR_INVALID_ARGUMENT", "[c_api]") {
  CHECK(adrv_device_open_first(nullptr, nullptr) == ADRV_ERR_INVALID_ARGUMENT);
  CHECK(adrv_pin_mode(nullptr, 0, ADRV_PIN_OUTPUT) ==
        ADRV_ERR_INVALID_ARGUMENT);
  bool value = false;
  CHECK(adrv_digital_read(nullptr, 0, &value) == ADRV_ERR_INVALID_ARGUMENT);
  CHECK(adrv_digital_read(nullptr, 0, nullptr) == ADRV_ERR_INVALID_ARGUMENT);
  std::string message = adrv_last_error_message();
  CHECK_FALSE(message.empty());
}

TEST_CASE("pin_mode + digital I/O round-trip", "[c_api]") {
  DeviceHandle dev;

  CHECK(adrv_pin_mode(dev.handle, 0, ADRV_PIN_OUTPUT) == ADRV_OK);
  CHECK(adrv_digital_write(dev.handle, 0, true) == ADRV_OK);
  bool value = false;
  CHECK(adrv_digital_read(dev.handle, 0, &value) == ADRV_OK);
  CHECK(value);

  CHECK(adrv_digital_write(dev.handle, 0, false) == ADRV_OK);
  CHECK(adrv_digital_read(dev.handle, 0, &value) == ADRV_OK);
  CHECK_FALSE(value);
}

TEST_CASE("read_all_digital reports the pin count regardless of buffer size",
          "[c_api]") {
  DeviceHandle dev;
  CHECK(adrv_pin_mode(dev.handle, 0, ADRV_PIN_OUTPUT) == ADRV_OK);
  CHECK(adrv_digital_write(dev.handle, 0, true) == ADRV_OK);

  bool small[1] = {false};
  size_t n_written = 0;
  CHECK(adrv_read_all_digital(dev.handle, small, 1, &n_written) == ADRV_OK);
  CHECK(n_written == adrv_pin_count(dev.handle));
  CHECK(small[0]);
}

TEST_CASE("analog_read_volts scales the raw ADC code", "[c_api]") {
  DeviceHandle dev;
  CHECK(adrv_pin_mode(dev.handle, 14, ADRV_PIN_ANALOG_IN) == ADRV_OK);
  dev.rig.fake.set_analog(14, 8192); // mid-scale of a 14-bit ADC

  double volts = -1.0;
  CHECK(adrv_analog_read_volts(dev.handle, 14, &volts) == ADRV_OK);
  CHECK_THAT(volts, WithinAbs(2.5, 0.05));
}

TEST_CASE("pwm_write_fraction and dac_write_volts accept in-range values",
          "[c_api]") {
  DeviceHandle dev;
  CHECK(adrv_pin_mode(dev.handle, 3, ADRV_PIN_PWM) == ADRV_OK);
  CHECK(adrv_pwm_write_fraction(dev.handle, 3, 0.5) == ADRV_OK);

  CHECK(adrv_pin_mode(dev.handle, 14, ADRV_PIN_DAC) == ADRV_OK);
  CHECK(adrv_dac_write_volts(dev.handle, 14, 1.0) == ADRV_OK);
}

TEST_CASE("invalid pin maps to ADRV_ERR_INVALID_PIN", "[c_api]") {
  DeviceHandle dev;
  bool value = false;
  const adrv_status_t status = adrv_digital_read(dev.handle, 250, &value);
  CHECK(status == ADRV_ERR_INVALID_PIN);
  CHECK_FALSE(std::string(adrv_last_error_message()).empty());
}

TEST_CASE("digital_write before pin_mode maps to ADRV_ERR_INVALID_MODE",
          "[c_api]") {
  DeviceHandle dev;
  CHECK(adrv_digital_write(dev.handle, 0, true) == ADRV_ERR_INVALID_MODE);
}

TEST_CASE("get_info reports the fake board's pin count", "[c_api]") {
  DeviceHandle dev;
  adrv_info_t info{};
  CHECK(adrv_device_get_info(dev.handle, &info) == ADRV_OK);
  CHECK(info.n_pins == 20);
  CHECK(std::string(adrv_board_name(info.board_id)) == "UNO R4 Minima");
}

TEST_CASE("sync and reset succeed on an idle device", "[c_api]") {
  DeviceHandle dev;
  CHECK(adrv_sync(dev.handle) == ADRV_OK);
  CHECK(adrv_reset(dev.handle) == ADRV_OK);
}

namespace {
FakeBoard streaming_board() {
  FakeBoard board = FakeBoard::portenta_h7();
  board.flags |= USBIO_FLAG_STREAMING;
  board.stream_max_channels = 4;
  return board;
}
} // namespace

TEST_CASE("start_stream + poll read samples, then stop", "[c_api][stream]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(19, ArduinoDriver::PinMode::AnalogIn); // A4
  adrv_device_t *handle = adrv_test_wrap_device(std::move(rig.device));

  const uint8_t pins[] = {19};
  adrv_stream_config_t config{};
  config.pins = pins;
  config.n_pins = 1;
  config.period_us = 1000;

  adrv_stream_t *stream = nullptr;
  REQUIRE(adrv_start_stream(handle, &config, &stream) == ADRV_OK);
  REQUIRE(stream != nullptr);
  CHECK(adrv_stream_running(stream));

  // The device call above is now busy streaming: ordinary I/O must fail.
  bool value = false;
  CHECK(adrv_digital_read(handle, 0, &value) == ADRV_ERR_DEVICE_BUSY);

  adrv_stream_stop(stream);
  CHECK_FALSE(adrv_stream_running(stream));

  adrv_stream_destroy(stream);
  adrv_device_close(handle);
}
