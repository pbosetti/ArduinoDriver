// test_device_api.cpp - Device behaviour against the fake firmware: handshake
// validation and not-ready retry, local argument checks, scaling, bulk reads,
// BUSY retry, STALL translation, sync() and reset().
#include "TestRig.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

using namespace ArduinoDriver;
using ArduinoDriver::Testing::FakeBoard;
using ArduinoDriver::Testing::FakeTransport;
using ArduinoDriver::Testing::fast_options;
using ArduinoDriver::Testing::Rig;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

using namespace std::chrono_literals;

// ---- Construction -----------------------------------------------------------

TEST_CASE("construction validates the handshake", "[construction]") {
  auto transport = std::make_unique<FakeTransport>(FakeBoard::uno_r4_minima());
  FakeTransport &fake = *transport;

  SECTION("wrong magic -> ProtocolError") {
    fake.set_magic("UIO2");
    CHECK_THROWS_AS(Device(std::move(transport), fast_options()),
                    ProtocolError);
  }
  SECTION("wrong protocol version -> ProtocolError") {
    fake.set_protocol_version(0x0002);
    CHECK_THROWS_AS(Device(std::move(transport), fast_options()),
                    ProtocolError);
  }
  SECTION("short GET_INFO reply -> ProtocolError") {
    fake.truncate_next_reply(Request::GetInfo, 23);
    CHECK_THROWS_AS(Device(std::move(transport), fast_options()),
                    ProtocolError);
  }
  SECTION("short GET_PIN_CAPS reply -> ProtocolError") {
    fake.truncate_next_reply(Request::GetPinCaps, 19);
    CHECK_THROWS_AS(Device(std::move(transport), fast_options()),
                    ProtocolError);
  }
  SECTION("a valid device constructs") {
    const Device device(std::move(transport), fast_options());
    CHECK(device.info().board_id == BoardId::UnoR4Minima);
    CHECK(device.pin_count() == 20);
  }
}

TEST_CASE("construction cross-checks n_ain with the capability table",
          "[construction]") {
  FakeBoard board = FakeBoard::uno_r4_minima();
  board.n_ain_override = 3;
  CHECK_THROWS_AS(Rig(board), ProtocolError);
}

TEST_CASE("construction rejects a null transport", "[construction]") {
  CHECK_THROWS_AS(Device(nullptr, fast_options()), std::invalid_argument);
}

TEST_CASE("construction waits for a sketch that has not called begin()",
          "[construction][ready]") {
  Device::Options options = fast_options();
  options.ready_max_attempts = 3;
  // The fake outlives the Device so that the log survives a failed
  // construction (a Device destroys the transport it was given).
  FakeTransport fake(FakeBoard::uno_r4_minima());
  auto borrowed = [&fake] {
    return std::make_unique<ArduinoDriver::Testing::BorrowedTransport>(fake);
  };

  SECTION("ready within the budget") {
    fake.set_not_ready(2);
    const Device device(borrowed(), options);
    CHECK(device.pin_count() == 20);
    CHECK(fake.count(Request::GetInfo) == 3);
    CHECK(fake.count(Request::GetPinCaps) == 1);
  }
  SECTION("exhaustion -> NotReady, before GET_PIN_CAPS") {
    fake.set_not_ready(3);
    CHECK_THROWS_AS(Device(borrowed(), options), NotReady);
    CHECK(fake.count(Request::GetInfo) == 3);
    CHECK(fake.count(Request::GetPinCaps) == 0);
  }
  SECTION("ready_max_attempts = 0 still tries once") {
    options.ready_max_attempts = 0;
    fake.set_not_ready(1);
    CHECK_THROWS_AS(Device(borrowed(), options), NotReady);
    CHECK(fake.count(Request::GetInfo) == 1);
  }
  SECTION("magic and version are checked even while not ready") {
    fake.set_not_ready(1);
    fake.set_protocol_version(0x0002);
    CHECK_THROWS_AS(Device(borrowed(), options), ProtocolError);
    CHECK(fake.count(Request::GetInfo) == 1);
  }
  SECTION("the ready back-off is honoured") {
    options.ready_delay = 2ms;
    fake.set_not_ready(2);
    const auto start = std::chrono::steady_clock::now();
    const Device device(borrowed(), options);
    CHECK(std::chrono::steady_clock::now() - start >= 4ms);
  }
  SECTION("NotReady is an Error") {
    fake.set_not_ready(10);
    CHECK_THROWS_AS(Device(borrowed(), options), Error);
  }
}

TEST_CASE("Device is movable", "[construction]") {
  auto transport = std::make_unique<FakeTransport>(FakeBoard::uno_r4_minima());
  FakeTransport &fake = *transport;
  Device first(std::move(transport), fast_options());
  Device second(std::move(first));
  second.pin_mode(2, PinMode::Output);
  CHECK(fake.mode(2) == PinMode::Output);
  CHECK(second.pin_count() == 20);
  CHECK(&second.transport() == &fake);
}

// ---- Static information -----------------------------------------------------

TEST_CASE("info() and capabilities mirror the board", "[info]") {
  SECTION("UNO R4 Minima") {
    Rig rig;
    const Info &info = rig.device.info();
    CHECK(info.protocol_version == ProtocolVersion);
    CHECK(info.board_id == BoardId::UnoR4Minima);
    CHECK(info.n_pins == 20);
    CHECK(info.n_ain == 6);
    CHECK(info.adc_bits == 14);
    CHECK(info.pwm_bits == 12);
    CHECK(info.dac_bits == 12);
    CHECK(info.queue_depth == 32);
    CHECK(info.vref_mv == 5000);
    CHECK(info.io_mv == 5000);
    CHECK_FALSE(info.supports_pulldown()); // the Renesas core lacks it
    CHECK_FALSE(info.has_vendor_interface());
    CHECK(info.has_dac());

    CHECK(rig.device.pin_count() == 20);
    CHECK(rig.device.all_pin_caps().size() == 20);
    CHECK(rig.device.analog_pins() ==
          std::vector<std::uint8_t>{14, 15, 16, 17, 18, 19});
    CHECK(rig.device.pin_caps(14) ==
          PinCaps{PinCaps::Dio | PinCaps::Ain | PinCaps::Dac});
    CHECK(rig.device.pin_caps(15) == PinCaps{PinCaps::Dio | PinCaps::Ain});
    CHECK(rig.device.pin_caps(3) == PinCaps{PinCaps::Dio | PinCaps::Pwm});
    CHECK(rig.device.pin_caps(2) == PinCaps{PinCaps::Dio});
    CHECK_THROWS_AS(rig.device.pin_caps(20), InvalidPin);
  }
  SECTION("Portenta H7") {
    Rig rig(FakeBoard::portenta_h7());
    const Info &info = rig.device.info();
    CHECK(info.board_id == BoardId::PortentaH7);
    CHECK(info.n_pins == 26);
    CHECK(info.n_ain == 7);
    CHECK(info.adc_bits == 16);
    CHECK(info.vref_mv == 3300);
    CHECK(info.io_mv == 3300);
    CHECK(info.has_vendor_interface());
    CHECK(info.supports_pulldown());
    CHECK(rig.device.analog_pins() ==
          std::vector<std::uint8_t>{15, 16, 17, 18, 19, 20, 21});
    CHECK(rig.device.pin_caps(0) == PinCaps{PinCaps::Dio | PinCaps::Pwm});
    CHECK(rig.device.pin_caps(14) == PinCaps{PinCaps::Dio | PinCaps::Pwm});
    CHECK(rig.device.pin_caps(15) == PinCaps{PinCaps::Ain}); // A0: ADC only
    CHECK(rig.device.pin_caps(18) == PinCaps{PinCaps::Ain});
    CHECK(rig.device.pin_caps(19) == PinCaps{PinCaps::Dio | PinCaps::Ain});
    CHECK(rig.device.pin_caps(21) ==
          PinCaps{PinCaps::Dio | PinCaps::Ain | PinCaps::Dac});
    CHECK(rig.device.pin_caps(22) == PinCaps{PinCaps::Dio});
    CHECK(rig.device.pin_caps(23) == PinCaps{PinCaps::Dio | PinCaps::Pwm});
    CHECK(rig.device.pin_caps(25) == PinCaps{PinCaps::Dio | PinCaps::Pwm});
    CHECK_THROWS_AS(rig.device.pin_caps(26), InvalidPin);
  }
}

// ---- Local validation -------------------------------------------------------

TEST_CASE("local validation rejects bad calls before any USB traffic",
          "[validation]") {
  Rig rig;
  Device &dev = rig.device;
  rig.fake.clear_log();

  CHECK_THROWS_AS(dev.pin_mode(20, PinMode::Output), InvalidPin);
  CHECK_THROWS_AS(dev.pin_mode(255, PinMode::Input), InvalidPin);
  CHECK_THROWS_AS(dev.pin_mode(0, PinMode::AnalogIn), NotSupported);
  CHECK_THROWS_AS(dev.pin_mode(2, PinMode::Pwm), NotSupported);
  CHECK_THROWS_AS(dev.pin_mode(15, PinMode::Dac), NotSupported);
  CHECK_THROWS_AS(dev.pin_mode(0, PinMode::InputPulldown), NotSupported);
  CHECK_THROWS_AS(dev.pin_mode(3, static_cast<PinMode>(7)), InvalidMode);

  CHECK_THROWS_AS(dev.digital_write(20, true), InvalidPin);
  CHECK_THROWS_AS(dev.digital_read(99), InvalidPin);
  CHECK_THROWS_AS(dev.analog_read(0), NotSupported);
  CHECK_THROWS_AS(dev.analog_read_volts(1), NotSupported);
  CHECK_THROWS_AS(dev.analog_read(20), InvalidPin);

  CHECK_THROWS_AS(dev.pwm_write(2, 0), NotSupported);
  CHECK_THROWS_AS(dev.pwm_write(3, 4096), InvalidValue);
  CHECK_THROWS_AS(dev.pwm_write_fraction(3, 1.01), InvalidValue);
  CHECK_THROWS_AS(dev.pwm_write_fraction(3, -0.01), InvalidValue);
  CHECK_THROWS_AS(
      dev.pwm_write_fraction(3, std::numeric_limits<double>::quiet_NaN()),
      InvalidValue);
  CHECK_THROWS_AS(dev.pwm_write_fraction(2, 0.5), NotSupported);

  CHECK_THROWS_AS(dev.dac_write(15, 0), NotSupported);
  CHECK_THROWS_AS(dev.dac_write(14, 4096), InvalidValue);
  CHECK_THROWS_AS(dev.dac_write_volts(14, 5.01), InvalidValue);
  CHECK_THROWS_AS(dev.dac_write_volts(14, -0.5), InvalidValue);
  CHECK_THROWS_AS(
      dev.dac_write_volts(14, std::numeric_limits<double>::infinity()),
      InvalidValue);

  CHECK(rig.fake.log().empty());

  SECTION("Portenta: analog-only pads have no digital side") {
    Rig portenta(FakeBoard::portenta_h7());
    portenta.fake.clear_log();
    CHECK_THROWS_AS(portenta.device.pin_mode(15, PinMode::Input), NotSupported);
    CHECK_THROWS_AS(portenta.device.pin_mode(15, PinMode::Output),
                    NotSupported);
    CHECK_THROWS_AS(portenta.device.digital_read(15), NotSupported);
    CHECK_THROWS_AS(portenta.device.digital_write(15, true), NotSupported);
    CHECK_THROWS_AS(portenta.device.pin_mode(22, PinMode::Pwm), NotSupported);
    CHECK(portenta.fake.log().empty());
    CHECK_NOTHROW(portenta.device.pin_mode(15, PinMode::AnalogIn));
    CHECK_NOTHROW(portenta.device.pin_mode(0, PinMode::InputPulldown));
    CHECK(portenta.fake.log().size() == 2);
  }
  SECTION("a board without a DAC rejects dac_write even on a DAC-flagged pin") {
    FakeBoard board = FakeBoard::uno_r4_minima();
    board.dac_bits = 0;
    Rig no_dac(board);
    no_dac.fake.clear_log();
    CHECK_THROWS_AS(no_dac.device.dac_write(14, 0), NotSupported);
    CHECK_THROWS_AS(no_dac.device.dac_write_volts(14, 0.0), NotSupported);
    CHECK(no_dac.fake.log().empty());
  }
}

// ---- Scaling ----------------------------------------------------------------

TEST_CASE("volts and fractions scale with the board resolution", "[scaling]") {
  SECTION("UNO R4 Minima: 14-bit ADC on 5 V, 12-bit PWM and DAC") {
    Rig rig;
    rig.device.pin_mode(14, PinMode::AnalogIn);
    rig.fake.set_analog(14, 16383);
    CHECK_THAT(rig.device.analog_read_volts(14), WithinAbs(5.0, 1e-9));
    rig.fake.set_analog(14, 0);
    CHECK_THAT(rig.device.analog_read_volts(14), WithinAbs(0.0, 1e-12));
    rig.fake.set_analog(14, 8191);
    CHECK_THAT(rig.device.analog_read_volts(14),
               WithinRel(5.0 * 8191 / 16383, 1e-12));
    CHECK_THAT(rig.device.to_volts(16383), WithinAbs(5.0, 1e-9));

    rig.device.pin_mode(3, PinMode::Pwm);
    rig.device.pwm_write_fraction(3, 0.0);
    CHECK(rig.fake.pwm_value(3) == 0);
    rig.device.pwm_write_fraction(3, 1.0);
    CHECK(rig.fake.pwm_value(3) == 4095);
    rig.device.pwm_write_fraction(3, 0.5);
    CHECK(rig.fake.pwm_value(3) == 2048); // round(2047.5)
    rig.device.pwm_write_fraction(3, 0.25);
    CHECK(rig.fake.pwm_value(3) == 1024); // round(1023.75)

    rig.device.pin_mode(14, PinMode::Dac);
    rig.device.dac_write_volts(14, 5.0);
    CHECK(rig.fake.dac_value(14) == 4095);
    rig.device.dac_write_volts(14, 2.5);
    CHECK(rig.fake.dac_value(14) == 2048);
    rig.device.dac_write_volts(14, 0.0);
    CHECK(rig.fake.dac_value(14) == 0);
    rig.device.dac_write(14, 1000);
    CHECK(rig.fake.dac_value(14) == 1000);
  }
  SECTION("Portenta H7: 16-bit ADC on 3.3 V, 12-bit DAC") {
    Rig rig(FakeBoard::portenta_h7());
    rig.device.pin_mode(15, PinMode::AnalogIn);
    rig.fake.set_analog(15, 65535);
    CHECK_THAT(rig.device.analog_read_volts(15), WithinAbs(3.3, 1e-9));
    rig.fake.set_analog(15, 32768);
    CHECK_THAT(rig.device.analog_read_volts(15),
               WithinRel(3.3 * 32768 / 65535, 1e-12));

    rig.device.pin_mode(21, PinMode::Dac);
    rig.device.dac_write_volts(21, 3.3);
    CHECK(rig.fake.dac_value(21) == 4095);
    rig.device.dac_write_volts(21, 1.65);
    CHECK(rig.fake.dac_value(21) == 2048);
    CHECK_THROWS_AS(rig.device.dac_write_volts(21, 3.31), InvalidValue);
  }
  SECTION("10-bit ADC, 8-bit PWM defaults of a plain board") {
    FakeBoard board;
    board.caps.assign(2, static_cast<std::uint8_t>(PinCaps::Dio | PinCaps::Ain |
                                                   PinCaps::Pwm));
    board.vref_mv = 3300;
    Rig rig(board);
    rig.device.pin_mode(0, PinMode::AnalogIn);
    rig.fake.set_analog(0, 1023);
    CHECK_THAT(rig.device.analog_read_volts(0), WithinAbs(3.3, 1e-9));
    rig.device.pin_mode(1, PinMode::Pwm);
    rig.device.pwm_write_fraction(1, 1.0);
    CHECK(rig.fake.pwm_value(1) == 255);
    CHECK_THROWS_AS(rig.device.pwm_write(1, 256), InvalidValue);
  }
}

// ---- Reads ------------------------------------------------------------------

TEST_CASE("pins are unconfigured after boot", "[read]") {
  Rig rig;
  CHECK_FALSE(rig.fake.mode(2).has_value());
  CHECK_THROWS_AS(rig.device.digital_read(2), InvalidMode);
  CHECK_THROWS_AS(rig.device.digital_write(2, true), InvalidMode);
  CHECK_THROWS_AS(rig.device.analog_read(14), InvalidMode);
  rig.fake.set_digital(2, true);
  const std::vector<bool> levels = rig.device.read_all_digital();
  CHECK(std::none_of(levels.begin(), levels.end(), [](bool b) { return b; }));
}

TEST_CASE("single reads return the device's shadow values", "[read]") {
  Rig rig;
  rig.device.pin_mode(2, PinMode::Input);
  rig.fake.set_digital(2, true);
  CHECK(rig.device.digital_read(2));
  rig.fake.set_digital(2, false);
  CHECK_FALSE(rig.device.digital_read(2));

  rig.device.pin_mode(5, PinMode::Output);
  rig.device.digital_write(5, true);
  CHECK(rig.fake.digital_value(5));
  CHECK(rig.device.digital_read(5)); // OUTPUT pins report the last write

  rig.device.pin_mode(16, PinMode::AnalogIn);
  rig.fake.set_analog(16, 12345);
  CHECK(rig.device.analog_read(16) == 12345);
}

TEST_CASE("read_all_digital unpacks the bitmap: bit i%8 of byte i/8 is pin i",
          "[read-all]") {
  Rig rig;
  constexpr std::array<std::uint8_t, 4> high_pins{0, 7, 8, 19};
  for (const std::uint8_t pin : high_pins) {
    rig.device.pin_mode(pin, PinMode::Input);
    rig.fake.set_digital(pin, true);
  }
  rig.device.pin_mode(1, PinMode::Input);
  rig.fake.set_digital(1, false);
  rig.device.pin_mode(14, PinMode::AnalogIn);
  rig.fake.set_digital(14, true); // not a digital mode: must read 0
  rig.fake.set_digital(3, true);  // unconfigured: must read 0

  // The wire image, independently of the driver's unpacking.
  std::array<std::byte, 5> raw{};
  REQUIRE(rig.fake.control_in(USBIO_REQ_DIO_READ_ALL, 0, 0, raw, 100ms) == 5);
  CHECK(read_u8(raw, 0) == 0);    // status OK
  CHECK(read_u8(raw, 2) == 0x81); // pins 0 and 7
  CHECK(read_u8(raw, 3) == 0x01); // pin 8
  CHECK(read_u8(raw, 4) == 0x08); // pin 19

  const std::vector<bool> levels = rig.device.read_all_digital();
  REQUIRE(levels.size() == 20);
  CHECK(levels[0]);
  CHECK(levels[7]);
  CHECK(levels[8]);
  CHECK(levels[19]);
  CHECK_FALSE(levels[1]);
  CHECK_FALSE(levels[3]);
  CHECK_FALSE(levels[14]);
  std::size_t high = 0;
  for (const bool level : levels) {
    high += level ? 1 : 0;
  }
  CHECK(high == 4);
}

TEST_CASE("read_all_analog returns one sample per analog pin, ascending",
          "[read-all]") {
  Rig rig(FakeBoard::portenta_h7());
  for (std::uint8_t pin = 15; pin <= 21; ++pin) {
    if (pin != 18) {
      rig.device.pin_mode(pin, PinMode::AnalogIn);
    }
    rig.fake.set_analog(pin, static_cast<std::uint16_t>(1000 + pin));
  }

  std::array<std::byte, 16> raw{};
  REQUIRE(rig.fake.control_in(USBIO_REQ_AI_READ_ALL, 0, 0, raw, 100ms) == 16);
  CHECK(read_u8(raw, 0) == 0);
  CHECK(read_u16le(raw, 2) == 1015);
  CHECK(read_u16le(raw, 8) == 0); // pin 18 is not in ANALOG_IN mode
  CHECK(read_u16le(raw, 14) == 1021);

  const std::vector<std::uint16_t> samples = rig.device.read_all_analog();
  CHECK(samples ==
        std::vector<std::uint16_t>{1015, 1016, 1017, 0, 1019, 1020, 1021});
  CHECK(samples.size() == rig.device.analog_pins().size());
}

// ---- BUSY -------------------------------------------------------------------

TEST_CASE("BUSY replies are retried up to busy_max_attempts", "[busy]") {
  Device::Options options = fast_options();
  options.busy_max_attempts = 3;

  SECTION("success within the budget") {
    Rig rig(FakeBoard::uno_r4_minima(), options);
    rig.device.pin_mode(2, PinMode::Input);
    rig.fake.set_digital(2, true);
    rig.fake.clear_log();
    rig.fake.inject_busy(2);
    CHECK(rig.device.digital_read(2));
    CHECK(rig.fake.count(Request::DioRead) == 3);
    CHECK(rig.fake.log().size() == 3);
  }
  SECTION("exhaustion -> DeviceBusy") {
    Rig rig(FakeBoard::uno_r4_minima(), options);
    rig.device.pin_mode(2, PinMode::Input);
    rig.fake.clear_log();
    rig.fake.inject_busy(3);
    CHECK_THROWS_AS(rig.device.digital_read(2), DeviceBusy);
    CHECK(rig.fake.count(Request::DioRead) == 3);
    CHECK(rig.fake.count(Request::GetStatus) == 0);
    // The device is free again: the next read succeeds.
    CHECK_NOTHROW(rig.device.digital_read(2));
  }
  SECTION("busy_max_attempts = 0 still tries once") {
    options.busy_max_attempts = 0;
    Rig rig(FakeBoard::uno_r4_minima(), options);
    rig.fake.clear_log();
    rig.fake.inject_busy(1);
    CHECK_THROWS_AS(rig.device.read_all_digital(), DeviceBusy);
    CHECK(rig.fake.count(Request::DioReadAll) == 1);
  }
  SECTION("bulk reads and analog reads retry too") {
    Rig rig(FakeBoard::uno_r4_minima(), options);
    rig.device.pin_mode(14, PinMode::AnalogIn);
    rig.fake.set_analog(14, 42);
    rig.fake.clear_log();
    rig.fake.inject_busy(1);
    CHECK(rig.device.analog_read(14) == 42);
    CHECK(rig.fake.count(Request::AiRead) == 2);
    rig.fake.inject_busy(2);
    CHECK(rig.device.read_all_analog()[0] == 42);
    CHECK(rig.fake.count(Request::AiReadAll) == 3);
  }
  SECTION("a mode error wins over BUSY, as in the firmware") {
    Rig rig(FakeBoard::uno_r4_minima(), options);
    rig.fake.inject_busy(1);
    CHECK_THROWS_AS(rig.device.digital_read(2), InvalidMode); // unconfigured
    rig.device.pin_mode(2, PinMode::Input);
    rig.fake.clear_log();
    CHECK_NOTHROW(rig.device.digital_read(2)); // the pending BUSY is consumed
    CHECK(rig.fake.count(Request::DioRead) == 2);
  }
  SECTION("the default back-off is honoured") {
    Device::Options slow = fast_options();
    slow.busy_max_attempts = 3;
    slow.busy_delay = 2ms;
    Rig rig(FakeBoard::uno_r4_minima(), slow);
    rig.fake.inject_busy(2);
    const auto start = std::chrono::steady_clock::now();
    rig.device.read_all_digital();
    CHECK(std::chrono::steady_clock::now() - start >= 4ms);
  }
}

// ---- STALL translation ------------------------------------------------------

TEST_CASE("a STALLed OUT request is explained through GET_STATUS", "[stall]") {
  Rig rig;

  SECTION("DIO_WRITE on an INPUT pin -> InvalidMode") {
    rig.device.pin_mode(2, PinMode::Input);
    rig.fake.clear_log();
    CHECK_THROWS_AS(rig.device.digital_write(2, true), InvalidMode);
    REQUIRE(rig.fake.log().size() == 2);
    CHECK(rig.fake.log()[0].request == USBIO_REQ_DIO_WRITE);
    CHECK(rig.fake.log()[1].request == USBIO_REQ_GET_STATUS);
    CHECK(rig.fake.last_error() == Status::Ok); // consumed by the read
    CHECK(rig.device.status() == Status::Ok);
  }
  SECTION("PWM_WRITE / DAC_WRITE on a pin in another mode -> InvalidMode") {
    rig.device.pin_mode(3, PinMode::Output);
    CHECK_THROWS_AS(rig.device.pwm_write(3, 10), InvalidMode);
    rig.device.pin_mode(14, PinMode::AnalogIn);
    CHECK_THROWS_AS(rig.device.dac_write(14, 10), InvalidMode);
  }
  SECTION("queue full -> QueueFull, cleared by RESET") {
    rig.fake.set_queue_full(true);
    CHECK_THROWS_AS(rig.device.pin_mode(2, PinMode::Output), QueueFull);
    CHECK_FALSE(rig.fake.mode(2).has_value()); // the mode was not recorded
    rig.device.reset();
    CHECK_NOTHROW(rig.device.pin_mode(2, PinMode::Output));
  }
  SECTION("every last_error code maps to its exception") {
    rig.fake.stall_next_out(Status::BadPin);
    CHECK_THROWS_AS(rig.device.reset(), InvalidPin);
    rig.fake.stall_next_out(Status::BadMode);
    CHECK_THROWS_AS(rig.device.reset(), InvalidMode);
    rig.fake.stall_next_out(Status::Unsupported);
    CHECK_THROWS_AS(rig.device.reset(), NotSupported);
    rig.fake.stall_next_out(Status::BadValue);
    CHECK_THROWS_AS(rig.device.reset(), InvalidValue);
    rig.fake.stall_next_out(Status::QueueFull);
    CHECK_THROWS_AS(rig.device.reset(), QueueFull);
    rig.fake.stall_next_out(Status::BadCmd);
    CHECK_THROWS_AS(rig.device.reset(), ProtocolError);
    rig.fake.stall_next_out(static_cast<Status>(200));
    CHECK_THROWS_AS(rig.device.reset(), ProtocolError);
  }
  SECTION("STALL with nothing recorded surfaces as StallError") {
    rig.fake.stall_next_out(Status::Ok);
    CHECK_THROWS_AS(rig.device.reset(), StallError);
  }
  SECTION("STALL followed by a short GET_STATUS reply -> ProtocolError") {
    rig.fake.stall_next_out(Status::BadMode);
    rig.fake.truncate_next_reply(Request::GetStatus, 2);
    CHECK_THROWS_AS(rig.device.reset(), ProtocolError);
  }
  SECTION("the mapped exceptions are all Errors") {
    rig.fake.stall_next_out(Status::BadValue);
    CHECK_THROWS_AS(rig.device.reset(), Error);
  }
}

TEST_CASE("an error status in an IN reply maps directly, without GET_STATUS",
          "[stall]") {
  Rig rig;
  rig.fake.clear_log();
  // Pin 14 has the AIN capability but is unconfigured.
  CHECK_THROWS_AS(rig.device.analog_read(14), InvalidMode);
  REQUIRE(rig.fake.log().size() == 1);
  CHECK(rig.fake.log()[0].request == USBIO_REQ_AI_READ);

  rig.device.pin_mode(14, PinMode::AnalogIn);
  CHECK_THROWS_AS(rig.device.digital_read(14), InvalidMode);
  CHECK(rig.fake.count(Request::GetStatus) == 0);
}

// ---- status() / sync() / reset() --------------------------------------------

TEST_CASE("status() reports last_error and queue_pending", "[status]") {
  Rig rig;
  rig.fake.set_queue_pending(2);
  std::uint8_t pending = 99;
  CHECK(rig.device.status(&pending) == Status::Ok);
  CHECK(pending == 2);
  CHECK(rig.device.status(&pending) == Status::Ok);
  CHECK(pending == 1);

  // A STALL produced outside the driver (OUT or IN) leaves last_error.
  CHECK_THROWS_AS(rig.fake.control_out(0x55, 0, 0, 100ms), StallError);
  CHECK(rig.device.status() == Status::BadCmd);
  CHECK(rig.device.status() == Status::Ok); // cleared by the previous read
  std::array<std::byte, 4> reply{};
  CHECK_THROWS_AS(rig.fake.control_in(0x55, 0, 0, reply, 100ms), StallError);
  CHECK(rig.device.status() == Status::BadCmd);
}

TEST_CASE("sync() polls GET_STATUS until the queue drains", "[sync]") {
  Device::Options options = fast_options();
  options.busy_max_attempts = 5;
  Rig rig(FakeBoard::uno_r4_minima(), options);

  SECTION("drains within the budget") {
    rig.fake.set_queue_pending(3);
    rig.fake.clear_log();
    rig.device.sync();
    CHECK(rig.fake.count(Request::GetStatus) == 4); // 3, 2, 1, 0
    CHECK(rig.fake.queue_pending() == 0);
  }
  SECTION("an empty queue needs a single poll") {
    rig.fake.clear_log();
    rig.device.sync();
    CHECK(rig.fake.log().size() == 1);
  }
  SECTION("gives up after busy_max_attempts polls") {
    rig.fake.set_queue_pending(10);
    rig.fake.clear_log();
    CHECK_THROWS_AS(rig.device.sync(), DeviceBusy);
    CHECK(rig.fake.count(Request::GetStatus) == 5);
  }
}

TEST_CASE("reset() returns DIO pins to INPUT and clears the queue", "[reset]") {
  SECTION("UNO R4 Minima: every pin is DIO-capable") {
    Rig rig;
    rig.device.pin_mode(2, PinMode::Output);
    rig.device.pin_mode(3, PinMode::Pwm);
    rig.device.pin_mode(14, PinMode::AnalogIn);
    rig.fake.set_analog(14, 500);
    rig.fake.set_queue_pending(4);
    rig.fake.clear_log();

    rig.device.reset();
    REQUIRE(rig.fake.log().size() == 1);
    CHECK(rig.fake.log()[0].request == USBIO_REQ_RESET);
    for (std::uint8_t pin = 0; pin < rig.device.pin_count(); ++pin) {
      CHECK(rig.fake.mode(pin) == PinMode::Input);
    }
    CHECK(rig.fake.queue_pending() == 0);
    CHECK(rig.fake.analog_value(14) == 0); // analog shadow cleared
    CHECK_THROWS_AS(rig.device.digital_write(2, true), InvalidMode);
    CHECK_NOTHROW(rig.device.digital_read(2));
    CHECK_THROWS_AS(rig.device.analog_read(14), InvalidMode);
  }
  SECTION("Portenta H7: analog-only pads go back to unconfigured") {
    Rig rig(FakeBoard::portenta_h7());
    rig.device.pin_mode(15, PinMode::AnalogIn);
    rig.device.pin_mode(19, PinMode::AnalogIn);
    rig.device.reset();
    for (std::uint8_t pin = 0; pin < rig.device.pin_count(); ++pin) {
      INFO("pin " << unsigned{pin});
      if (rig.device.pin_caps(pin).dio()) {
        CHECK(rig.fake.mode(pin) == PinMode::Input);
      } else {
        CHECK_FALSE(rig.fake.mode(pin).has_value());
      }
    }
    CHECK_THROWS_AS(rig.device.analog_read(15), InvalidMode);
    CHECK_NOTHROW(rig.device.digital_read(19));
  }
}

// ---- Malformed replies ------------------------------------------------------

TEST_CASE("short or empty replies are protocol errors", "[protocol]") {
  Rig rig;
  rig.device.pin_mode(2, PinMode::Input);
  rig.device.pin_mode(14, PinMode::AnalogIn);

  rig.fake.truncate_next_reply(Request::DioRead, 1);
  CHECK_THROWS_AS(rig.device.digital_read(2), ProtocolError);
  rig.fake.truncate_next_reply(Request::DioRead, 0);
  CHECK_THROWS_AS(rig.device.digital_read(2), ProtocolError);
  rig.fake.truncate_next_reply(Request::AiRead, 3);
  CHECK_THROWS_AS(rig.device.analog_read(14), ProtocolError);
  rig.fake.truncate_next_reply(Request::DioReadAll, 4);
  CHECK_THROWS_AS(rig.device.read_all_digital(), ProtocolError);
  rig.fake.truncate_next_reply(Request::AiReadAll, 13);
  CHECK_THROWS_AS(rig.device.read_all_analog(), ProtocolError);
  rig.fake.truncate_next_reply(Request::GetStatus, 3);
  CHECK_THROWS_AS(rig.device.status(), ProtocolError);

  // The knobs are one-shot: the device is healthy again.
  CHECK_NOTHROW(rig.device.digital_read(2));
  CHECK_NOTHROW(rig.device.status());
}

// ---- Errors -----------------------------------------------------------------

TEST_CASE("UsbError carries the libusb code and its name", "[errors]") {
  const StallError stall("stalled");
  CHECK(stall.code() == LibusbError::Pipe);
  CHECK(stall.name() == "LIBUSB_ERROR_PIPE");
  CHECK(std::string_view(stall.what()) == "stalled");

  const TimeoutError timeout("late");
  CHECK(timeout.code() == LibusbError::Timeout);
  CHECK(timeout.name() == "LIBUSB_ERROR_TIMEOUT");

  CHECK(UsbError("x", LibusbError::NoDevice).name() ==
        "LIBUSB_ERROR_NO_DEVICE");
  CHECK(UsbError("x", -1234).name() == "LIBUSB_ERROR_UNKNOWN");
  CHECK(UsbError("x", 0).name() == "LIBUSB_SUCCESS");
  CHECK(usb_error_name(LibusbError::Access) == "LIBUSB_ERROR_ACCESS");

  CHECK_THROWS_AS(throw StallError("x"), UsbError);
  CHECK_THROWS_AS(throw TimeoutError("x"), Error);
  CHECK_THROWS_AS(throw QueueFull("x"), std::runtime_error);
  CHECK_THROWS_AS(throw NotReady("x"), Error);
  CHECK_THROWS_AS(throw DeviceNotFound("x"), Error);
}
