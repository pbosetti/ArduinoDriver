// test_protocol_encoding.cpp - wire encoding of every Device call, GET_INFO
// decoding and the reply-size helpers.
#include "TestRig.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

using namespace ArduinoDriver;
using ArduinoDriver::Testing::FakeBoard;
using ArduinoDriver::Testing::LoggedRequest;
using ArduinoDriver::Testing::Rig;

namespace {

void check_request(const LoggedRequest &logged, Request request,
                   std::uint16_t value, std::uint16_t index,
                   std::uint16_t length) {
  INFO("expected " << to_string(request));
  CHECK(unsigned{logged.request} ==
        unsigned{static_cast<std::uint8_t>(request)});
  CHECK(unsigned{logged.request_type} == unsigned{request_type(request)});
  CHECK(logged.value == value);
  CHECK(logged.index == index);
  CHECK(logged.length == length);
}

FakeBoard plain_board(std::size_t n_pins) {
  FakeBoard board;
  board.name = "plain";
  board.caps.assign(n_pins, PinCaps::Dio);
  return board;
}

} // namespace

// ---- Device calls -> control transfers --------------------------------------

TEST_CASE("construction issues GET_INFO then GET_PIN_CAPS", "[encoding]") {
  SECTION("UNO R4 Minima: 20 pins") {
    Rig rig;
    REQUIRE(rig.fake.log().size() == 2);
    check_request(rig.fake.log()[0], Request::GetInfo, 0, 0, 24);
    check_request(rig.fake.log()[1], Request::GetPinCaps, 0, 0, 20);
  }
  SECTION("Portenta H7: 22 pins") {
    Rig rig(FakeBoard::portenta_h7());
    REQUIRE(rig.fake.log().size() == 2);
    check_request(rig.fake.log()[0], Request::GetInfo, 0, 0, 24);
    check_request(rig.fake.log()[1], Request::GetPinCaps, 0, 0, 26);
  }
}

TEST_CASE("PIN_MODE carries the mode in wValue and the pin in wIndex",
          "[encoding]") {
  Rig rig;
  rig.fake.clear_log();
  struct Case {
    std::uint8_t pin;
    PinMode mode;
    std::uint16_t wire_value;
  };
  const std::array<Case, 6> cases{{{0, PinMode::Input, 0},
                                   {1, PinMode::Output, 1},
                                   {2, PinMode::InputPullup, 2},
                                   {14, PinMode::AnalogIn, 4},
                                   {3, PinMode::Pwm, 5},
                                   {14, PinMode::Dac, 6}}};
  for (const Case &c : cases) {
    rig.device.pin_mode(c.pin, c.mode);
    check_request(rig.fake.last(), Request::PinMode, c.wire_value, c.pin, 0);
  }
  CHECK(rig.fake.log().size() == cases.size());

  // INPUT_PULLDOWN is only accepted by boards that support it.
  Rig portenta(FakeBoard::portenta_h7());
  portenta.device.pin_mode(4, PinMode::InputPulldown);
  check_request(portenta.fake.last(), Request::PinMode, 3, 4, 0);
}

TEST_CASE("DIO_WRITE carries the level in wValue", "[encoding]") {
  Rig rig;
  rig.device.pin_mode(2, PinMode::Output);
  rig.fake.clear_log();
  rig.device.digital_write(2, true);
  check_request(rig.fake.last(), Request::DioWrite, 1, 2, 0);
  rig.device.digital_write(2, false);
  check_request(rig.fake.last(), Request::DioWrite, 0, 2, 0);
  CHECK(rig.fake.log().size() == 2);
}

TEST_CASE("DIO_READ asks for a 2-byte reply", "[encoding]") {
  Rig rig;
  rig.device.pin_mode(7, PinMode::InputPullup);
  rig.fake.clear_log();
  rig.device.digital_read(7);
  REQUIRE(rig.fake.log().size() == 1);
  check_request(rig.fake.last(), Request::DioRead, 0, 7, 2);
}

TEST_CASE("AI_READ asks for a 4-byte reply", "[encoding]") {
  Rig rig;
  rig.device.pin_mode(16, PinMode::AnalogIn);
  rig.fake.clear_log();
  rig.device.analog_read(16);
  REQUIRE(rig.fake.log().size() == 1);
  check_request(rig.fake.last(), Request::AiRead, 0, 16, 4);
}

TEST_CASE("PWM_WRITE carries the raw duty in wValue", "[encoding]") {
  Rig rig;
  rig.device.pin_mode(9, PinMode::Pwm);
  rig.fake.clear_log();
  rig.device.pwm_write(9, 1234);
  REQUIRE(rig.fake.log().size() == 1);
  check_request(rig.fake.last(), Request::PwmWrite, 1234, 9, 0);
}

TEST_CASE("DAC_WRITE carries the raw code in wValue", "[encoding]") {
  SECTION("UNO R4 Minima: DAC on pin 14") {
    Rig rig;
    rig.device.pin_mode(14, PinMode::Dac);
    rig.fake.clear_log();
    rig.device.dac_write(14, 4095);
    REQUIRE(rig.fake.log().size() == 1);
    check_request(rig.fake.last(), Request::DacWrite, 4095, 14, 0);
  }
  SECTION("Portenta H7: DAC on pin 21") {
    Rig rig(FakeBoard::portenta_h7());
    rig.device.pin_mode(21, PinMode::Dac);
    rig.fake.clear_log();
    rig.device.dac_write(21, 100);
    check_request(rig.fake.last(), Request::DacWrite, 100, 21, 0);
  }
}

TEST_CASE("DIO_READ_ALL asks for header + bitmap", "[encoding]") {
  SECTION("20 pins -> 2 + 3 bytes") {
    Rig rig;
    rig.fake.clear_log();
    rig.device.read_all_digital();
    REQUIRE(rig.fake.log().size() == 1);
    check_request(rig.fake.last(), Request::DioReadAll, 0, 0, 5);
  }
  SECTION("9 pins -> 2 + 2 bytes") {
    Rig rig(plain_board(9));
    rig.fake.clear_log();
    rig.device.read_all_digital();
    check_request(rig.fake.last(), Request::DioReadAll, 0, 0, 4);
  }
  SECTION("1 pin -> 2 + 1 bytes") {
    Rig rig(plain_board(1));
    rig.fake.clear_log();
    rig.device.read_all_digital();
    check_request(rig.fake.last(), Request::DioReadAll, 0, 0, 3);
  }
}

TEST_CASE("AI_READ_ALL asks for header + 2 bytes per analog pin",
          "[encoding]") {
  SECTION("UNO R4 Minima: 6 analog pins -> 14 bytes") {
    Rig rig;
    rig.fake.clear_log();
    rig.device.read_all_analog();
    REQUIRE(rig.fake.log().size() == 1);
    check_request(rig.fake.last(), Request::AiReadAll, 0, 0, 14);
  }
  SECTION("Portenta H7: 7 analog pins -> 16 bytes") {
    Rig rig(FakeBoard::portenta_h7());
    rig.fake.clear_log();
    rig.device.read_all_analog();
    check_request(rig.fake.last(), Request::AiReadAll, 0, 0, 16);
  }
  SECTION("no analog pins -> header only") {
    Rig rig(plain_board(4));
    rig.fake.clear_log();
    CHECK(rig.device.read_all_analog().empty());
    check_request(rig.fake.last(), Request::AiReadAll, 0, 0, 2);
  }
}

TEST_CASE("GET_STATUS asks for a 4-byte reply", "[encoding]") {
  Rig rig;
  rig.fake.clear_log();
  rig.device.status();
  REQUIRE(rig.fake.log().size() == 1);
  check_request(rig.fake.last(), Request::GetStatus, 0, 0, 4);
}

TEST_CASE("RESET is an OUT request with empty wValue and wIndex",
          "[encoding]") {
  Rig rig;
  rig.fake.clear_log();
  rig.device.reset();
  REQUIRE(rig.fake.log().size() == 1);
  check_request(rig.fake.last(), Request::Reset, 0, 0, 0);
}

// ---- GET_INFO decoding ------------------------------------------------------

namespace {

std::array<std::byte, InfoLen>
info_bytes(const std::array<std::uint8_t, InfoLen> &raw) {
  std::array<std::byte, InfoLen> bytes{};
  for (std::size_t i = 0; i < raw.size(); ++i) {
    bytes[i] = static_cast<std::byte>(raw[i]);
  }
  return bytes;
}

// A Portenta-like usbio_info_t, written out by hand, little-endian.
constexpr std::array<std::uint8_t, InfoLen> PortentaInfo{
    'U',  'I',  'O', '1', // magic
    0x01, 0x00,           // protocol_version = 1
    0x01, 0x02,           // board_id = 0x0201 (Portenta H7)
    22,                   // n_pins
    7,                    // n_ain
    16,                   // adc_bits
    12,                   // pwm_bits
    12,                   // dac_bits
    32,                   // queue_depth
    0xE4, 0x0C,           // vref_mv = 3300
    0xE4, 0x0C,           // io_mv = 3300
    0x03, 0x00,           // flags = VENDOR_INTERFACE | PULLDOWN
    0,    0,    0,   0,   // reserved
};

} // namespace

TEST_CASE("decode_info reads usbio_info_t field by field, little-endian",
          "[protocol]") {
  static_assert(InfoLen == 24);
  const Info info = decode_info(info_bytes(PortentaInfo));
  CHECK(info.protocol_version == 1);
  CHECK(info.board_id == BoardId::PortentaH7);
  CHECK(info.n_pins == 22);
  CHECK(info.n_ain == 7);
  CHECK(info.adc_bits == 16);
  CHECK(info.pwm_bits == 12);
  CHECK(info.dac_bits == 12);
  CHECK(info.queue_depth == 32);
  CHECK(info.vref_mv == 3300);
  CHECK(info.io_mv == 3300);
  CHECK(info.flags == 3);
  CHECK(info.has_vendor_interface());
  CHECK(info.supports_pulldown());
  CHECK(info.has_dac());

  SECTION("byte order matters") {
    std::array<std::uint8_t, InfoLen> raw = PortentaInfo;
    raw[14] = 0x0C; // vref_mv bytes swapped
    raw[15] = 0xE4;
    CHECK(decode_info(info_bytes(raw)).vref_mv == 0xE40C);
  }
  SECTION("the protocol version is decoded, not enforced") {
    std::array<std::uint8_t, InfoLen> raw = PortentaInfo;
    raw[4] = 0x02;
    CHECK(decode_info(info_bytes(raw)).protocol_version == 2);
  }
  SECTION("a short buffer is a ProtocolError") {
    const auto bytes = info_bytes(PortentaInfo);
    CHECK_THROWS_AS(decode_info(std::span(bytes).first(23)), ProtocolError);
    CHECK_THROWS_AS(decode_info(std::span<const std::byte>{}), ProtocolError);
  }
  SECTION("a wrong magic is a ProtocolError") {
    std::array<std::uint8_t, InfoLen> raw = PortentaInfo;
    raw[3] = '2';
    CHECK_THROWS_AS(decode_info(info_bytes(raw)), ProtocolError);
  }
}

// ---- Helpers ----------------------------------------------------------------

TEST_CASE("reply-size helpers", "[protocol]") {
  CHECK(dio_bitmap_len(0) == 0);
  CHECK(dio_bitmap_len(1) == 1);
  CHECK(dio_bitmap_len(8) == 1);
  CHECK(dio_bitmap_len(9) == 2);
  CHECK(dio_bitmap_len(20) == 3);
  CHECK(dio_bitmap_len(22) == 3);
  CHECK(dio_bitmap_len(MaxPins) == 16);

  CHECK(dio_read_all_len(1) == 3);
  CHECK(dio_read_all_len(20) == 5);
  CHECK(dio_read_all_len(24) == 5);
  CHECK(dio_read_all_len(25) == 6);

  CHECK(ai_read_all_len(0) == 2);
  CHECK(ai_read_all_len(6) == 14);
  CHECK(ai_read_all_len(7) == 16);
  CHECK(ai_read_all_len(MaxAin) == 66);

  static_assert(dio_read_all_len(MaxPins) <= MaxReplyLen);
  static_assert(ai_read_all_len(MaxAin) <= MaxReplyLen);
}

TEST_CASE("max_value", "[protocol]") {
  CHECK(max_value(0) == 0);
  CHECK(max_value(1) == 1);
  CHECK(max_value(8) == 255);
  CHECK(max_value(10) == 1023);
  CHECK(max_value(12) == 4095);
  CHECK(max_value(14) == 16383);
  CHECK(max_value(16) == 65535);
  CHECK(max_value(17) == 65535);
}

TEST_CASE("request direction and bmRequestType", "[protocol]") {
  for (const Request out :
       {Request::PinMode, Request::DioWrite, Request::PwmWrite,
        Request::DacWrite, Request::Reset}) {
    INFO(to_string(out));
    CHECK(is_out(out));
    CHECK(request_type(out) == 0x40);
  }
  for (const Request in :
       {Request::GetInfo, Request::GetPinCaps, Request::DioRead,
        Request::AiRead, Request::DioReadAll, Request::AiReadAll,
        Request::GetStatus}) {
    INFO(to_string(in));
    CHECK_FALSE(is_out(in));
    CHECK(request_type(in) == 0xC0);
  }

  // Interface-recipient form: bit 0 of bmRequestType, pin in the high byte
  // of wIndex and the interface number in the low byte.
  CHECK(request_type(Request::DioWrite, Recipient::Interface) == 0x41);
  CHECK(request_type(Request::DioRead, Recipient::Interface) == 0xC1);
  CHECK(request_type(Request::DioWrite, Recipient::Device) == 0x40);
  CHECK(bm_request_type(true, Recipient::Device) == 0x40);
  CHECK(bm_request_type(false, Recipient::Device) == 0xC0);
  CHECK(bm_request_type(true, Recipient::Interface) == 0x41);
  CHECK(bm_request_type(false, Recipient::Interface) == 0xC1);
  CHECK(interface_index(23, 2) == 0x1702);
  CHECK(interface_index(0, 0) == 0);
  CHECK(interface_index(0xFF, 0xFF) == 0xFFFF);
}

TEST_CASE("little-endian helpers round-trip", "[protocol]") {
  std::array<std::byte, 4> bytes{};
  write_u16le(bytes, 1, 0xBEEF);
  CHECK(read_u8(bytes, 0) == 0);
  CHECK(read_u8(bytes, 1) == 0xEF);
  CHECK(read_u8(bytes, 2) == 0xBE);
  CHECK(read_u8(bytes, 3) == 0);
  CHECK(read_u16le(bytes, 1) == 0xBEEF);
  write_u8(bytes, 3, 0x7F);
  CHECK(read_u16le(bytes, 2) == 0x7FBE);
}

TEST_CASE("capability requirements of the pin modes", "[protocol]") {
  CHECK(required_caps(PinMode::Input) == PinCaps::Dio);
  CHECK(required_caps(PinMode::Output) == PinCaps::Dio);
  CHECK(required_caps(PinMode::InputPullup) == PinCaps::Dio);
  CHECK(required_caps(PinMode::InputPulldown) == PinCaps::Dio);
  CHECK(required_caps(PinMode::AnalogIn) == PinCaps::Ain);
  CHECK(required_caps(PinMode::Pwm) == PinCaps::Pwm);
  CHECK(required_caps(PinMode::Dac) == PinCaps::Dac);
  CHECK(required_caps(static_cast<PinMode>(7)) == 0xFF);

  const PinCaps dio{PinCaps::Dio};
  const PinCaps dio_ain{PinCaps::Dio | PinCaps::Ain};
  CHECK(mode_supported(PinMode::Output, dio, false));
  CHECK_FALSE(mode_supported(PinMode::AnalogIn, dio, false));
  CHECK(mode_supported(PinMode::AnalogIn, dio_ain, false));
  CHECK_FALSE(mode_supported(PinMode::InputPulldown, dio, false));
  CHECK(mode_supported(PinMode::InputPulldown, dio, true));
  CHECK_FALSE(mode_supported(PinMode::Pwm, dio_ain, true));
  CHECK(PinCaps{PinCaps::Dio | PinCaps::Pwm}.has(PinCaps::Pwm));
  CHECK_FALSE(PinCaps{PinCaps::Pwm}.has(PinCaps::Dio | PinCaps::Pwm));
}

TEST_CASE("names", "[protocol]") {
  CHECK(to_string(Request::DioWrite) == "DIO_WRITE");
  CHECK(to_string(Request::GetPinCaps) == "GET_PIN_CAPS");
  CHECK(to_string(Status::Ok) == "OK");
  CHECK(to_string(Status::BadMode) == "BAD_MODE");
  CHECK(to_string(Status::QueueFull) == "QUEUE_FULL");
  CHECK(to_string(static_cast<Status>(200)) == "UNKNOWN");
  CHECK(to_string(PinMode::InputPulldown) == "INPUT_PULLDOWN");
  CHECK(to_string(PinMode::AnalogIn) == "ANALOG_IN");
  CHECK(board_name(BoardId::UnoR4Minima) == "UNO R4 Minima");
  CHECK(board_name(BoardId::PortentaH7) == "Portenta H7");
  CHECK(board_name(BoardId::RenesasGeneric) == "Renesas board");
  CHECK(board_name(static_cast<BoardId>(0x7777)) == "unknown");
  CHECK(to_string(BoardId::GigaR1) == board_name(BoardId::GigaR1));
  CHECK_FALSE(describe(Status::BadPin).empty());
}
