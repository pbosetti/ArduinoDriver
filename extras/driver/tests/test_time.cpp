// test_time.cpp - GET_TIME: wire encoding/decoding, the 64-bit microsecond
// reconstruction (Protocol.h's reconstruct_micros64(), a pure function --
// most of this suite calls it directly, no FakeTransport involved) and
// Device::read_time()'s host-time anchor.
#include "TestRig.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <thread>

using namespace ArduinoDriver;
using ArduinoDriver::Testing::FakeBoard;
using ArduinoDriver::Testing::fast_options;
using ArduinoDriver::Testing::Rig;

using namespace std::chrono_literals;

// ---- reconstruct_micros64(): the pure arithmetic -----------------------

namespace {
// W = 2^32, the wrap period of both micros() (in microseconds) and of
// reconstruct_micros64()'s "epoch" (in microseconds too -- millis() wraps
// every 1000 such epochs).
constexpr std::uint64_t Wrap = std::uint64_t{1} << 32;
} // namespace

TEST_CASE("reconstruct_micros64: boot and simple mid-range values",
          "[time][micros64]") {
  static_assert(reconstruct_micros64(0, 0) == 0);
  static_assert(reconstruct_micros64(1000, 1000000) == 1000000);
  static_assert(reconstruct_micros64(1, 1000) == 1000);
  // A truncated millis() (fractional microseconds dropped) still resolves
  // to the exact micros() reading, since micros64 is built from micros
  // (exact) with only the epoch taken from millis (approximate).
  static_assert(reconstruct_micros64(1, 1999) == 1999);
}

TEST_CASE("reconstruct_micros64: micros() has wrapped once, self-consistent",
          "[time][micros64]") {
  // True T = 4295000000 us (~71.583 min): past one micros() wrap.
  // millis = floor(T/1000) = 4295000; micros = T mod 2^32 = 32704.
  static_assert(reconstruct_micros64(4295000, 32704) == 4295000000ull);
}

TEST_CASE("reconstruct_micros64: right at the wrap, micros() just past it",
          "[time][micros64][wrap]") {
  // True T = 2^32 + 50 (50 us after micros() wrapped).
  // micros = 50; millis = floor(T/1000) = 4294967 (its own 1 ms truncation
  // still puts it 346 us before T, on the *other* side of the epoch
  // boundary from where T actually is -- the case the wrap-correction
  // branch in reconstruct_micros64() exists for).
  constexpr std::uint64_t true_t = Wrap + 50;
  static_assert(reconstruct_micros64(4294967, 50) == true_t);
}

TEST_CASE("reconstruct_micros64: right at the wrap, micros() just before it",
          "[time][micros64][wrap]") {
  // True T = 2 * 2^32 - 40 (40 us before micros()'s second wrap), well
  // inside its own epoch from millis()'s point of view too: no correction
  // needed, included as the mirror-image sanity check of the case above.
  constexpr std::uint64_t true_t = 2 * Wrap - 40;
  static_assert(reconstruct_micros64(8589934, 4294967256u) == true_t);
}

TEST_CASE("reconstruct_micros64: millis/micros disagreement right at a wrap",
          "[time][micros64][wrap]") {
  // A pair that could not come from reading millis()/micros() at the same
  // instant (see usbio_protocol.h: GET_TIME serves both from one moment, so
  // this should not happen on the wire, but the function must still behave
  // predictably): millis says "just into epoch 1" while micros says "near
  // the end of epoch 0". The algorithm resolves the conflict in millis's
  // favour (its coarse*1000 estimate is far closer to the epoch-0
  // interpretation of micros than to the epoch-1 one), silently reinterpreting
  // micros as belonging to epoch 0 -- exactly the "no way to detect it from
  // these two values alone" the function's own documentation warns about.
  constexpr std::uint32_t millis_value = 4294968;   // 1 ms into epoch 1
  constexpr std::uint32_t micros_value = 4294967200; // near the end of epoch 0
  static_assert(reconstruct_micros64(millis_value, micros_value) ==
               micros_value); // epoch 0, not epoch 1: base + micros == micros
}

TEST_CASE("reconstruct_micros64: near the ~49.7-day millis() wrap",
          "[time][micros64]") {
  // millis() at its maximum uint32 value, with a self-consistent micros():
  // reconstruct_micros64() is documented valid right up to this boundary.
  constexpr std::uint32_t millis_value = 0xFFFFFFFFu;
  constexpr std::uint64_t coarse =
      static_cast<std::uint64_t>(millis_value) * 1000ull; // 4294967295000
  constexpr auto micros_value =
      static_cast<std::uint32_t>(coarse % Wrap); // self-consistent micros()
  static_assert(reconstruct_micros64(millis_value, micros_value) == coarse);
}

// ---- decode_time_reply() -------------------------------------------------

TEST_CASE("decode_time_reply reads usbio_time_reply_t field by field",
          "[time][protocol]") {
  std::array<std::byte, TimeReplyLen> bytes{};
  write_u8(bytes, TimeReplyOffset::Status, static_cast<std::uint8_t>(Status::Ok));
  write_u32le(bytes, TimeReplyOffset::Millis, 123456);
  write_u32le(bytes, TimeReplyOffset::Micros, 654321);
  const TimeReply reply = decode_time_reply(bytes);
  CHECK(reply.status == Status::Ok);
  CHECK(reply.millis == 123456);
  CHECK(reply.micros == 654321);

  SECTION("a short buffer is a ProtocolError") {
    CHECK_THROWS_AS(decode_time_reply(std::span(bytes).first(TimeReplyLen - 1)),
                    ProtocolError);
  }
}

// ---- Device::read_time() -------------------------------------------------

TEST_CASE("GET_TIME is a wIndex=0 IN request for a 12-byte reply",
          "[time][encoding]") {
  Rig rig;
  rig.fake.clear_log();
  rig.fake.set_millis(1000);
  rig.fake.set_micros(1000000);
  const DeviceTime t = rig.device.read_time();
  REQUIRE(rig.fake.log().size() == 1);
  CHECK(rig.fake.log()[0].request == USBIO_REQ_GET_TIME);
  CHECK(rig.fake.log()[0].is_in());
  CHECK(rig.fake.log()[0].index == 0);
  CHECK(rig.fake.log()[0].length == static_cast<std::uint16_t>(TimeReplyLen));
  CHECK(t.millis == 1000);
  CHECK(t.micros == 1000000);
  CHECK(t.micros64 == 1000000);
}

TEST_CASE("read_time() reconstructs micros64 from the device's reply",
          "[time]") {
  Rig rig;
  rig.fake.set_millis(4295000);
  rig.fake.set_micros(32704);
  const DeviceTime t = rig.device.read_time();
  CHECK(t.micros64 == reconstruct_micros64(4295000, 32704));
  CHECK(t.micros64 == 4295000000ull);
}

TEST_CASE("read_time() anchors to a host steady_clock interval bracketing "
          "the transfer",
          "[time]") {
  Rig rig;
  const auto before = std::chrono::steady_clock::now();
  const DeviceTime t = rig.device.read_time();
  const auto after = std::chrono::steady_clock::now();
  CHECK(t.host_time >= before);
  CHECK(t.host_time <= after);
  CHECK(t.round_trip >= 0us);
  // host_time is documented as the midpoint of the bracketing interval, so
  // it cannot be farther from either edge than the whole interval itself.
  CHECK(t.host_time - before <= after - before);
}

TEST_CASE("read_time() throws DeviceBusy while a Stream is running",
          "[time][stream]") {
  FakeBoard board = FakeBoard::portenta_h7();
  board.flags |= USBIO_FLAG_STREAMING;
  board.stream_max_channels = 4;
  Rig rig(board, fast_options());
  rig.device.pin_mode(19, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {19};
  Stream stream = rig.device.start_stream(config);
  CHECK_THROWS_AS(rig.device.read_time(), DeviceBusy);
  stream.stop();
  CHECK_NOTHROW(rig.device.read_time());
}
