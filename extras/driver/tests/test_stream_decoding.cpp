// test_stream_decoding.cpp - Stream's byte-stream reassembly, exercised
// through Device::start_stream() + FakeTransport's bulk model: framing,
// straddling reassembly, resync on USBIO_STREAM_MAGIC after garbage,
// seq-gap accounting and the digital-bitmap record layout.
#include "TestRig.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdint>
#include <span>
#include <vector>

using namespace ArduinoDriver;
using ArduinoDriver::Testing::FakeBoard;
using ArduinoDriver::Testing::fast_options;
using ArduinoDriver::Testing::Rig;

using namespace std::chrono_literals;

namespace {

/// Any FakeBoard with streaming turned on, up to 8 channels.
FakeBoard streaming_board(FakeBoard board = FakeBoard::portenta_h7()) {
  board.flags |= USBIO_FLAG_STREAMING;
  board.stream_max_channels = 8;
  return board;
}

/// Reads exactly `want` samples off `stream` (across as many read() calls as
/// it takes), or fails the test if they do not show up within a generous
/// number of attempts -- the FakeTransport bulk model has no real timer, so
/// every attempt that finds nothing returns near-instantly.
std::vector<Sample> drain(Stream &stream, std::size_t want) {
  std::vector<Sample> out(want);
  std::size_t total = 0;
  // The fake has no real hardware latency, so a passing test finishes in a
  // handful of iterations; cap the worst case (a genuine decoding bug) at a
  // few seconds instead of stream.read()'s full timeout x hundreds of tries.
  for (int attempt = 0; attempt < 200 && total < want; ++attempt) {
    total += stream.read(std::span(out).subspan(total), 20ms);
  }
  REQUIRE(total == want);
  return out;
}

} // namespace

TEST_CASE("Stream decodes well-formed records delivered in one packet",
          "[stream][decoding]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(15, PinMode::AnalogIn); // A0, ADC-only pad
  rig.device.pin_mode(16, PinMode::AnalogIn); // A1
  const std::vector<std::uint8_t> pins{15, 16};

  StreamConfig config;
  config.pins = pins;
  Stream stream = rig.device.start_stream(config);

  rig.fake.set_stream_ramp(/*start=*/100, /*step=*/10, /*t0_us=*/1000,
                           /*dt_us=*/500);
  rig.fake.queue_stream_records(3);

  const std::vector<Sample> samples = drain(stream, 6);
  CHECK(samples[0].pin == 15);
  CHECK(samples[0].raw == 100);
  CHECK(samples[0].t_us == 1000);
  CHECK(samples[1].pin == 16);
  CHECK(samples[1].raw == 110);
  CHECK(samples[1].t_us == 1000);
  CHECK(samples[2].raw == 120);
  CHECK(samples[2].t_us == 1500);
  CHECK(samples[3].raw == 130);
  CHECK(samples[4].raw == 140);
  CHECK(samples[5].raw == 150);
  CHECK(samples[5].t_us == 2000);

  const StreamStats stats = stream.stats();
  CHECK(stats.records_received == 3);
  CHECK(stats.seq_gaps == 0);
  CHECK(stats.resyncs == 0);
  CHECK(stats.host_drops == 0);
}

TEST_CASE("Stream reassembles a record straddling two bulk_in() calls",
          "[stream][decoding]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(15, PinMode::AnalogIn);
  rig.device.pin_mode(16, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {15, 16};
  Stream stream = rig.device.start_stream(config);

  rig.fake.set_stream_ramp(1, 1, 0, 100);
  // Each record is 12 (header) + 2*2 (samples) = 16 bytes; two records =
  // 32 bytes. Force the first record to arrive as 7 then 9 bytes (splitting
  // it mid-header and mid-samples), the second whole -- queued atomically
  // with the record bytes so there is no race against the worker thread.
  rig.fake.queue_stream_records(2, /*seq_step=*/1, {7, 9, 16});

  const std::vector<Sample> samples = drain(stream, 4);
  CHECK(samples[0].raw == 1);
  CHECK(samples[1].raw == 2);
  CHECK(samples[2].raw == 3);
  CHECK(samples[3].raw == 4);

  const StreamStats stats = stream.stats();
  CHECK(stats.records_received == 2);
  CHECK(stats.resyncs == 0);
}

TEST_CASE("Stream tolerates a zero-length bulk_in() read", "[stream][decoding]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(15, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {15};
  Stream stream = rig.device.start_stream(config);

  // A zero-length packet (queued first: even an empty queue honours a
  // planned chunk size of 0) followed by two ordinary records.
  rig.fake.queue_bulk_chunk(0);
  rig.fake.set_stream_ramp(7, 1, 0, 1);
  rig.fake.queue_stream_records(2);

  const std::vector<Sample> samples = drain(stream, 2);
  CHECK(samples[0].raw == 7);
  CHECK(samples[1].raw == 8);
  CHECK(stream.stats().records_received == 2);
}

TEST_CASE("Stream resyncs on the magic after injected garbage",
          "[stream][decoding]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(15, PinMode::AnalogIn);
  rig.device.pin_mode(16, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {15, 16};
  Stream stream = rig.device.start_stream(config);

  rig.fake.set_stream_ramp(50, 5, 0, 10);
  rig.fake.queue_bulk_garbage(23); // deliberately not a multiple of the
                                   // 16-byte record length
  rig.fake.queue_stream_records(2);

  const std::vector<Sample> samples = drain(stream, 4);
  CHECK(samples[0].raw == 50);
  CHECK(samples[1].raw == 55);
  CHECK(samples[2].raw == 60);
  CHECK(samples[3].raw == 65);

  const StreamStats stats = stream.stats();
  CHECK(stats.records_received == 2);
  CHECK(stats.resyncs == 1);
  CHECK(stats.seq_gaps == 0);
}

TEST_CASE("Stream accounts for device-side seq gaps", "[stream][decoding]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(15, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {15};
  Stream stream = rig.device.start_stream(config);

  rig.fake.set_stream_ramp(0, 1, 0, 1);
  rig.fake.queue_stream_records(1, 1); // record0: seq 0
  rig.fake.queue_stream_records(1, 4); // record1: seq 1; device then "drops"
                                       // 3 records worth of seq (2,3,4)
  rig.fake.queue_stream_records(1, 1); // record2: seq 5 -> gap of 3

  const std::vector<Sample> samples = drain(stream, 3);
  CHECK(samples[0].raw == 0);
  CHECK(samples[1].raw == 1);
  CHECK(samples[2].raw == 2);

  const StreamStats stats = stream.stats();
  CHECK(stats.records_received == 3);
  CHECK(stats.seq_gaps == 3);
}

TEST_CASE("Stream record framing accounts for the padded digital bitmap",
          "[stream][decoding]") {
  // UNO R4 Minima: 20 pins -> dio_bitmap_len = 3 (odd), padded to 4 bytes,
  // so this actually exercises the padding, unlike the 26-pin Portenta.
  Rig rig(streaming_board(FakeBoard::uno_r4_minima()), fast_options());
  rig.device.pin_mode(14, PinMode::AnalogIn);
  rig.device.pin_mode(15, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {14, 15};
  config.flags = StreamFlags::Digital;
  Stream stream = rig.device.start_stream(config);

  rig.fake.set_stream_ramp(200, 1, 0, 1);
  rig.fake.queue_stream_records(2);

  // Each record is 12 + 2*2 + 4 (padded bitmap) = 20 bytes; if the padding
  // were computed wrong the second record's magic would land on the wrong
  // byte and either fail to decode or (astronomically unlikely) resync.
  const std::vector<Sample> samples = drain(stream, 4);
  CHECK(samples[0].raw == 200);
  CHECK(samples[1].raw == 201);
  CHECK(samples[2].raw == 202);
  CHECK(samples[3].raw == 203);

  const StreamStats stats = stream.stats();
  CHECK(stats.records_received == 2);
  CHECK(stats.resyncs == 0);
}
