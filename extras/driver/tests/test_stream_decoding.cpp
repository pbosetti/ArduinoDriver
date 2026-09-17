// test_stream_decoding.cpp - Stream's byte-stream reassembly, exercised
// through Device::start_stream() + FakeTransport's bulk model: framing,
// straddling reassembly, resync on USBIO_STREAM_MAGIC after garbage,
// seq-gap accounting and the digital-bitmap record layout.
#include "TestRig.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdint>
#include <span>
#include <string>
#include <thread>
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

/// Spins until `stream`'s host_drops counter reaches `want`, or fails the
/// test after a generous number of attempts. Used to make a queue_capacity
/// overflow test deterministic: Impl::deliver() bumps host_drops and pushes
/// the surviving record onto `ready` under the same lock, so observing the
/// wanted host_drops count here guarantees the queue already holds exactly
/// the records that are meant to survive -- unlike records_received, which
/// Impl::worker_main() increments *before* calling deliver() for that
/// record, so waiting on it can race ahead of the corresponding drop.
void wait_for_host_drops(Stream &stream, std::uint64_t want) {
  for (int attempt = 0;
       attempt < 500 && stream.stats().host_drops < want; ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  REQUIRE(stream.stats().host_drops == want);
}

/// One hand-built single-channel record (header + one sample), for edge cases
/// the ramp generator cannot produce (chosen seq and t_us values).
std::vector<std::byte> single_channel_record(std::uint32_t seq,
                                             std::uint32_t t_us,
                                             std::uint16_t raw) {
  std::vector<std::byte> bytes(stream_record_len(1, false, 0));
  StreamHeader header;
  header.magic = StreamMagic;
  header.n_samples = 1;
  header.seq = seq;
  header.t_us = t_us;
  encode_stream_header(bytes, header);
  write_u16le(bytes, StreamHeaderLen, raw);
  return bytes;
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

TEST_CASE("Stream drops the oldest records beyond queue_capacity",
          "[stream][decoding]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(15, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {15};
  config.queue_capacity = 2;
  Stream stream = rig.device.start_stream(config);

  rig.fake.set_stream_ramp(/*start=*/100, /*step=*/10, /*t0_us=*/1000,
                           /*dt_us=*/500);
  rig.fake.queue_stream_records(5);

  // Let the worker decode and drop every excess record before reading: with
  // capacity 2 and 5 records, exactly 3 drops must have happened (records
  // 0/1/2) by the time read() runs.
  wait_for_host_drops(stream, 3);

  const std::vector<Sample> samples = drain(stream, 2);
  CHECK(samples[0].raw == 130);   // record 3 (0-based): 100 + 10*3
  CHECK(samples[0].t_us == 2500); // 1000 + 500*3
  CHECK(samples[1].raw == 140);   // record 4: 100 + 10*4
  CHECK(samples[1].t_us == 3000); // 1000 + 500*4

  const StreamStats stats2 = stream.stats();
  CHECK(stats2.records_received == 5);
  CHECK(stats2.host_drops == 3);
}

TEST_CASE("Stream discards records sampled before its STREAM_START",
          "[stream][decoding]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(15, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {15};

  SECTION("leftovers of an earlier session") {
    rig.fake.set_micros(10000); // start_stream() reads this before START
    Stream stream = rig.device.start_stream(config);
    for (const auto &rec : {single_channel_record(40, 8000, 1),
                            single_channel_record(41, 9000, 2),
                            single_channel_record(0, 10500, 3),
                            single_channel_record(1, 11000, 4)}) {
      rig.fake.queue_bulk_bytes(rec);
    }
    const std::vector<Sample> samples = drain(stream, 2);
    CHECK(samples[0].raw == 3);
    CHECK(samples[0].t_us == 10500);
    CHECK(samples[1].raw == 4);
    const StreamStats stats = stream.stats();
    CHECK(stats.stale_records == 2);
    CHECK(stats.records_received == 2);
    CHECK(stats.seq_gaps == 0);
  }
  SECTION("across micros()' 2^32 wrap") {
    rig.fake.set_micros(0xFFFFFF00u);
    Stream stream = rig.device.start_stream(config);
    rig.fake.queue_bulk_bytes(single_channel_record(7, 0xFFFFFE00u, 1)); // stale
    rig.fake.queue_bulk_bytes(single_channel_record(0, 0xFFFFFF80u, 2));
    rig.fake.queue_bulk_bytes(single_channel_record(1, 0x00000010u, 3)); // wrapped
    const std::vector<Sample> samples = drain(stream, 2);
    CHECK(samples[0].raw == 2);
    CHECK(samples[1].raw == 3);
    CHECK(stream.stats().stale_records == 1);
  }
}

TEST_CASE("a backwards seq jump is a device restart, not a gap",
          "[stream][decoding]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(15, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {15};
  Stream stream = rig.device.start_stream(config); // start_t_us == 0
  for (const auto &rec : {single_channel_record(5, 100, 1),
                          single_channel_record(6, 200, 2),
                          single_channel_record(0, 300, 3),
                          single_channel_record(1, 400, 4)}) {
    rig.fake.queue_bulk_bytes(rec);
  }
  drain(stream, 4);
  const StreamStats stats = stream.stats();
  CHECK(stats.records_received == 4);
  CHECK(stats.seq_gaps == 0);
}

TEST_CASE("Stream stops and reports why when bulk_in() fails",
          "[stream][decoding]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(15, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {15};
  Stream stream = rig.device.start_stream(config);

  rig.fake.set_stream_ramp(/*start=*/100, /*step=*/10, /*t0_us=*/1000,
                           /*dt_us=*/500);
  rig.fake.queue_stream_records(2);
  drain(stream, 2);
  CHECK(stream.running());
  CHECK(stream.error().empty());

  rig.fake.fail_bulk_in(LibusbError::Pipe);
  for (int attempt = 0; attempt < 500 && stream.running(); ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  REQUIRE_FALSE(stream.running());
  CHECK(stream.error().find("LIBUSB_ERROR_PIPE") != std::string::npos);

  // stop() after the worker died on its own is still safe and keeps the
  // reason available.
  stream.stop();
  CHECK(stream.error().find("LIBUSB_ERROR_PIPE") != std::string::npos);
}

TEST_CASE("Stream ends, keeping its records, when the device stops streaming",
          "[stream][decoding]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(15, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {15};
  Stream stream = rig.device.start_stream(config);

  rig.fake.set_stream_ramp(/*start=*/100, /*step=*/10, /*t0_us=*/1000,
                           /*dt_us=*/500);
  rig.fake.queue_stream_records(2);
  drain(stream, 2);

  // Records still in transit when another session stops the device stream.
  rig.fake.queue_stream_records(3);
  rig.fake.stop_stream_on_device();
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (stream.running() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
  }
  REQUIRE_FALSE(stream.running());
  CHECK(stream.error().find("device stopped streaming") != std::string::npos);
  const std::vector<Sample> tail = drain(stream, 3);
  CHECK(tail.front().raw == 120);
  CHECK(stream.stats().records_received == 5);

  stream.stop();
  CHECK(stream.error().find("device stopped streaming") != std::string::npos);
}

TEST_CASE("an idle stream on a running device keeps running",
          "[stream][decoding]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(15, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {15};
  Stream stream = rig.device.start_stream(config);

  // No records for longer than two GET_STREAM_STATUS polls.
  std::this_thread::sleep_for(450ms);
  CHECK(stream.running());
  CHECK(stream.error().empty());
  CHECK(rig.fake.count(Request::StreamStatus) >= 2);
}

TEST_CASE("a regular stop() leaves Stream::error() empty", "[stream][decoding]") {
  Rig rig(streaming_board(), fast_options());
  rig.device.pin_mode(15, PinMode::AnalogIn);
  StreamConfig config;
  config.pins = {15};
  Stream stream = rig.device.start_stream(config);
  stream.stop();
  CHECK_FALSE(stream.running());
  CHECK(stream.error().empty());
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
