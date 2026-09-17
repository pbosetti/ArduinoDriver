// Stream.cpp - see Stream.h. Worker thread: pulls bytes off
// Transport::bulk_in(), reassembles usbio_stream_header_t + samples records
// out of the continuous byte stream (resyncing on USBIO_STREAM_MAGIC after
// any loss), and hands decoded Sample values to read() / on_records().
#include "arduino_driver/Stream.h"

#include "arduino_driver/Device.h"
#include "arduino_driver/Errors.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>

namespace ArduinoDriver {

namespace {
/// One bulk_in() call reads into a chunk this large; a few USBIO_STREAM_EP_SIZE
/// packets, generous enough that a lively stream drains in one round trip.
constexpr std::size_t ChunkSize = 4096;
/// Bulk read timeout: short enough that the worker notices stop_requested
/// promptly, long enough to not spin when the stream is idle.
constexpr std::chrono::milliseconds IoTimeout{200};
/// How often the worker refreshes device_overruns via GET_STREAM_STATUS.
constexpr std::chrono::milliseconds StatusPollInterval{200};
} // namespace

// ---- Device trampolines ------------------------------------------------------

StreamStatus Stream::poll_status(Device &device) {
  return device.poll_stream_status();
}

void Stream::end(Device &device) noexcept { device.end_stream(); }

// ---- Impl ---------------------------------------------------------------

struct Stream::Impl {
  Impl(Device &dev, StreamConfig cfg, std::uint32_t start_us)
      : device(dev), transport(dev.transport()), config(std::move(cfg)),
        n_pins(dev.pin_count()), digital(StreamFlags{config.flags}.digital()),
        start_t_us(start_us) {}

  void worker_main();
  void deliver(std::vector<Sample> samples);

  Device &device;
  Transport &transport;
  StreamConfig config;
  std::size_t n_pins;
  bool digital;
  std::uint32_t start_t_us; // device micros() just before STREAM_START

  std::thread worker;
  std::atomic<bool> stop_requested{false};
  std::atomic<bool> running{true};
  std::once_flag stop_once;

  mutable std::mutex mutex; // guards ready, stats, callback, error below
  std::condition_variable cv;
  std::deque<std::vector<Sample>> ready;
  StreamStats stats;
  RecordCallback callback;
  std::string error; // why the worker stopped on its own; empty otherwise
};

void Stream::Impl::deliver(std::vector<Sample> samples) {
  RecordCallback cb;
  {
    std::lock_guard<std::mutex> lock(mutex);
    cb = callback;
  }
  if (cb) {
    cb(samples);
  }
  {
    std::lock_guard<std::mutex> lock(mutex);
    while (ready.size() >= config.queue_capacity) {
      ready.pop_front();
      ++stats.host_drops;
    }
    ready.push_back(std::move(samples));
  }
  cv.notify_one();
}

void Stream::Impl::worker_main() {
  const auto n_samples_expected = static_cast<std::uint16_t>(config.pins.size());

  std::vector<std::byte> buffer;
  std::array<std::byte, ChunkSize> chunk{};
  std::optional<std::uint32_t> last_seq;
  bool hunting = false;
  // Set once GET_STREAM_STATUS reports that the device is no longer sampling
  // although this Stream never asked it to stop.
  bool device_stopped = false;
  auto next_status_poll = std::chrono::steady_clock::now();

  while (!stop_requested.load(std::memory_order_acquire)) {
    std::size_t n = 0;
    try {
      n = transport.bulk_in(chunk, IoTimeout);
    } catch (const Error &e) {
      // fatal transport failure (e.g. device unplugged): record why, stop
      std::lock_guard<std::mutex> lock(mutex);
      error = e.what();
      break;
    }
    if (n > 0) {
      buffer.insert(buffer.end(), chunk.begin(),
                    chunk.begin() + static_cast<std::ptrdiff_t>(n));
    } else if (device_stopped) {
      // The records the device produced before stopping have all been read:
      // nothing more will ever arrive, so end the stream instead of waiting.
      std::lock_guard<std::mutex> lock(mutex);
      error = "the device stopped streaming (STREAM_STOP, RESET or PIN_MODE "
              "on a streamed pin from another session, an overrun with "
              "StopOnOverrun, or the device giving up on an endpoint the "
              "host did not drain)";
      break;
    }

    std::size_t pos = 0;
    while (buffer.size() - pos >= StreamHeaderLen) {
      const std::span<const std::byte> head(buffer.data() + pos,
                                            buffer.size() - pos);
      const StreamHeader header = decode_stream_header(head);
      if (header.magic != StreamMagic || header.n_samples != n_samples_expected) {
        hunting = true;
        ++pos; // not a real record header: scan forward one byte and retry
        continue;
      }
      const std::size_t record_len =
          stream_record_len(header.n_samples, digital, n_pins);
      if (head.size() < record_len) {
        break; // the rest is still in transit: wait for more bytes
      }
      if (hunting) {
        std::lock_guard<std::mutex> lock(mutex);
        ++stats.resyncs;
        hunting = false;
      }
      // STREAM_STOP leaves whatever the device had already handed to its
      // bulk endpoint there, so the first bytes a new session reads can be
      // records of an earlier one. Every record of this stream was sampled
      // after start_t_us (read just before STREAM_START): anything older,
      // modulo micros()' 2^32 wrap, is stale.
      const std::uint32_t age = start_t_us - header.t_us;
      if (age != 0 && age < 0x80000000u) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          ++stats.stale_records;
        }
        pos += record_len;
        continue;
      }

      std::vector<Sample> samples(header.n_samples);
      for (std::uint16_t i = 0; i < header.n_samples; ++i) {
        const std::uint16_t raw =
            read_u16le(head, StreamHeaderLen + 2 * static_cast<std::size_t>(i));
        samples[i].pin = config.pins[i];
        samples[i].raw = raw;
        samples[i].volts = device.to_volts(raw);
        samples[i].t_us = header.t_us;
      }
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (last_seq) {
          const auto expected = static_cast<std::uint32_t>(*last_seq + 1);
          const std::uint32_t gap = header.seq - expected;
          // A backwards jump (gap >= 2^31 once wrapped) means the device
          // restarted its seq counter, not that ~4 billion records were lost.
          if (gap != 0 && gap < 0x80000000u) {
            stats.seq_gaps += gap;
          }
        }
        last_seq = header.seq;
        ++stats.records_received;
      }
      pos += record_len;
      deliver(std::move(samples));
    }
    if (pos > 0) {
      buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(pos));
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= next_status_poll) {
      try {
        const StreamStatus st = Stream::poll_status(device);
        std::lock_guard<std::mutex> lock(mutex);
        stats.device_overruns = st.overruns;
        // stop() sets stop_requested before STREAM_STOP goes out, so a
        // device that is not running here was stopped by someone else. Keep
        // reading until its endpoint is empty (see above), then end.
        if (!st.running) {
          device_stopped = true;
        }
      } catch (const Error &) {
        // transient (e.g. a control-transfer timeout): retry next round
      }
      next_status_poll = now + StatusPollInterval;
    }
  }
  running.store(false, std::memory_order_release);
  cv.notify_all(); // wake a blocked read(): nothing more will ever arrive
}

// ---- Stream ---------------------------------------------------------------

Stream::Stream(Device &device, StreamConfig config, std::uint32_t start_t_us)
    : _impl(std::make_unique<Impl>(device, std::move(config), start_t_us)) {
  Impl *impl = _impl.get();
  _impl->worker = std::thread([impl] { impl->worker_main(); });
}

Stream::Stream(Stream &&) noexcept = default;

Stream &Stream::operator=(Stream &&other) noexcept {
  if (this != &other) {
    stop();
    _impl = std::move(other._impl);
  }
  return *this;
}

Stream::~Stream() { stop(); }

void Stream::stop() {
  if (!_impl) {
    return;
  }
  _impl->stop_requested.store(true, std::memory_order_release);
  _impl->cv.notify_all();
  Impl *impl = _impl.get();
  std::call_once(impl->stop_once, [impl] {
    if (impl->worker.joinable()) {
      impl->worker.join();
    }
    Stream::end(impl->device);
  });
}

std::size_t Stream::read(std::span<Sample> out,
                         std::chrono::milliseconds timeout) {
  if (!_impl || out.empty()) {
    return 0;
  }
  std::unique_lock<std::mutex> lock(_impl->mutex);
  if (_impl->ready.empty()) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    _impl->cv.wait_until(lock, deadline, [this] {
      return !_impl->ready.empty() ||
            _impl->stop_requested.load(std::memory_order_acquire);
    });
  }
  std::size_t copied = 0;
  while (!_impl->ready.empty()) {
    const std::vector<Sample> &front = _impl->ready.front();
    if (front.size() > out.size() - copied) {
      break; // does not fit whole; leave it queued for the next call
    }
    std::copy(front.begin(), front.end(),
              out.begin() + static_cast<std::ptrdiff_t>(copied));
    copied += front.size();
    _impl->ready.pop_front();
  }
  return copied;
}

void Stream::on_records(RecordCallback callback) {
  if (!_impl) {
    return;
  }
  std::lock_guard<std::mutex> lock(_impl->mutex);
  _impl->callback = std::move(callback);
}

StreamStats Stream::stats() const {
  if (!_impl) {
    return {};
  }
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return _impl->stats;
}

std::string Stream::error() const {
  if (!_impl) {
    return {};
  }
  std::lock_guard<std::mutex> lock(_impl->mutex);
  return _impl->error;
}

bool Stream::running() const noexcept {
  return _impl && _impl->running.load(std::memory_order_acquire);
}

const std::vector<std::uint8_t> &Stream::pins() const noexcept {
  static const std::vector<std::uint8_t> empty;
  return _impl ? _impl->config.pins : empty;
}

} // namespace ArduinoDriver
