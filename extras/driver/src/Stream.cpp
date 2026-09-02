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
/// Bound on queued-but-unread records, so a consumer that stops calling
/// read() does not grow the queue without limit; overflow counts as
/// host_drops.
constexpr std::size_t MaxReadyRecords = 1024;
} // namespace

// ---- Device trampolines ------------------------------------------------------

StreamStatus Stream::poll_status(Device &device) {
  return device.poll_stream_status();
}

void Stream::end(Device &device) noexcept { device.end_stream(); }

// ---- Impl ---------------------------------------------------------------

struct Stream::Impl {
  Impl(Device &dev, StreamConfig cfg)
      : device(dev), transport(dev.transport()), config(std::move(cfg)),
        n_pins(dev.pin_count()), digital(StreamFlags{config.flags}.digital()) {
  }

  void worker_main();
  void deliver(std::vector<Sample> samples);

  Device &device;
  Transport &transport;
  StreamConfig config;
  std::size_t n_pins;
  bool digital;

  std::thread worker;
  std::atomic<bool> stop_requested{false};
  std::atomic<bool> running{true};
  std::once_flag stop_once;

  mutable std::mutex mutex; // guards ready, stats, callback below
  std::condition_variable cv;
  std::deque<std::vector<Sample>> ready;
  StreamStats stats;
  RecordCallback callback;
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
    if (ready.size() >= MaxReadyRecords) {
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
  auto next_status_poll = std::chrono::steady_clock::now();

  while (!stop_requested.load(std::memory_order_acquire)) {
    std::size_t n = 0;
    try {
      n = transport.bulk_in(chunk, IoTimeout);
    } catch (const Error &) {
      break; // fatal transport failure (e.g. device unplugged): stop
    }
    if (n > 0) {
      buffer.insert(buffer.end(), chunk.begin(),
                    chunk.begin() + static_cast<std::ptrdiff_t>(n));
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
          if (header.seq != expected) {
            stats.seq_gaps += static_cast<std::uint64_t>(header.seq - expected);
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

Stream::Stream(Device &device, StreamConfig config)
    : _impl(std::make_unique<Impl>(device, std::move(config))) {
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

bool Stream::running() const noexcept {
  return _impl && _impl->running.load(std::memory_order_acquire);
}

const std::vector<std::uint8_t> &Stream::pins() const noexcept {
  static const std::vector<std::uint8_t> empty;
  return _impl ? _impl->config.pins : empty;
}

} // namespace ArduinoDriver
