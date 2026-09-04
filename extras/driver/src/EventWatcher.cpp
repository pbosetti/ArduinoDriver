// EventWatcher.cpp - see EventWatcher.h. Worker thread: calls
// Device::poll_events() every config.poll_interval and hands each decoded
// event to the callback, in the order poll_events() returned them.
#include "arduino_driver/EventWatcher.h"

#include "arduino_driver/Device.h"
#include "arduino_driver/Errors.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace ArduinoDriver {

// ---- Impl -------------------------------------------------------------------

struct EventWatcher::Impl {
  Impl(Device &dev, EventWatcherConfig cfg, Callback cb)
      : device(dev), config(std::move(cfg)), callback(std::move(cb)) {}

  void worker_main();

  Device &device;
  EventWatcherConfig config;
  Callback callback;

  std::thread worker;
  std::atomic<bool> stop_requested{false};
  std::atomic<bool> running{true};
  std::once_flag stop_once;

  std::mutex cv_mutex; // paired with cv, for prompt shutdown
  std::condition_variable cv;

  mutable std::mutex stats_mutex;
  EventWatcherStats stats;
};

void EventWatcher::Impl::worker_main() {
  while (!stop_requested.load(std::memory_order_acquire)) {
    try {
      std::uint8_t dropped = 0;
      const std::vector<PinEvent> events = device.poll_events(&dropped);
      if (dropped > 0) {
        std::lock_guard<std::mutex> lock(stats_mutex);
        stats.dropped += dropped;
      }
      for (const PinEvent &e : events) {
        if (callback) {
          callback(e);
        }
        std::lock_guard<std::mutex> lock(stats_mutex);
        ++stats.events_delivered;
      }
    } catch (const Error &) {
      // Transient (a control-transfer timeout, or DeviceBusy while a Stream
      // happens to be running on the same Device): retry after the usual
      // poll interval, same as Stream's internal status poll does.
    }
    std::unique_lock<std::mutex> lock(cv_mutex);
    cv.wait_for(lock, config.poll_interval, [this] {
      return stop_requested.load(std::memory_order_acquire);
    });
  }
  running.store(false, std::memory_order_release);
}

// ---- EventWatcher -----------------------------------------------------------

EventWatcher::EventWatcher(Device &device, EventWatcherConfig config,
                          Callback callback)
    : _impl(std::make_unique<Impl>(device, std::move(config),
                                   std::move(callback))) {
  if (_impl->config.pins.empty()) {
    throw std::invalid_argument(
        "EventWatcher: the pin list must not be empty");
  }
  std::vector<std::uint8_t> armed;
  try {
    for (const WatchedPin &w : _impl->config.pins) {
      device.configure_event(w.pin, w.edge, w.debounce);
      armed.push_back(w.pin);
    }
  } catch (...) {
    for (auto it = armed.rbegin(); it != armed.rend(); ++it) {
      try {
        device.configure_event(*it, EdgeMode::Off, std::chrono::milliseconds{0});
      } catch (...) {
        // best effort rollback: the original failure is what matters
      }
    }
    throw;
  }
  Impl *impl = _impl.get();
  impl->worker = std::thread([impl] { impl->worker_main(); });
}

EventWatcher::EventWatcher(EventWatcher &&) noexcept = default;

EventWatcher &EventWatcher::operator=(EventWatcher &&other) noexcept {
  if (this != &other) {
    stop();
    _impl = std::move(other._impl);
  }
  return *this;
}

EventWatcher::~EventWatcher() { stop(); }

void EventWatcher::stop() {
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
    for (const WatchedPin &w : impl->config.pins) {
      try {
        impl->device.configure_event(w.pin, EdgeMode::Off,
                                     std::chrono::milliseconds{0});
      } catch (...) {
        // best effort: this may run from the destructor and must not throw
      }
    }
  });
}

EventWatcherStats EventWatcher::stats() const {
  if (!_impl) {
    return {};
  }
  std::lock_guard<std::mutex> lock(_impl->stats_mutex);
  return _impl->stats;
}

bool EventWatcher::running() const noexcept {
  return _impl && _impl->running.load(std::memory_order_acquire);
}

const std::vector<WatchedPin> &EventWatcher::pins() const noexcept {
  static const std::vector<WatchedPin> empty;
  return _impl ? _impl->config.pins : empty;
}

} // namespace ArduinoDriver
