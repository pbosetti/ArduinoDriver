// test_event_watcher.cpp - EventWatcher: arming/disarming, callback
// delivery, clean RAII stop, move semantics, rollback on a failed
// construction, and -- the point of the Device threading change (see
// Device.h) -- ordinary Device calls succeeding from another thread while
// an EventWatcher's worker thread polls concurrently.
#include "TestRig.h"

#include "arduino_driver/EventWatcher.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

using namespace ArduinoDriver;
using ArduinoDriver::Testing::FakeBoard;
using ArduinoDriver::Testing::fast_options;
using ArduinoDriver::Testing::Rig;

using namespace std::chrono_literals;

namespace {

FakeBoard events_board() {
  FakeBoard board = FakeBoard::portenta_h7();
  board.flags |= USBIO_FLAG_EVENTS;
  board.event_max_pins = 4;
  return board;
}

void toggle(Rig &rig, std::uint8_t pin, int n) {
  bool level = rig.fake.digital_value(pin);
  for (int i = 0; i < n; ++i) {
    level = !level;
    rig.fake.set_digital(pin, level);
  }
}

/// Polls `pred` until it is true or `timeout` elapses; returns the last
/// value of `pred()`. The fake has no real hardware latency, so a passing
/// test settles in a handful of iterations.
bool wait_until(const std::function<bool()> &pred,
               std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  bool ok = pred();
  while (!ok && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(2ms);
    ok = pred();
  }
  return ok;
}

} // namespace

// ---- Arming / disarming ------------------------------------------------

TEST_CASE("constructing an EventWatcher arms every configured pin",
          "[events][watcher]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  rig.device.pin_mode(1, PinMode::Input);

  EventWatcherConfig config;
  config.pins = {{0, EdgeMode::Rising, 5ms}, {1, EdgeMode::Falling, 0ms}};
  EventWatcher watcher(rig.device, config, [](const PinEvent &) {});

  const std::vector<EventCount> counts = rig.device.event_counts();
  REQUIRE(counts.size() == 2);
  CHECK(counts[0].pin == 0);
  CHECK(counts[0].mode == EdgeMode::Rising);
  CHECK(counts[1].pin == 1);
  CHECK(counts[1].mode == EdgeMode::Falling);
  CHECK(watcher.pins().size() == 2);
  watcher.stop();
}

TEST_CASE("the destructor stops the worker and disarms the pins",
          "[events][watcher]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  {
    EventWatcherConfig config;
    config.pins = {{0, EdgeMode::Change, 0ms}};
    EventWatcher watcher(rig.device, config, [](const PinEvent &) {});
    CHECK(watcher.running());
    CHECK(rig.device.event_counts().size() == 1);
  }
  CHECK(rig.device.event_counts().empty()); // disarmed
  // The Device is fully usable afterwards (no lingering "busy" state).
  CHECK_NOTHROW(rig.device.digital_read(0));
}

TEST_CASE("stop() is idempotent and safe before the destructor runs",
          "[events][watcher]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  EventWatcherConfig config;
  config.pins = {{0, EdgeMode::Change, 0ms}};
  EventWatcher watcher(rig.device, config, [](const PinEvent &) {});
  watcher.stop();
  CHECK_FALSE(watcher.running());
  CHECK_NOTHROW(watcher.stop());
  CHECK(rig.device.event_counts().empty());
}

TEST_CASE("EventWatcher is move-only and moving transfers ownership",
          "[events][watcher]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  EventWatcherConfig config;
  config.pins = {{0, EdgeMode::Change, 0ms}};
  EventWatcher watcher(rig.device, config, [](const PinEvent &) {});
  EventWatcher moved = std::move(watcher);
  CHECK(moved.running());
  CHECK(moved.pins().size() == 1);
  moved.stop();
  CHECK(rig.device.event_counts().empty());
}

TEST_CASE("an empty pin list is rejected", "[events][watcher]") {
  Rig rig(events_board(), fast_options());
  EventWatcherConfig config;
  CHECK_THROWS_AS(
      EventWatcher(rig.device, config, [](const PinEvent &) {}),
      std::invalid_argument);
}

TEST_CASE("a failed construction rolls back the pins it already armed",
          "[events][watcher]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  // Pin 1 is left unconfigured: EVENT_CONFIG on it STALLs BAD_MODE ->
  // InvalidMode, after pin 0 has already been armed.
  EventWatcherConfig config;
  config.pins = {{0, EdgeMode::Change, 0ms}, {1, EdgeMode::Change, 0ms}};
  CHECK_THROWS_AS(EventWatcher(rig.device, config, [](const PinEvent &) {}),
                  InvalidMode);
  CHECK(rig.device.event_counts().empty()); // pin 0's arming was undone
}

// ---- Callback delivery -------------------------------------------------

TEST_CASE("EventWatcher delivers a callback per event, in order",
          "[events][watcher]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);

  std::mutex mutex;
  std::vector<PinEvent> received;
  EventWatcherConfig config;
  config.pins = {{0, EdgeMode::Change, 0ms}};
  config.poll_interval = 2ms;
  EventWatcher watcher(rig.device, config, [&](const PinEvent &event) {
    std::lock_guard<std::mutex> lock(mutex);
    received.push_back(event);
  });

  toggle(rig, 0, 4); // 4 edges

  REQUIRE(wait_until(
      [&] {
        std::lock_guard<std::mutex> lock(mutex);
        return received.size() >= 4;
      },
      2s));
  watcher.stop();

  std::lock_guard<std::mutex> lock(mutex);
  REQUIRE(received.size() == 4);
  for (std::size_t i = 0; i < received.size(); ++i) {
    CHECK(received[i].pin == 0);
  }
  CHECK(received[0].seq == 1);
  CHECK(received[3].seq == 4);
  CHECK(watcher.stats().events_delivered == 4);
}

TEST_CASE("EventWatcherStats.dropped accumulates ring drops the worker sees",
          "[events][watcher]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  // Arm and overflow the ring directly (single-threaded, deterministic --
  // no worker thread exists yet to race the queue fill) before an
  // EventWatcher ever gets a chance to drain it.
  rig.device.configure_event(0, EdgeMode::Change, 0ms);
  toggle(rig, 0, 40); // EventQueueDepth (32) + 8 dropped
  REQUIRE(rig.fake.event_queue_size() == 32);
  REQUIRE(rig.fake.pending_event_drops() == 8);

  // Re-arming the same, already-watched pin (what the constructor below
  // does) resets its counter but leaves the queue and drop count alone.
  EventWatcherConfig config;
  config.pins = {{0, EdgeMode::Change, 0ms}};
  EventWatcher watcher(rig.device, config, [](const PinEvent &) {});

  REQUIRE(wait_until([&] { return watcher.stats().events_delivered >= 32; },
                     5s));
  watcher.stop();
  CHECK(watcher.stats().events_delivered == 32);
  CHECK(watcher.stats().dropped == 8);
}

// ---- Concurrent Device use while a watcher runs (Device.h's threading
// contract) --------------------------------------------------------------

TEST_CASE("ordinary Device calls succeed from another thread while an "
          "EventWatcher polls concurrently",
          "[events][watcher][threading]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);  // watched
  rig.device.pin_mode(2, PinMode::Output); // driven by the "user" thread
  rig.device.pin_mode(15, PinMode::AnalogIn); // A0, ADC-only pad

  std::atomic<std::uint64_t> delivered{0};
  EventWatcherConfig config;
  config.pins = {{0, EdgeMode::Change, 0ms}};
  config.poll_interval = 1ms;
  EventWatcher watcher(rig.device, config,
                       [&](const PinEvent &) { ++delivered; });

  // A dedicated thread drives ordinary I/O on the same Device the whole
  // time the watcher's worker thread is polling EVENT_POP -- exactly the
  // scenario the internal control-transfer mutex exists for. Catch2's
  // assertion macros are not themselves thread-safe (see its docs), so
  // failures here are recorded through a plain atomic and asserted on the
  // main thread below, after the driver has been joined.
  std::atomic<bool> stop_driver{false};
  std::atomic<bool> driver_ok{true};
  std::atomic<std::uint64_t> writes{0};
  std::thread driver([&] {
    bool level = false;
    while (!stop_driver.load(std::memory_order_acquire)) {
      try {
        level = !level;
        rig.device.digital_write(2, level);
        rig.device.digital_read(2);
        rig.device.analog_read(15);
        ++writes;
      } catch (...) {
        driver_ok.store(false, std::memory_order_relaxed);
      }
    }
  });

  // Meanwhile, a second thread injects a handful of edges on the watched
  // pin -- fewer than EventQueueDepth, so no interleaving with the
  // watcher's own poll cadence can ever drop one: this assertion stays
  // deterministic (not timing-dependent) under any scheduling, including a
  // ThreadSanitizer build's heavier instrumentation.
  std::thread injector([&] {
    for (int i = 0; i < 10; ++i) {
      rig.fake.set_digital(0, true);
      rig.fake.set_digital(0, false);
    }
  });

  injector.join();
  const bool delivered_ok = wait_until([&] { return delivered.load() >= 20; }, 5s);
  stop_driver.store(true, std::memory_order_release);
  driver.join();
  watcher.stop();

  REQUIRE(delivered_ok);
  CHECK(driver_ok.load());
  CHECK(writes.load() > 0);
  CHECK(delivered.load() == 20); // 10 rising + 10 falling, none dropped
}
