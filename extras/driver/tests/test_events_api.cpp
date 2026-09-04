// test_events_api.cpp - Device's pin-event API against the fake firmware:
// EVENT_CONFIG encoding/validation, EVENT_POP ordering/pending/dropped,
// EVENT_COUNTS staying exact across drops, wait_event()'s timeout, the
// EdgeMode::Off / PIN_MODE / RESET unwatch semantics, and the DeviceBusy
// gate while a Stream runs.
#include "TestRig.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

using namespace ArduinoDriver;
using ArduinoDriver::Testing::FakeBoard;
using ArduinoDriver::Testing::fast_options;
using ArduinoDriver::Testing::Rig;

using namespace std::chrono_literals;

namespace {

/// Portenta H7 with pin events turned on, up to 4 watched pins at once.
FakeBoard events_board() {
  FakeBoard board = FakeBoard::portenta_h7();
  board.flags |= USBIO_FLAG_EVENTS;
  board.event_max_pins = 4;
  return board;
}

/// Toggles `pin` `n` times starting from its current level (so n edges are
/// generated, alternating direction), with no delay -- callers wanting
/// distinguishable rising/falling edges use n = 2 with EdgeMode::Change, or
/// call set_digital() directly.
void toggle(Rig &rig, std::uint8_t pin, int n) {
  bool level = rig.fake.digital_value(pin);
  for (int i = 0; i < n; ++i) {
    level = !level;
    rig.fake.set_digital(pin, level);
  }
}

} // namespace

// ---- EVENT_CONFIG: encoding and local validation ---------------------------

TEST_CASE("EVENT_CONFIG carries (debounce << 8) | edge in wValue, pin in wIndex",
          "[events][encoding]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  rig.fake.clear_log();
  rig.device.configure_event(0, EdgeMode::Rising, 50ms);
  REQUIRE(rig.fake.log().size() == 1);
  CHECK(rig.fake.log()[0].request == USBIO_REQ_EVENT_CONFIG);
  CHECK(rig.fake.log()[0].is_out());
  CHECK(rig.fake.log()[0].index == 0);
  CHECK(rig.fake.log()[0].value == ((50u << 8) | 1u));
}

TEST_CASE("configure_event throws NotSupported when info().events() is false",
          "[events][validation]") {
  Rig rig(FakeBoard::portenta_h7(), fast_options()); // no USBIO_FLAG_EVENTS
  CHECK_FALSE(rig.device.info().events());
  rig.device.pin_mode(0, PinMode::Input);
  CHECK_THROWS_AS(rig.device.configure_event(0, EdgeMode::Rising), NotSupported);
}

TEST_CASE("configure_event validates locally before any USB traffic",
          "[events][validation]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  rig.fake.clear_log();

  SECTION("pin out of range -> InvalidPin") {
    CHECK_THROWS_AS(rig.device.configure_event(200, EdgeMode::Rising), InvalidPin);
    CHECK(rig.fake.log().empty());
  }
  SECTION("unknown edge mode -> InvalidValue") {
    CHECK_THROWS_AS(
        rig.device.configure_event(0, static_cast<EdgeMode>(4)), InvalidValue);
    CHECK(rig.fake.log().empty());
  }
  SECTION("debounce above MaxDebounceMs -> InvalidValue") {
    CHECK_THROWS_AS(
        rig.device.configure_event(0, EdgeMode::Rising, 256ms), InvalidValue);
    CHECK(rig.fake.log().empty());
  }
  SECTION("debounce at MaxDebounceMs is fine") {
    CHECK_NOTHROW(rig.device.configure_event(
        0, EdgeMode::Rising, std::chrono::milliseconds{MaxDebounceMs}));
    CHECK(rig.fake.log().size() == 1);
  }
}

TEST_CASE("configure_event surfaces device-side BAD_MODE as InvalidMode",
          "[events][validation]") {
  Rig rig(events_board(), fast_options());
  SECTION("unconfigured pin") {
    CHECK_THROWS_AS(rig.device.configure_event(0, EdgeMode::Rising), InvalidMode);
  }
  SECTION("OUTPUT pin") {
    rig.device.pin_mode(0, PinMode::Output);
    CHECK_THROWS_AS(rig.device.configure_event(0, EdgeMode::Rising), InvalidMode);
  }
  SECTION("ANALOG_IN pin") {
    rig.device.pin_mode(19, PinMode::AnalogIn);
    CHECK_THROWS_AS(rig.device.configure_event(19, EdgeMode::Rising), InvalidMode);
  }
  SECTION("analog-only pad (no CAP_DIO at all)") {
    // A0 on the Portenta: PinCaps::Ain only, cannot even reach INPUT.
    CHECK_THROWS_AS(rig.device.pin_mode(15, PinMode::Input), NotSupported);
    CHECK_THROWS_AS(rig.device.configure_event(15, EdgeMode::Rising), InvalidMode);
  }
}

TEST_CASE("configure_event enforces event_max_pins only when arming a new pin",
          "[events][validation]") {
  Rig rig(events_board(), fast_options()); // event_max_pins == 4
  for (std::uint8_t pin = 0; pin < 4; ++pin) {
    rig.device.pin_mode(pin, PinMode::Input);
    CHECK_NOTHROW(rig.device.configure_event(pin, EdgeMode::Change));
  }
  rig.device.pin_mode(4, PinMode::Input);
  CHECK_THROWS_AS(rig.device.configure_event(4, EdgeMode::Change), InvalidValue);

  // Re-arming one of the 4 already-watched pins is not "a new pin": no
  // capacity error, even though 4 pins are already watched.
  CHECK_NOTHROW(rig.device.configure_event(2, EdgeMode::Falling, 5ms));
  // Unwatching one first frees a slot for a genuinely new pin.
  rig.device.configure_event(0, EdgeMode::Off);
  CHECK_NOTHROW(rig.device.configure_event(4, EdgeMode::Change));
}

// ---- Unwatch semantics: EdgeMode::Off / PIN_MODE / RESET -------------------

TEST_CASE("EdgeMode::Off unwatches, clears the counter and discards queued "
          "events",
          "[events]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  rig.device.configure_event(0, EdgeMode::Change);
  toggle(rig, 0, 2); // two edges, left unpopped in the queue

  rig.device.configure_event(0, EdgeMode::Off);
  CHECK(rig.device.event_counts().empty());
  CHECK(rig.device.poll_events().empty()); // its queued events are gone too

  // Unwatching a pin that was never watched is a harmless no-op.
  CHECK_NOTHROW(rig.device.configure_event(1, EdgeMode::Off));
}

TEST_CASE("re-arming a watched pin replaces its mode/debounce and resets its "
          "counter",
          "[events]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  rig.device.configure_event(0, EdgeMode::Rising);
  toggle(rig, 0, 2); // one rising, one falling -> counter == 1
  REQUIRE(rig.device.event_counts().at(0).count == 1);

  rig.device.configure_event(0, EdgeMode::Falling, 10ms);
  const std::vector<EventCount> counts = rig.device.event_counts();
  REQUIRE(counts.size() == 1);
  CHECK(counts[0].pin == 0);
  CHECK(counts[0].mode == EdgeMode::Falling);
  CHECK(counts[0].count == 0);
}

TEST_CASE("a PIN_MODE out of INPUT* unwatches the pin but keeps its queued "
          "events readable",
          "[events]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  rig.device.configure_event(0, EdgeMode::Change);
  toggle(rig, 0, 1); // one queued, unpopped event

  rig.device.pin_mode(0, PinMode::Output); // takes it out of INPUT*
  CHECK(rig.device.event_counts().empty()); // unwatched
  const std::vector<PinEvent> events = rig.device.poll_events();
  REQUIRE(events.size() == 1); // but still readable
  CHECK(events[0].pin == 0);

  SECTION("moving between two INPUT* modes keeps the watch") {
    rig.device.pin_mode(1, PinMode::Input);
    rig.device.configure_event(1, EdgeMode::Change);
    rig.device.pin_mode(1, PinMode::InputPullup); // still an INPUT* mode
    CHECK(rig.device.event_counts().size() == 1);
  }
}

TEST_CASE("RESET unwatches every pin and clears the event queue", "[events]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  rig.device.pin_mode(1, PinMode::Input);
  rig.device.configure_event(0, EdgeMode::Change);
  rig.device.configure_event(1, EdgeMode::Change);
  toggle(rig, 0, 2);
  toggle(rig, 1, 2);

  rig.device.reset(); // also puts every DIO pin back in INPUT
  CHECK(rig.device.event_counts().empty());
  CHECK(rig.device.poll_events().empty());
}

// ---- EVENT_COUNTS: arm order -------------------------------------------

TEST_CASE("event_counts() lists pins in the order they were armed",
          "[events]") {
  Rig rig(events_board(), fast_options());
  for (const std::uint8_t pin : {std::uint8_t{2}, std::uint8_t{0}, std::uint8_t{1}}) {
    rig.device.pin_mode(pin, PinMode::Input);
    rig.device.configure_event(pin, EdgeMode::Change);
  }
  const std::vector<EventCount> counts = rig.device.event_counts();
  REQUIRE(counts.size() == 3);
  CHECK(counts[0].pin == 2);
  CHECK(counts[1].pin == 0);
  CHECK(counts[2].pin == 1);
}

// ---- EVENT_POP: ordering, pending, dropped ---------------------------------

TEST_CASE("poll_events drains the full queue across multiple EVENT_POP calls, "
          "oldest first",
          "[events][pop]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  rig.device.configure_event(0, EdgeMode::Change); // debounce 0: every edge counts
  rig.fake.clear_log();
  toggle(rig, 0, 10); // more than MaxEventsPerPop (7): needs 2 EVENT_POP calls

  const std::vector<PinEvent> events = rig.device.poll_events();
  REQUIRE(events.size() == 10);
  for (std::size_t i = 0; i < events.size(); ++i) {
    CHECK(events[i].pin == 0);
    CHECK(events[i].seq ==
         static_cast<std::uint16_t>(i + 1)); // oldest first, acceptance order
  }
  CHECK(rig.fake.count(Request::EventPop) == 2); // ceil(10 / 7)
}

TEST_CASE("poll_events reports dropped events while event_counts stays exact",
          "[events][pop][drop]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  rig.device.configure_event(0, EdgeMode::Change);
  toggle(rig, 0, 40); // EventQueueDepth (32) + 8: 8 edges dropped from the
                      // queue, but every one of the 40 still counted

  CHECK(rig.device.event_counts().at(0).count == 40);
  CHECK(rig.fake.pending_event_drops() == 8);

  std::uint8_t dropped = 0;
  const std::vector<PinEvent> events = rig.device.poll_events(&dropped);
  CHECK(events.size() == 32); // only the ring's capacity is recoverable...
  CHECK(dropped == 8);        // ...but the loss is reported exactly
  CHECK(events.front().seq == 1);
  CHECK(events.back().seq == 32); // the newest (33..40) were the ones dropped

  // The drop counter is cleared by having been reported.
  CHECK(rig.fake.pending_event_drops() == 0);
}

TEST_CASE("poll_events on an empty queue is a single EVENT_POP returning "
          "nothing",
          "[events][pop]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  rig.device.configure_event(0, EdgeMode::Change);
  rig.fake.clear_log();
  std::uint8_t dropped = 99;
  CHECK(rig.device.poll_events(&dropped).empty());
  CHECK(dropped == 0);
  CHECK(rig.fake.count(Request::EventPop) == 1);
}

// ---- wait_event -------------------------------------------------------------

TEST_CASE("wait_event returns an already-queued event promptly", "[events][wait]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  rig.device.configure_event(0, EdgeMode::Rising);
  rig.fake.set_digital(0, true);

  const auto start = std::chrono::steady_clock::now();
  const std::optional<PinEvent> event = rig.device.wait_event(500ms, 5ms);
  REQUIRE(event.has_value());
  CHECK(event->pin == 0);
  CHECK(event->edge == EdgeMode::Rising);
  CHECK(std::chrono::steady_clock::now() - start < 200ms);
}

TEST_CASE("wait_event times out when nothing arrives", "[events][wait]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  rig.device.configure_event(0, EdgeMode::Rising);

  const auto start = std::chrono::steady_clock::now();
  const std::optional<PinEvent> event = rig.device.wait_event(30ms, 5ms);
  CHECK_FALSE(event.has_value());
  CHECK(std::chrono::steady_clock::now() - start >= 30ms);
}

TEST_CASE("wait_event picks up an event injected mid-wait from another thread",
          "[events][wait][threading]") {
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  rig.device.configure_event(0, EdgeMode::Rising);

  std::thread injector([&rig] {
    std::this_thread::sleep_for(15ms);
    rig.fake.set_digital(0, true);
  });
  const std::optional<PinEvent> event = rig.device.wait_event(500ms, 5ms);
  injector.join();
  REQUIRE(event.has_value());
  CHECK(event->pin == 0);
}

TEST_CASE("wait_event never discards an event beyond the first", "[events][wait]") {
  // Two events queued; wait_event() must hand back exactly one, leaving the
  // other for the next call -- not silently drop it (see Device.h).
  Rig rig(events_board(), fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  rig.device.configure_event(0, EdgeMode::Change);
  toggle(rig, 0, 2);

  const std::optional<PinEvent> first = rig.device.wait_event(100ms, 5ms);
  REQUIRE(first.has_value());
  CHECK(first->seq == 1);
  const std::optional<PinEvent> second = rig.device.wait_event(100ms, 5ms);
  REQUIRE(second.has_value());
  CHECK(second->seq == 2);
}

// ---- DeviceBusy while a Stream runs -----------------------------------------

TEST_CASE("event methods throw DeviceBusy while a Stream is running",
          "[events][stream]") {
  FakeBoard board = events_board();
  board.flags |= USBIO_FLAG_STREAMING;
  board.stream_max_channels = 4;
  Rig rig(board, fast_options());
  rig.device.pin_mode(0, PinMode::Input);
  rig.device.pin_mode(19, PinMode::AnalogIn);
  rig.device.configure_event(0, EdgeMode::Change);

  StreamConfig config;
  config.pins = {19};
  Stream stream = rig.device.start_stream(config);

  CHECK_THROWS_AS(rig.device.configure_event(1, EdgeMode::Rising), DeviceBusy);
  CHECK_THROWS_AS(rig.device.poll_events(), DeviceBusy);
  CHECK_THROWS_AS(rig.device.event_counts(), DeviceBusy);
  CHECK_THROWS_AS(rig.device.wait_event(10ms, 5ms), DeviceBusy);

  stream.stop();
  CHECK_NOTHROW(rig.device.event_counts());
}
