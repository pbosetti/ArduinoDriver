// EventWatcher.h - RAII background polling of pin events (Device::
// configure_event() / poll_events()), modelled on Stream.h.
//
// Unlike Stream, EventWatcher needs no privileged access to Device and no
// busy-lock of its own: configure_event() / poll_events() are ordinary
// public Device methods, already safe to call from any thread because every
// control transfer is individually serialised inside Device (see the
// threading contract in Device.h). EventWatcher is just a convenience
// wrapper that arms the requested pins, runs a worker thread that calls
// poll_events() on a timer and hands each decoded event to a callback, and
// disarms the pins again on stop() / the destructor. It never marks the
// Device "busy": the thread that constructs it can keep calling ordinary
// Device methods (digital_write(), pin_mode(), ...) the whole time.
//
// A Stream that is running on the same Device still blocks poll_events()
// (DeviceBusy), exactly as it blocks every other ordinary method -- an
// EventWatcher does not change that, it just does not add a *second* such
// gate of its own.
//
// An EventWatcher must not outlive the Device it was built from, same as a
// Stream.
#pragma once

#include "arduino_driver/Protocol.h"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace ArduinoDriver {

class Device;

/// One pin to arm, as given to EventWatcher's constructor.
struct WatchedPin {
  std::uint8_t pin{0};
  EdgeMode edge{EdgeMode::Change};
  std::chrono::milliseconds debounce{0};
};

/// Tunables of an EventWatcher.
struct EventWatcherConfig {
  /// Pins to arm (Device::configure_event() for each, in order) when the
  /// EventWatcher is constructed. Must be non-empty.
  std::vector<WatchedPin> pins;
  /// How often the worker thread calls Device::poll_events().
  std::chrono::milliseconds poll_interval{20};
};

/// Counters accumulated by an EventWatcher since it started.
struct EventWatcherStats {
  std::uint64_t events_delivered{0}; ///< callback invocations so far
  std::uint64_t dropped{0}; ///< usbio_event_header_t.dropped, summed across
                            ///< every poll_events() call the worker made
};

/// RAII pin-event watcher. Move-only; the destructor stops the worker thread
/// and disarms the pins it armed (best effort). See the file comment above
/// for the threading contract.
class EventWatcher {
public:
  /// Invoked from the worker thread with each event, in the order
  /// poll_events() returned it. Keep it fast: it runs inline in the worker's
  /// poll loop, ahead of the next poll_events() call.
  using Callback = std::function<void(const PinEvent &)>;

  /// Arms config.pins on `device` (Device::configure_event(), in order) and
  /// starts the worker thread. Throws whatever configure_event() throws
  /// (NotSupported, InvalidPin, InvalidMode, InvalidValue, DeviceBusy) if
  /// arming any pin fails; pins already armed by this call are best-effort
  /// unarmed before the exception propagates, and the worker thread is never
  /// started. Throws std::invalid_argument when config.pins is empty.
  EventWatcher(Device &device, EventWatcherConfig config, Callback callback);
  EventWatcher(EventWatcher &&other) noexcept;
  EventWatcher &operator=(EventWatcher &&other) noexcept;
  EventWatcher(const EventWatcher &) = delete;
  EventWatcher &operator=(const EventWatcher &) = delete;
  /// Stops the worker thread and disarms the pins (best effort; errors are
  /// swallowed).
  ~EventWatcher();

  /// Snapshot of the counters accumulated so far. Safe to call from any
  /// thread at any time.
  EventWatcherStats stats() const;

  /// True until stop() has completed (or the destructor has run).
  bool running() const noexcept;

  /// Pins armed, in the order given to the constructor.
  const std::vector<WatchedPin> &pins() const noexcept;

  /// Stops the worker thread and disarms the pins (best effort). Idempotent;
  /// the destructor calls it if the caller does not. Safe to call from any
  /// thread, including from inside the callback of a *different*
  /// EventWatcher -- but calling it from this EventWatcher's own callback
  /// deadlocks (it would join the thread the callback is running on), same
  /// as stopping a Stream from its own callback would.
  void stop();

private:
  struct Impl;
  std::unique_ptr<Impl> _impl;
};

} // namespace ArduinoDriver
