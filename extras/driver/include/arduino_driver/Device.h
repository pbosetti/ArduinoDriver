// Device.h - high-level, synchronous view of one UsbIo device.
#pragma once

#include "arduino_driver/Errors.h"
#include "arduino_driver/Protocol.h"
#include "arduino_driver/Stream.h"
#include "arduino_driver/Transport.h"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace ArduinoDriver {

/// Tunables of a Device (a namespace-scope struct so that it can be a
/// default argument of the constructor).
struct DeviceOptions {
  /// Timeout of every control transfer.
  std::chrono::milliseconds timeout{100};
  /// Total attempts of an IN request that keeps answering BUSY, and of
  /// GET_STATUS polls in sync(). 0 behaves as 1.
  unsigned busy_max_attempts{50};
  /// Pause between two BUSY attempts.
  std::chrono::microseconds busy_delay{200};
  /// Total GET_INFO attempts while the device reports n_pins == 0 (USB
  /// enumerated before the sketch called UsbIo.begin()). 0 behaves as 1.
  unsigned ready_max_attempts{40};
  /// Pause between two not-ready attempts (40 x 50 ms = 2 s by default).
  std::chrono::milliseconds ready_delay{50};
};

/// Device::read_time() result: the device's raw millis()/micros() at reply
/// time, the reconstructed 64-bit microsecond clock (see
/// reconstruct_micros64() in Protocol.h), and a host-time anchor.
///
/// The anchor lets the host place device time on its own timeline: host_time
/// is the midpoint of a host steady_clock interval bracketing the GET_TIME
/// control transfer (read immediately before and after it), and round_trip
/// is that interval's length. The device's clock at host_time is micros64
/// microseconds since boot, with an uncertainty bounded by round_trip / 2
/// (the classic NTP one-way-delay assumption: the transfer takes about as
/// long in each direction). For a control transfer this is typically a
/// fraction of a millisecond to a couple of milliseconds; it degrades under
/// system load, a USB hub, or a busy device queue, so treat round_trip
/// itself as the authoritative accuracy figure for a given reading rather
/// than the "typical" number in this comment.
struct DeviceTime {
  std::uint32_t millis{0};
  std::uint32_t micros{0};
  std::uint64_t micros64{0}; ///< reconstructed from millis + micros; valid
                             ///< for ~49.7 days from device boot
  std::chrono::steady_clock::time_point host_time{}; ///< midpoint of the
      ///< round trip: the host's best estimate of "now" on its own clock, at
      ///< the moment the device's clock read micros64
  std::chrono::microseconds round_trip{0}; ///< length of the round trip;
      ///< host_time's uncertainty is +/- round_trip / 2
};

/// One UsbIo device behind a Transport.
///
/// Construction issues GET_INFO (magic and protocol version are verified;
/// n_pins == 0 means "not ready" and is retried, see DeviceOptions) and
/// GET_PIN_CAPS, so every later call validates its pin index, the pin's
/// capabilities and the value range locally, before any USB traffic, and
/// throws InvalidPin / NotSupported / InvalidValue accordingly. Rejections
/// coming from the device (STALL on an OUT request, error status in an IN
/// reply) are mapped to the same exception types: BAD_PIN -> InvalidPin,
/// BAD_MODE -> InvalidMode, UNSUPPORTED -> NotSupported, BAD_VALUE ->
/// InvalidValue, QUEUE_FULL -> QueueFull, anything else -> ProtocolError.
/// IN replies reporting BUSY are retried (Options::busy_max_attempts,
/// Options::busy_delay) and DeviceBusy is thrown when the retries run out.
///
/// ---- Threading contract ----------------------------------------------
///
/// Every Device method that touches the transport funnels through send_out()
/// / raw_in() (read_in() and the streaming/event helpers call those too), and
/// each of those two methods holds a private mutex for the duration of its
/// one control transfer. That makes individual control transfers safe to
/// issue concurrently from multiple threads: they are simply serialised at
/// the wire, one at a time, in whatever order they arrive. This is what lets
/// an EventWatcher's worker thread poll EVENT_POP on a background thread
/// while the owning thread keeps calling ordinary methods (digital_write(),
/// analog_read(), ...) without either side needing to know about the other.
///
/// That mutex is deliberately narrower than the pre-existing Stream gate:
/// starting a Stream (start_stream()) still marks the Device "busy" and every
/// *other* method (the ordinary I/O above, plus configure_event() /
/// poll_events() / event_counts() / wait_event() / read_time()) still throws
/// DeviceBusy for as long as it runs, exactly as before -- see Stream.h.
/// EventWatcher gets no equivalent gate: it is just a convenience wrapper
/// around the public, already-thread-safe event methods, so it never locks
/// the caller out. Running more than one EventWatcher (or otherwise driving
/// configure_event() concurrently) over overlapping pins is not guarded
/// against -- the watch-set bookkeeping is entirely on the device, so the
/// last EVENT_CONFIG for a given pin simply wins, same as calling pin_mode()
/// on the same pin from two threads would.
///
/// Lock ordering: the streaming path's internal mutex (StreamState::mutex,
/// guarding poll_stream_status() / end_stream() / start_stream() against each
/// other) is always acquired *before* the control-transfer mutex, never
/// after, because those three methods take it and then call send_out() /
/// raw_in() from inside that critical section. No code path does the
/// opposite (acquire the control-transfer mutex and then reach for
/// StreamState::mutex), so the two never deadlock against each other.
///
/// What is still NOT safe: moving or destroying a Device while a Stream or
/// EventWatcher created from it is still alive (both are references into the
/// Device, like an iterator into its container -- see Stream.h), or blocking
/// an EventWatcher callback on stop()/the destructor of its own EventWatcher
/// (that joins the thread the callback is running on: same deadlock a Stream
/// callback would have). Barring that, ordinary methods and the event
/// methods may be called freely from any thread, at any time, without
/// external synchronisation.
class Device {
public:
  using Options = DeviceOptions;

  /// Takes ownership of the transport and reads GET_INFO / GET_PIN_CAPS.
  /// Throws std::invalid_argument for a null transport, ProtocolError when
  /// the device does not speak this protocol, NotReady when the sketch has
  /// not called begin(), UsbError on transfer failure.
  explicit Device(std::unique_ptr<Transport> transport, Options options = {});
  Device(Device &&) noexcept = default;
  Device &operator=(Device &&) noexcept = default;
  Device(const Device &) = delete;
  Device &operator=(const Device &) = delete;
  ~Device() = default;

  // ---- Static information (no USB traffic) ----------------------------------

  const Info &info() const noexcept { return _info; }
  const Options &options() const noexcept { return _options; }
  /// The transport in use (for diagnostics; never null).
  Transport &transport() noexcept { return *_transport; }
  const Transport &transport() const noexcept { return *_transport; }
  /// Number of addressable pins (Info::n_pins).
  std::size_t pin_count() const noexcept { return _caps.size(); }
  /// Capabilities of one pin. Throws InvalidPin.
  PinCaps pin_caps(std::uint8_t pin) const;
  /// Capabilities of every pin, indexed by pin number.
  std::span<const PinCaps> all_pin_caps() const noexcept { return _caps; }
  /// Pins carrying PinCaps::Ain in ascending order: this is also the order of
  /// the samples returned by read_all_analog().
  const std::vector<std::uint8_t> &analog_pins() const noexcept {
    return _analog_pins;
  }

  // ---- Configuration --------------------------------------------------------

  /// PIN_MODE. Throws NotSupported when the pin lacks the capability the
  /// mode needs (INPUT_PULLDOWN also needs Info::supports_pulldown()).
  /// Pins are "unconfigured" after boot: reads fail with InvalidMode until
  /// a mode is set (or reset() is called).
  void pin_mode(std::uint8_t pin, PinMode mode);

  // ---- Digital I/O ----------------------------------------------------------

  /// DIO_WRITE. The pin must be in OUTPUT mode (else InvalidMode).
  void digital_write(std::uint8_t pin, bool high);
  /// DIO_READ. The pin must be in an INPUT* or OUTPUT mode (else
  /// InvalidMode); OUTPUT pins report the last value written.
  bool digital_read(std::uint8_t pin);
  /// DIO_READ_ALL: one entry per pin, pin i at index i. Pins that are not in
  /// an INPUT* or OUTPUT mode read false.
  std::vector<bool> read_all_digital();

  // ---- Analog input ---------------------------------------------------------

  /// AI_READ: raw sample, 0 .. 2^adc_bits-1. The pin must be in ANALOG_IN
  /// mode (else InvalidMode).
  std::uint16_t analog_read(std::uint8_t pin);
  /// analog_read() scaled to volts: raw * vref_mv / 1000 / (2^adc_bits - 1).
  double analog_read_volts(std::uint8_t pin);
  /// AI_READ_ALL: one raw sample per analog pin, in analog_pins() order.
  /// Pins not in ANALOG_IN mode read 0. Samples may come from different
  /// sampling rounds.
  std::vector<std::uint16_t> read_all_analog();
  /// Volts for a raw sample of this board (see analog_read_volts()).
  double to_volts(std::uint16_t raw) const noexcept;

  // ---- PWM / DAC output -----------------------------------------------------

  /// PWM_WRITE with a raw duty, 0 .. 2^pwm_bits-1 (else InvalidValue). The
  /// pin must be in PWM mode (else InvalidMode).
  void pwm_write(std::uint8_t pin, std::uint16_t duty);
  /// pwm_write() with a duty fraction in 0 .. 1 (else InvalidValue),
  /// rounded to the nearest duty code.
  void pwm_write_fraction(std::uint8_t pin, double fraction);
  /// DAC_WRITE with a raw code, 0 .. 2^dac_bits-1 (else InvalidValue). The
  /// pin must be in DAC mode (else InvalidMode).
  void dac_write(std::uint8_t pin, std::uint16_t value);
  /// dac_write() with a voltage in 0 .. vref_mv/1000 (else InvalidValue),
  /// rounded to the nearest code (vref_mv is the DAC full scale too).
  void dac_write_volts(std::uint8_t pin, double volts);

  // ---- Control --------------------------------------------------------------

  /// GET_STATUS. Returns `last_error`, the reason of the most recent
  /// STALLed request, OUT or IN (Status::Ok when there is none); the device
  /// clears it on every read. Optionally reports the number of accepted
  /// commands that loop() has not executed yet.
  Status status(std::uint8_t *queue_pending = nullptr);
  /// Polls GET_STATUS until the command queue is empty. Throws DeviceBusy
  /// after Options::busy_max_attempts polls. Clears `last_error` as a side
  /// effect.
  void sync();
  /// RESET: every DIO-capable pin back to INPUT, analog-only pads back to
  /// unconfigured, queue cleared.
  void reset();

  // ---- Streaming (Phase 2) ---------------------------------------------------

  /// Builds the channel selection (STREAM_SELECT for each pin, in order) and
  /// starts the device sampling it (STREAM_START), returning an RAII Stream.
  /// Throws NotSupported when info().streaming() is false; InvalidValue when
  /// config.pins is empty, longer than info().stream_max_channels, or the
  /// period is out of range; InvalidPin for an out-of-range pin; InvalidMode
  /// when a pin is not already in ANALOG_IN or an INPUT* mode (the device
  /// STALLs STREAM_SELECT with BAD_MODE); DeviceBusy when a Stream from this
  /// Device is already running. A failed call leaves no pins selected.
  /// See Stream.h for the threading contract this puts in effect for as long
  /// as the returned Stream runs (or is not destroyed / stopped).
  Stream start_stream(StreamConfig config);

  // ---- Device time ------------------------------------------------------

  /// GET_TIME plus a host-time anchor: see DeviceTime for what the result
  /// means and how accurate it is. Throws DeviceBusy while a Stream is
  /// running (see the threading contract above).
  DeviceTime read_time();

  // ---- Pin events (only when info().events() is true) -----------------------

  /// EVENT_CONFIG. EdgeMode::Off unwatches the pin (clears its counter and
  /// discards its queued events); arming an already-watched pin replaces its
  /// edge mode and debounce and resets its counter. Throws NotSupported when
  /// info().events() is false; InvalidPin for pin >= pin_count(); InvalidMode
  /// when the pin lacks digital I/O or is not in an INPUT* mode (the device
  /// STALLs EVENT_CONFIG with BAD_MODE); InvalidValue for an unknown edge
  /// mode, for debounce > MaxDebounceMs, or when arming a pin that is not yet
  /// watched would exceed info().event_max_pins (the device STALLs
  /// EVENT_CONFIG with BAD_VALUE either way); DeviceBusy while a Stream is
  /// running.
  void configure_event(std::uint8_t pin, EdgeMode edge,
                       std::chrono::milliseconds debounce = {});
  /// Drains every event currently queued: one or more EVENT_POP calls,
  /// looping while the reply's `pending` is set, oldest edge first. When
  /// `dropped` is not null, it receives the total events the device-side
  /// ring had to drop since the previous poll_events() / wait_event() call
  /// (saturating at 255); the per-pin counters in event_counts() stay exact
  /// regardless of drops -- see usbio_protocol.h "Pin events". Throws
  /// NotSupported when info().events() is false; DeviceBusy while a Stream is
  /// running.
  std::vector<PinEvent> poll_events(std::uint8_t *dropped = nullptr);
  /// EVENT_COUNTS: the accepted-edge counter of every currently watched pin,
  /// in the order the pins were armed. Throws NotSupported when
  /// info().events() is false; DeviceBusy while a Stream is running.
  std::vector<EventCount> event_counts();
  /// Polls for a single event every `poll_interval` until one arrives or
  /// `timeout` elapses; returns nullopt on timeout. Unlike poll_events(),
  /// this never pops more than one event per device round trip, so an event
  /// that arrives during the wait but is not the first one taken stays
  /// queued on the device for the next wait_event() / poll_events() call --
  /// nothing found this way is ever lost. Throws NotSupported when
  /// info().events() is false; DeviceBusy while a Stream is running.
  std::optional<PinEvent>
  wait_event(std::chrono::milliseconds timeout,
            std::chrono::milliseconds poll_interval = std::chrono::milliseconds{20});

private:
  friend class Stream;

  void load_info();
  void load_caps();
  /// pin_caps() plus a capability check (NotSupported).
  PinCaps require_caps(std::uint8_t pin, std::uint8_t cap_mask,
                       std::string_view feature) const;
  /// OUT request; STALL is translated through GET_STATUS.last_error.
  void send_out(Request request, std::uint16_t value, std::uint16_t index);
  /// IN request carrying a status byte: BUSY retry, status mapping and
  /// minimum-length check. Returns the bytes received.
  std::size_t read_in(Request request, std::uint16_t index,
                      std::span<std::byte> reply, std::size_t min_len);
  /// IN request without interpretation (wValue = 0).
  std::size_t raw_in(Request request, std::uint16_t index,
                     std::span<std::byte> reply);
  /// Throws DeviceBusy when a Stream is running; called at the top of every
  /// method above that touches the transport (not by the Stream-only calls
  /// below, and not by the pure accessors, which stay usable while streaming).
  void check_not_streaming(std::string_view what) const;
  /// GET_STREAM_STATUS, for Stream's worker to refresh its stats(). Callable
  /// only while a stream is running; serialised against end_stream().
  StreamStatus poll_stream_status();
  /// STREAM_STOP (best effort: errors are swallowed) and clears the
  /// streaming flag. Called by Stream::stop() / its destructor; never
  /// throws, safe to call more than once.
  void end_stream() noexcept;
  /// Throws NotSupported when info().events() is false; called at the top of
  /// every event method (mirrors require_caps() for the capability flag).
  void require_events(std::string_view what) const;
  /// One EVENT_POP requesting at most `max_events` (0 = as many as the wire
  /// reply holds, i.e. min(MaxEventsPerPop, queued)); decoded entries are
  /// appended to `out`, oldest first. `dropped` / `pending`, when not null,
  /// receive the reply header's fields. Building block for poll_events()
  /// (loops with max_events = 0 while pending) and wait_event() (calls with
  /// max_events = 1 so it never discards an event beyond the first).
  void pop_events_once(std::uint16_t max_events, std::vector<PinEvent> &out,
                       std::uint8_t *dropped, bool *pending);

  std::unique_ptr<Transport> _transport;
  Options _options;
  Info _info{};
  std::vector<PinCaps> _caps;
  std::vector<std::uint8_t> _analog_pins;

  /// Streaming state lives behind a pointer so Device stays movable
  /// (std::atomic and std::mutex are neither): moving a Device with a
  /// running Stream is not supported (see Stream.h).
  struct StreamState {
    std::atomic<bool> streaming{false};
    std::mutex mutex; ///< serialises poll_stream_status() / end_stream()
                      ///< against each other and against start_stream()
    std::vector<std::uint8_t> selected; ///< STREAM_SELECT bookkeeping
  };
  std::unique_ptr<StreamState> _stream{std::make_unique<StreamState>()};

  /// Serialises individual control transfers -- send_out()'s and raw_in()'s
  /// single control_out()/control_in() call each -- so Device methods can be
  /// called concurrently from multiple threads (see the threading contract
  /// above). Behind a pointer for the same reason StreamState is: a plain
  /// std::mutex member would make Device non-movable. Kept as a distinct
  /// mutex from StreamState::mutex (never the same object) so the two nest
  /// predictably: StreamState::mutex is always acquired first, this one
  /// second, never the other way around.
  struct IoState {
    std::mutex mutex;
  };
  std::unique_ptr<IoState> _io{std::make_unique<IoState>()};
};

} // namespace ArduinoDriver
