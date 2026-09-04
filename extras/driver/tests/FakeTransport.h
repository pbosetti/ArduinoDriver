// FakeTransport.h - in-process model of the UsbIo firmware.
//
// Implements Transport by following the rules of usbio_protocol.h: the OUT
// check order (bRequest, pin, request-specific checks, queue capacity),
// intended-mode bookkeeping with "unconfigured" pins, STALL + last_error on
// rejected requests, BUSY on data reads, replies clamped to wLength, RESET
// semantics and the not-ready state before begin(). It records every request,
// lets the tests set the shadow input values and inject faults, and exposes
// the state the firmware would have.
#pragma once

#include "arduino_driver/Protocol.h"
#include "arduino_driver/Transport.h"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <initializer_list>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace ArduinoDriver::Testing {

/// What a fake board reports in GET_INFO and GET_PIN_CAPS.
struct FakeBoard {
  std::string name;
  BoardId board_id{BoardId::Unknown};
  std::vector<std::uint8_t> caps; ///< one capability byte per pin
  std::uint8_t adc_bits{10};
  std::uint8_t pwm_bits{8};
  std::uint8_t dac_bits{0};
  std::uint8_t queue_depth{static_cast<std::uint8_t>(QueueDepth)};
  std::uint16_t vref_mv{5000};
  std::uint16_t io_mv{5000};
  std::uint16_t flags{0};
  /// Report this n_ain instead of the real count (exercises the driver's
  /// consistency check).
  std::optional<std::uint8_t> n_ain_override;
  /// usbio_info_t.stream_max_channels; 0 unless a test opts in (set flags
  /// with USBIO_FLAG_STREAMING too, e.g. `board.flags |=
  /// USBIO_FLAG_STREAMING; board.stream_max_channels = 4;`).
  std::uint8_t stream_max_channels{0};
  /// usbio_info_t.event_max_pins; 0 unless a test opts in (set flags with
  /// USBIO_FLAG_EVENTS too, e.g. `board.flags |= USBIO_FLAG_EVENTS;
  /// board.event_max_pins = 4;`).
  std::uint8_t event_max_pins{0};

  std::uint8_t n_pins() const noexcept;
  /// Pins carrying PinCaps::Ain, ascending.
  std::vector<std::uint8_t> analog_pins() const;
  std::uint8_t n_ain() const;
  bool supports_pulldown() const noexcept {
    return (flags & USBIO_FLAG_PULLDOWN) != 0;
  }

  /// UNO R4 Minima as the Renesas firmware reports it: pins 0..19, all DIO;
  /// AIN 14..19; DAC 14; PWM 3,5,6,9,10,11; 14/12/12 bits; 5000/5000 mV; no
  /// INPUT_PULLDOWN (the core maps it to INPUT); no vendor interface.
  static FakeBoard uno_r4_minima();
  /// Portenta H7 as the mbed firmware reports it: 26 pins. D0-D14 = 0..14
  /// DIO+PWM; A0-A3 = 15..18 AIN only (ADC-only pads); A4-A6 = 19..21
  /// DIO+AIN with the DAC on 21; 22 DIO; LEDR/LEDG/LEDB = 23..25 DIO+PWM;
  /// 16/12/12 bits; 3300/3300 mV; vendor interface; INPUT_PULLDOWN.
  static FakeBoard portenta_h7();
};

/// One control transfer as it appeared on the wire.
struct LoggedRequest {
  std::uint8_t request_type; ///< USBIO_REQTYPE_IN or USBIO_REQTYPE_OUT
  std::uint8_t request;      ///< bRequest
  std::uint16_t value;       ///< wValue
  std::uint16_t index;       ///< wIndex
  std::uint16_t length;      ///< wLength (0 for OUT requests)

  bool is_in() const noexcept { return request_type == USBIO_REQTYPE_IN; }
  bool is_out() const noexcept { return request_type == USBIO_REQTYPE_OUT; }
};

class FakeTransport final : public Transport {
public:
  explicit FakeTransport(FakeBoard board);

  // ---- Transport ------------------------------------------------------------
  std::size_t control_in(std::uint8_t request, std::uint16_t value,
                         std::uint16_t index, std::span<std::byte> data,
                         std::chrono::milliseconds timeout) override;
  void control_out(std::uint8_t request, std::uint16_t value,
                   std::uint16_t index,
                   std::chrono::milliseconds timeout) override;
  /// Serves bytes off the bulk queue (see "Bulk streaming model" below), in
  /// chunks controlled by queue_bulk_chunk() where one was queued, otherwise
  /// as many as fit `data`. Returns 0 (not an error) when the queue is
  /// empty, matching Transport::bulk_in()'s "0 on timeout" contract. Throws
  /// NotSupported when the board has no USBIO_FLAG_STREAMING, mirroring
  /// LibusbTransport on a board without a bulk IN endpoint.
  std::size_t bulk_in(std::span<std::byte> data,
                      std::chrono::milliseconds timeout) override;

  // ---- Firmware state -------------------------------------------------------
  const FakeBoard &board() const noexcept { return _board; }
  /// Intended mode recorded when PIN_MODE was accepted; nullopt while the
  /// pin is unconfigured (after boot, or after RESET for analog-only pads).
  std::optional<PinMode> mode(std::uint8_t pin) const;
  /// Shadow digital level: the sampled input, or the last value written.
  bool digital_value(std::uint8_t pin) const;
  std::uint16_t analog_value(std::uint8_t pin) const;
  std::uint16_t pwm_value(std::uint8_t pin) const;
  std::uint16_t dac_value(std::uint8_t pin) const;
  /// Reason of the last STALL (OUT or IN); cleared by GET_STATUS.
  Status last_error() const noexcept { return _last_error; }
  std::uint8_t queue_pending() const noexcept { return _queue_pending; }

  // ---- Stimulus and fault injection -----------------------------------------
  /// Sets the level sampled on an input pin.
  void set_digital(std::uint8_t pin, bool high);
  /// Sets the raw sample of an analog pin.
  void set_analog(std::uint8_t pin, std::uint16_t raw);
  /// The next `reads` data reads (DIO_READ, AI_READ, *_READ_ALL) answer BUSY.
  void inject_busy(unsigned reads) noexcept { _busy_reads = reads; }
  /// While set, every queued OUT request STALLs with QUEUE_FULL (RESET is
  /// still accepted and clears the condition, as in the firmware).
  void set_queue_full(bool full) noexcept { _queue_full = full; }
  /// GET_STATUS reports `n` pending commands, then n-1, ... : one command
  /// "executes" per poll.
  void set_queue_pending(std::uint8_t n) noexcept { _queue_pending = n; }
  /// The next `replies` GET_INFO replies report n_pins == 0, as a device
  /// whose sketch has not reached begin() yet.
  void set_not_ready(unsigned replies) noexcept { _not_ready = replies; }
  /// The next OUT request STALLs with this last_error whatever its content
  /// (Status::Ok: STALL with nothing recorded).
  void stall_next_out(Status reason) noexcept { _forced_stall = reason; }
  /// The next reply to `request` is truncated to `max_len` bytes.
  void truncate_next_reply(Request request, std::size_t max_len) noexcept {
    _truncate = Truncation{request, max_len};
  }
  /// GET_INFO reports this protocol version.
  void set_protocol_version(std::uint16_t version) noexcept {
    _protocol_version = version;
  }
  /// GET_INFO reports this magic (first four characters, zero-padded).
  void set_magic(std::string_view magic) noexcept;

  // ---- Streaming state (mirrors the device-side selection/scheduler) --------
  bool stream_running() const noexcept { return _stream_running; }
  const std::vector<std::uint8_t> &stream_selected() const noexcept {
    return _stream_selected;
  }
  std::uint8_t stream_flags() const noexcept { return _stream_flags; }
  std::uint16_t stream_period_us() const noexcept { return _stream_period_us; }
  /// GET_STREAM_STATUS.seq as of the last queue_stream_records() call
  /// (guarded: safe to call while a Stream's worker thread is running).
  std::uint32_t stream_seq() const noexcept;
  /// Device-side ring overruns GET_STREAM_STATUS should report next
  /// (guarded: safe to call while a Stream's worker thread is running).
  void set_stream_overruns(std::uint32_t n) noexcept;

  // ---- Device clock (millis()/micros()), and pin events ----------------------
  // GET_TIME serves these two counters directly. They also drive the events
  // model below: set_digital() compares the new level against the shadow it
  // replaces and, for a watched pin, runs it through debounce and the
  // bounded event queue exactly as poll() is documented to in
  // usbio_protocol.h -- i.e. set_digital() IS this fake's "poll() detected a
  // level change" moment. Guarded (see "Bulk streaming model" below): an
  // EventWatcher's worker thread reaches GET_TIME/EVENT_* on this same path
  // while the test thread calls set_digital() / advance_millis() /
  // set_millis() / configure_event()-via-Device from another thread.

  /// Sets millis() (does not touch micros()); lets a test build a deliberately
  /// inconsistent millis()/micros() pair, e.g. to exercise GET_TIME decoding
  /// right at a wrap boundary.
  void set_millis(std::uint32_t ms) noexcept;
  /// Sets micros() (does not touch millis()); see set_millis().
  void set_micros(std::uint32_t us) noexcept;
  /// Advances both counters together, the way real firmware ticks would
  /// (each wraps on uint32_t overflow, exactly like the device).
  void advance_millis(std::uint32_t delta_ms) noexcept;
  std::uint32_t fake_millis() const noexcept;
  std::uint32_t fake_micros() const noexcept;

  /// Pins currently watched (EVENT_CONFIG-armed), in arm order -- mirrors
  /// what EVENT_COUNTS would report.
  std::vector<std::uint8_t> watched_event_pins() const;
  /// Events queued and not yet drained by EVENT_POP (guarded snapshot).
  std::size_t event_queue_size() const noexcept;
  /// The saturating drop counter the next EVENT_POP will report (guarded
  /// snapshot; EVENT_POP resets it to 0 as it reports it, exactly like the
  /// real header field).
  std::uint8_t pending_event_drops() const noexcept;

  // ---- Bulk streaming model ---------------------------------------------------
  // The bulk queue is a flat byte buffer bulk_in() drains from; tests build it
  // with the helpers below to exercise framing, straddling, resync and drop
  // accounting without a real timer/thread. A Stream's worker thread calls
  // bulk_in() (and, indirectly, the StreamStatus branch of control_in(), and
  // control_out() for STREAM_STOP via the destructor / stop()) in the
  // background as soon as Device::start_stream() returns, so these methods --
  // and set_stream_overruns(), log(), last(), count(), clear_log() -- are
  // safe to call from the test's thread while a stream is running: the bulk
  // queue, the two counters GET_STREAM_STATUS reports (seq, overruns) and the
  // wire log are all guarded by an internal mutex. Everything else in
  // FakeTransport is still only meant to be touched from one thread at a
  // time, as before.

  /// The generator queue_stream_records() uses: sample i of record r is
  /// `start + step * (r * n_samples + i)` (wrapping mod 2^16); `t_us` of
  /// record r is `t0_us + dt_us * r`.
  void set_stream_ramp(std::uint16_t start, std::uint16_t step,
                       std::uint32_t t0_us, std::uint32_t dt_us) noexcept;
  /// Appends `count` well-formed records built from the ramp and the current
  /// stream_selected()/stream_flags() (so it matches whatever STREAM_START
  /// actually configured) to the bulk queue. `seq_step` lets the first of
  /// these records jump ahead by more than 1 from the running seq counter,
  /// simulating `seq_step - 1` device-side drops (Stream::stats().seq_gaps).
  /// `chunk_plan`, when given, is queued (see queue_bulk_chunk()) under the
  /// same lock as the record bytes, so it takes effect atomically with
  /// them -- the only way to force an exact split point (e.g. a record
  /// straddling two bulk_in() calls) without a race against a Stream's
  /// worker thread that may already be draining the queue.
  void queue_stream_records(std::size_t count, std::uint32_t seq_step = 1,
                            std::initializer_list<std::size_t> chunk_plan = {});
  /// Appends `n` bytes that never form a valid USBIO_STREAM_MAGIC, to
  /// exercise resync.
  void queue_bulk_garbage(std::size_t n);
  /// Appends raw bytes verbatim (for hand-built edge cases).
  void queue_bulk_bytes(std::span<const std::byte> bytes);
  /// Forces the next bulk_in() call to return exactly `n` bytes off the
  /// front of the queue (0 for a zero-length packet, or less than one
  /// record to force it to straddle two calls). One-shot: queue several to
  /// script a whole sequence of calls; once exhausted, bulk_in() goes back
  /// to draining as much of the queue as fits the caller's buffer.
  void queue_bulk_chunk(std::size_t n);
  /// Bytes still queued and not yet served by bulk_in() (guarded).
  std::size_t bulk_queue_size() const noexcept;

  // ---- Wire log -------------------------------------------------------------
  // Returned by value (guarded): a Stream's worker thread appends to the log
  // too (STREAM_STATUS polls, and STREAM_STOP from stop() / the destructor),
  // so a reference into it would not be safe to use outside the lock.
  std::vector<LoggedRequest> log() const;
  /// Most recent request; throws std::logic_error when the log is empty.
  LoggedRequest last() const;
  std::size_t count(Request request) const;
  void clear_log();

private:
  struct Truncation {
    Request request;
    std::size_t max_len;
  };

  [[noreturn]] void stall(Status reason);
  bool take_busy() noexcept;
  std::size_t encode_info(std::span<std::byte> out);
  void check_pin(std::uint8_t pin) const;
  void handle_stream_select(std::uint16_t value, std::uint16_t index);
  void handle_stream_start(std::uint16_t value, std::uint16_t index);
  /// EVENT_CONFIG's device-side logic (pin/mode/value/capacity order per
  /// usbio_protocol.h "Event requests"). Caller (control_out()) has already
  /// checked USBIO_FLAG_EVENTS.
  void handle_event_config(std::uint16_t value, std::uint16_t index);
  /// Removes `pin` from the watch set, if watched (a no-op otherwise). When
  /// `purge_queue` is set (EVENT_CONFIG's EdgeMode::Off), already-queued
  /// events for that pin are discarded too; otherwise (a PIN_MODE that takes
  /// a watched pin out of an INPUT* mode) they are left for EVENT_POP, per
  /// usbio_protocol.h. Caller holds _mutex.
  void unwatch_event_locked(std::uint8_t pin, bool purge_queue);
  /// set_digital()'s edge-detection step: compares `new_level` against the
  /// pin's current shadow value and, for a watched pin whose armed edge mode
  /// wants that transition, runs debounce, bumps the per-pin counter and
  /// tries to enqueue the event (dropping the newest on a full ring). A
  /// no-op for an unwatched pin or a non-edge (level unchanged). Caller holds
  /// _mutex.
  void handle_edge_locked(std::uint8_t pin, bool new_level);

  /// One watched pin, as the device-side model tracks it.
  struct EventWatch {
    std::uint8_t pin;
    EdgeMode edge;
    std::uint8_t debounce_ms;
    std::uint16_t count{0};
    std::optional<std::uint32_t> last_accept_ms;
  };
  /// One queued edge, as EVENT_POP will report it.
  struct QueuedEvent {
    std::uint8_t pin;
    EdgeMode edge;
    std::uint16_t seq;
    std::uint32_t t_ms;
  };

  FakeBoard _board;
  std::vector<std::optional<PinMode>> _mode;
  std::vector<std::uint8_t> _dio;
  std::vector<std::uint16_t> _ain;
  std::vector<std::uint16_t> _pwm;
  std::vector<std::uint16_t> _dac;
  std::vector<LoggedRequest> _log;
  Status _last_error{Status::Ok};
  std::uint8_t _queue_pending{0};
  unsigned _busy_reads{0};
  unsigned _not_ready{0};
  bool _queue_full{false};
  std::optional<Status> _forced_stall;
  std::optional<Truncation> _truncate;
  std::uint16_t _protocol_version{ProtocolVersion};
  std::array<char, USBIO_MAGIC_LEN> _magic{};

  // Streaming (device-side selection/scheduler state).
  bool _stream_running{false};
  std::vector<std::uint8_t> _stream_selected;
  std::uint8_t _stream_flags{0};
  std::uint16_t _stream_period_us{0};
  std::uint32_t _stream_seq{0};
  std::uint32_t _stream_overruns{0};

  // Bulk model, device clock and pin events. _mutex guards exactly the state
  // a Stream's worker thread or an EventWatcher's worker thread touches
  // concurrently with the test thread: the bulk byte queue, the two counters
  // GET_STREAM_STATUS reports (seq, overruns), the millis()/micros() clock,
  // and the events watch set / queue / drop counter. The ramp parameters are
  // only ever touched from the test thread (queue_stream_records() is what
  // turns them into queue bytes) and need no locking.
  mutable std::mutex _mutex;
  std::uint32_t _millis{0};
  std::uint32_t _micros{0};
  std::vector<EventWatch> _event_watch;   ///< arm order == EVENT_COUNTS order
  std::deque<QueuedEvent> _event_queue;   ///< bounded at EventQueueDepth
  std::uint8_t _event_dropped{0};         ///< saturating; cleared by EVENT_POP
  std::deque<std::byte> _bulk_queue;
  std::deque<std::size_t> _bulk_chunks;
  std::uint16_t _ramp_start{0};
  std::uint16_t _ramp_step{0};
  std::uint32_t _ramp_t0_us{0};
  std::uint32_t _ramp_dt_us{0};
  std::uint32_t _ramp_record_index{0};
};

} // namespace ArduinoDriver::Testing
