// FakeTransport.cpp - see FakeTransport.h.
#include "FakeTransport.h"

#include <algorithm>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace ArduinoDriver::Testing {

namespace {

void set_caps(FakeBoard &board, std::uint8_t first, std::uint8_t last,
              std::uint8_t mask) {
  for (std::uint8_t pin = first; pin <= last; ++pin) {
    board.caps.at(pin) = mask;
  }
}

void add_cap(FakeBoard &board, std::uint8_t pin, std::uint8_t mask) {
  board.caps.at(pin) = static_cast<std::uint8_t>(board.caps.at(pin) | mask);
}

bool is_digital_mode(std::optional<PinMode> mode) noexcept {
  return mode == PinMode::Input || mode == PinMode::Output ||
         mode == PinMode::InputPullup || mode == PinMode::InputPulldown;
}

/// STREAM_SELECT accepts ANALOG_IN or an *input* digital mode -- not OUTPUT.
bool is_input_mode(std::optional<PinMode> mode) noexcept {
  return mode == PinMode::Input || mode == PinMode::InputPullup ||
         mode == PinMode::InputPulldown;
}

std::uint16_t clamp_u16(std::size_t n) noexcept {
  return static_cast<std::uint16_t>(std::min<std::size_t>(n, 0xFFFFu));
}

} // namespace

// ---- FakeBoard --------------------------------------------------------------

std::uint8_t FakeBoard::n_pins() const noexcept {
  return static_cast<std::uint8_t>(caps.size());
}

std::vector<std::uint8_t> FakeBoard::analog_pins() const {
  std::vector<std::uint8_t> pins;
  for (std::size_t pin = 0; pin < caps.size(); ++pin) {
    if (PinCaps{caps[pin]}.ain()) {
      pins.push_back(static_cast<std::uint8_t>(pin));
    }
  }
  return pins;
}

std::uint8_t FakeBoard::n_ain() const {
  return n_ain_override.value_or(
      static_cast<std::uint8_t>(analog_pins().size()));
}

FakeBoard FakeBoard::uno_r4_minima() {
  FakeBoard board;
  board.name = "UNO R4 Minima (fake)";
  board.board_id = BoardId::UnoR4Minima;
  board.caps.assign(20, PinCaps::Dio);
  constexpr std::array<std::uint8_t, 6> pwm_pins{3, 5, 6, 9, 10, 11};
  for (const std::uint8_t pin : pwm_pins) {
    add_cap(board, pin, PinCaps::Pwm);
  }
  for (std::uint8_t pin = 14; pin <= 19; ++pin) {
    add_cap(board, pin, PinCaps::Ain);
  }
  add_cap(board, 14, PinCaps::Dac);
  board.adc_bits = 14;
  board.pwm_bits = 12;
  board.dac_bits = 12;
  board.vref_mv = 5000;
  board.io_mv = 5000;
  board.flags = 0; // no vendor interface, INPUT_PULLDOWN maps to INPUT
  return board;
}

FakeBoard FakeBoard::portenta_h7() {
  FakeBoard board;
  board.name = "Portenta H7 (fake)";
  board.board_id = BoardId::PortentaH7;
  board.caps.assign(26, 0);
  set_caps(board, 0, 14, PinCaps::Dio | PinCaps::Pwm);  // D0..D14
  set_caps(board, 15, 18, PinCaps::Ain);                // A0..A3, ADC-only
  set_caps(board, 19, 21, PinCaps::Dio | PinCaps::Ain); // A4..A6
  add_cap(board, 21, PinCaps::Dac);                     // A6 = DAC
  set_caps(board, 22, 22, PinCaps::Dio);
  set_caps(board, 23, 25, PinCaps::Dio | PinCaps::Pwm); // LEDR, LEDG, LEDB
  board.adc_bits = 16;
  board.pwm_bits = 12;
  board.dac_bits = 12;
  board.vref_mv = 3300;
  board.io_mv = 3300;
  board.flags = USBIO_FLAG_VENDOR_INTERFACE | USBIO_FLAG_PULLDOWN;
  return board;
}

// ---- FakeTransport ----------------------------------------------------------

FakeTransport::FakeTransport(FakeBoard board)
    : _board(std::move(board)), _mode(_board.caps.size()),
      _dio(_board.caps.size(), 0), _ain(_board.caps.size(), 0),
      _pwm(_board.caps.size(), 0), _dac(_board.caps.size(), 0) {
  if (_board.caps.size() > MaxPins) {
    throw std::invalid_argument("FakeBoard: more than USBIO_MAX_PINS pins");
  }
  set_magic(Magic);
}

void FakeTransport::set_magic(std::string_view magic) noexcept {
  _magic.fill('\0');
  const std::size_t n = std::min(magic.size(), _magic.size());
  std::copy_n(magic.begin(), n, _magic.begin());
}

std::size_t FakeTransport::control_in(std::uint8_t request, std::uint16_t value,
                                      std::uint16_t index,
                                      std::span<std::byte> data,
                                      std::chrono::milliseconds) {
  {
    // A Stream's worker thread polls GET_STREAM_STATUS on this same path
    // while the test thread may be reading the log (log(), count(), ...).
    std::lock_guard<std::mutex> lock(_mutex);
    _log.push_back(
        {USBIO_REQTYPE_IN, request, value, index, clamp_u16(data.size())});
  }

  std::array<std::byte, MaxReplyLen> reply{};
  std::size_t len = 0;
  const std::uint8_t n_pins = _board.n_pins();

  switch (static_cast<Request>(request)) {
  case Request::GetInfo:
    len = encode_info(reply);
    break;

  case Request::GetPinCaps:
    // wIndex = first pin; wIndex >= n_pins -> zero-length reply.
    for (std::size_t pin = index; pin < n_pins; ++pin) {
      write_u8(reply, len++, _board.caps[pin]);
    }
    break;

  case Request::DioRead: {
    // Firmware order: pin, mode, then BUSY.
    Status status = Status::Ok;
    std::uint8_t level = 0;
    if (index >= n_pins) {
      status = Status::BadPin;
    } else if (!is_digital_mode(_mode[index])) {
      status = Status::BadMode;
    } else if (take_busy()) {
      status = Status::Busy;
    } else {
      level = _dio[index];
    }
    write_u8(reply, 0, static_cast<std::uint8_t>(status));
    write_u8(reply, 1, level);
    len = DioReplyLen;
    break;
  }

  case Request::AiRead: {
    Status status = Status::Ok;
    std::uint16_t raw = 0;
    if (index >= n_pins) {
      status = Status::BadPin;
    } else if (_mode[index] != PinMode::AnalogIn) {
      status = Status::BadMode;
    } else if (take_busy()) {
      status = Status::Busy;
    } else {
      raw = _ain[index];
    }
    write_u8(reply, 0, static_cast<std::uint8_t>(status));
    write_u8(reply, 1, 0);
    write_u16le(reply, 2, raw);
    len = AiReplyLen;
    break;
  }

  case Request::DioReadAll: {
    const Status status = take_busy() ? Status::Busy : Status::Ok;
    write_u8(reply, 0, static_cast<std::uint8_t>(status));
    write_u8(reply, 1, 0);
    for (std::size_t pin = 0; pin < n_pins; ++pin) {
      if (is_digital_mode(_mode[pin]) && _dio[pin] != 0) {
        reply[AllHeaderLen + pin / 8] |=
            static_cast<std::byte>(1u << (pin % 8));
      }
    }
    len = dio_read_all_len(n_pins);
    break;
  }

  case Request::AiReadAll: {
    const Status status = take_busy() ? Status::Busy : Status::Ok;
    write_u8(reply, 0, static_cast<std::uint8_t>(status));
    write_u8(reply, 1, 0);
    const std::vector<std::uint8_t> pins = _board.analog_pins();
    for (std::size_t i = 0; i < pins.size(); ++i) {
      const std::uint8_t pin = pins[i];
      const std::uint16_t raw = _mode[pin] == PinMode::AnalogIn ? _ain[pin] : 0;
      write_u16le(reply, AllHeaderLen + 2 * i, raw);
    }
    len = ai_read_all_len(pins.size());
    break;
  }

  case Request::GetStatus:
    write_u8(reply, 0, static_cast<std::uint8_t>(Status::Ok));
    write_u8(reply, 1, _queue_pending);
    write_u8(reply, 2, static_cast<std::uint8_t>(_last_error));
    write_u8(reply, 3, 0);
    _last_error = Status::Ok; // cleared by the read
    if (_queue_pending > 0) {
      --_queue_pending; // loop() executed one command meanwhile
    }
    len = StatusReplyLen;
    break;

  case Request::StreamStatus:
    if ((_board.flags & USBIO_FLAG_STREAMING) == 0) {
      stall(Status::Unsupported);
    }
    write_u8(reply, StreamStatusOffset::Status, static_cast<std::uint8_t>(Status::Ok));
    write_u8(reply, StreamStatusOffset::Running, _stream_running ? 1 : 0);
    write_u8(reply, StreamStatusOffset::NChannels,
            static_cast<std::uint8_t>(_stream_selected.size()));
    write_u8(reply, StreamStatusOffset::Flags, _stream_flags);
    write_u16le(reply, StreamStatusOffset::PeriodUs, _stream_period_us);
    write_u16le(reply, StreamStatusOffset::Reserved, 0);
    {
      // seq/overruns are also touched by queue_stream_records() /
      // set_stream_overruns() from the test thread while a Stream's worker
      // thread is running.
      std::lock_guard<std::mutex> lock(_mutex);
      write_u32le(reply, StreamStatusOffset::Seq, _stream_seq);
      write_u32le(reply, StreamStatusOffset::Overruns, _stream_overruns);
    }
    len = StreamStatusLen;
    break;

  case Request::GetTime:
    // "always OK" per usbio_protocol.h: no pin/mode/busy checks.
    write_u8(reply, TimeReplyOffset::Status, static_cast<std::uint8_t>(Status::Ok));
    write_u8(reply, TimeReplyOffset::Reserved, 0);
    write_u16le(reply, TimeReplyOffset::Reserved2, 0);
    {
      // The clock is also touched by set_millis()/set_micros()/
      // advance_millis() from the test thread while an EventWatcher's
      // worker thread may be reading it here.
      std::lock_guard<std::mutex> lock(_mutex);
      write_u32le(reply, TimeReplyOffset::Millis, _millis);
      write_u32le(reply, TimeReplyOffset::Micros, _micros);
    }
    len = TimeReplyLen;
    break;

  case Request::EventPop: {
    if ((_board.flags & USBIO_FLAG_EVENTS) == 0) {
      stall(Status::Unsupported);
    }
    // wIndex = max events, 0 = as many as fit (min(MaxEventsPerPop, queued)).
    std::lock_guard<std::mutex> lock(_mutex);
    const std::size_t requested =
        index == 0 ? MaxEventsPerPop : std::min<std::size_t>(index, MaxEventsPerPop);
    const std::size_t n_events = std::min(requested, _event_queue.size());
    write_u8(reply, EventHeaderOffset::Status, static_cast<std::uint8_t>(Status::Ok));
    write_u8(reply, EventHeaderOffset::Count, static_cast<std::uint8_t>(n_events));
    write_u8(reply, EventHeaderOffset::Dropped, _event_dropped);
    _event_dropped = 0; // cleared by the read, like GET_STATUS.last_error
    for (std::size_t i = 0; i < n_events; ++i) {
      const QueuedEvent &e = _event_queue[i];
      const std::size_t base = EventHeaderLen + i * EventLen;
      write_u8(reply, base + EventOffset::Pin, e.pin);
      write_u8(reply, base + EventOffset::Edge, static_cast<std::uint8_t>(e.edge));
      write_u16le(reply, base + EventOffset::Seq, e.seq);
      write_u32le(reply, base + EventOffset::TMs, e.t_ms);
    }
    _event_queue.erase(_event_queue.begin(),
                       _event_queue.begin() + static_cast<std::ptrdiff_t>(n_events));
    write_u8(reply, EventHeaderOffset::Pending,
            static_cast<std::uint8_t>(std::min<std::size_t>(_event_queue.size(), 255)));
    len = event_pop_len(n_events);
    break;
  }

  case Request::EventCounts: {
    if ((_board.flags & USBIO_FLAG_EVENTS) == 0) {
      stall(Status::Unsupported);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    write_u8(reply, EventHeaderOffset::Status, static_cast<std::uint8_t>(Status::Ok));
    write_u8(reply, EventHeaderOffset::Count,
            static_cast<std::uint8_t>(_event_watch.size()));
    write_u8(reply, EventHeaderOffset::Dropped, 0); // EVENT_POP-only field
    write_u8(reply, EventHeaderOffset::Pending, 0);  // EVENT_POP-only field
    for (std::size_t i = 0; i < _event_watch.size(); ++i) {
      const EventWatch &w = _event_watch[i];
      const std::size_t base = EventHeaderLen + i * EventCountLen;
      write_u8(reply, base + EventCountOffset::Pin, w.pin);
      write_u8(reply, base + EventCountOffset::Mode,
              static_cast<std::uint8_t>(w.edge));
      write_u16le(reply, base + EventCountOffset::Count, w.count);
    }
    len = event_counts_len(_event_watch.size());
    break;
  }

  default:
    // Unknown bRequest, or an OUT request issued in the IN direction.
    stall(Status::BadCmd);
  }

  if (_truncate && _truncate->request == static_cast<Request>(request)) {
    len = std::min(len, _truncate->max_len);
    _truncate.reset();
  }
  const std::size_t n = std::min(len, data.size()); // clamp to wLength
  std::copy_n(reply.begin(), n, data.begin());
  return n;
}

void FakeTransport::control_out(std::uint8_t request, std::uint16_t value,
                                std::uint16_t index,
                                std::chrono::milliseconds) {
  {
    // See control_in(): a Stream's worker thread reaches this path too
    // (STREAM_STOP, from stop() / the destructor).
    std::lock_guard<std::mutex> lock(_mutex);
    _log.push_back({USBIO_REQTYPE_OUT, request, value, index, 0});
  }

  if (_forced_stall) {
    const Status reason = *_forced_stall;
    _forced_stall.reset();
    stall(reason);
  }

  // Firmware order: bRequest, pin, request-specific checks, queue capacity.
  const std::uint8_t n_pins = _board.n_pins();
  const auto req = static_cast<Request>(request);
  switch (req) {
  case Request::Reset:
    // Always accepted: queue cleared, DIO-capable pins back to INPUT,
    // analog-only pads back to unconfigured, analog shadow cleared. Also
    // stops a running stream and clears its selection.
    for (std::size_t pin = 0; pin < n_pins; ++pin) {
      if (PinCaps{_board.caps[pin]}.dio()) {
        _mode[pin] = PinMode::Input;
      } else {
        _mode[pin].reset();
      }
      _ain[pin] = 0;
    }
    _queue_pending = 0;
    _queue_full = false;
    _stream_running = false;
    _stream_selected.clear();
    {
      // RESET unwatches every pin and clears the event queue (see
      // usbio_protocol.h "Event requests"); also touched by an
      // EventWatcher's worker thread via EVENT_POP.
      std::lock_guard<std::mutex> lock(_mutex);
      _event_watch.clear();
      _event_queue.clear();
      _event_dropped = 0;
    }
    return;
  case Request::EventConfig:
    if ((_board.flags & USBIO_FLAG_EVENTS) == 0) {
      stall(Status::Unsupported);
    }
    handle_event_config(value, index);
    return;
  case Request::StreamSelect:
    if ((_board.flags & USBIO_FLAG_STREAMING) == 0) {
      stall(Status::Unsupported);
    }
    handle_stream_select(value, index);
    return;
  case Request::StreamStart:
    if ((_board.flags & USBIO_FLAG_STREAMING) == 0) {
      stall(Status::Unsupported);
    }
    handle_stream_start(value, index);
    return;
  case Request::StreamStop:
    if ((_board.flags & USBIO_FLAG_STREAMING) == 0) {
      stall(Status::Unsupported);
    }
    _stream_running = false; // always accepted; keeps the selection
    return;
  case Request::PinMode:
  case Request::DioWrite:
  case Request::PwmWrite:
  case Request::DacWrite:
    break;
  default:
    stall(Status::BadCmd);
  }
  if (index >= n_pins) {
    stall(Status::BadPin);
  }

  switch (req) {
  case Request::PinMode: {
    if (value >= PinModeCount) {
      stall(Status::BadValue);
    }
    const auto mode = static_cast<PinMode>(value);
    if (!mode_supported(mode, PinCaps{_board.caps[index]},
                        _board.supports_pulldown())) {
      stall(Status::Unsupported);
    }
    if (_queue_full) {
      stall(Status::QueueFull);
    }
    _mode[index] = mode; // intended mode, recorded at acceptance
    if (!is_input_mode(mode)) {
      // "A PIN_MODE that takes a watched pin out of an INPUT* mode
      // unwatches it (its queued events remain readable)" -- usbio_protocol.h.
      std::lock_guard<std::mutex> lock(_mutex);
      unwatch_event_locked(static_cast<std::uint8_t>(index),
                           /*purge_queue=*/false);
    }
    return;
  }
  case Request::DioWrite:
    if (_mode[index] != PinMode::Output) {
      stall(Status::BadMode);
    }
    if (_queue_full) {
      stall(Status::QueueFull);
    }
    _dio[index] = value != 0 ? 1 : 0;
    return;
  case Request::PwmWrite:
    if (_mode[index] != PinMode::Pwm) {
      stall(Status::BadMode);
    }
    if (value > max_value(_board.pwm_bits)) {
      stall(Status::BadValue);
    }
    if (_queue_full) {
      stall(Status::QueueFull);
    }
    _pwm[index] = value;
    return;
  case Request::DacWrite:
    if (_mode[index] != PinMode::Dac) {
      stall(Status::BadMode);
    }
    if (value > max_value(_board.dac_bits)) {
      stall(Status::BadValue);
    }
    if (_queue_full) {
      stall(Status::QueueFull);
    }
    _dac[index] = value;
    return;
  default:
    stall(Status::BadCmd);
  }
}

// ---- State access -----------------------------------------------------------

void FakeTransport::check_pin(std::uint8_t pin) const {
  if (pin >= _board.n_pins()) {
    throw std::out_of_range("FakeTransport: pin " + std::to_string(pin) +
                            " does not exist on " + _board.name);
  }
}

std::optional<PinMode> FakeTransport::mode(std::uint8_t pin) const {
  check_pin(pin);
  return _mode[pin];
}

bool FakeTransport::digital_value(std::uint8_t pin) const {
  check_pin(pin);
  return _dio[pin] != 0;
}

std::uint16_t FakeTransport::analog_value(std::uint8_t pin) const {
  check_pin(pin);
  return _ain[pin];
}

std::uint16_t FakeTransport::pwm_value(std::uint8_t pin) const {
  check_pin(pin);
  return _pwm[pin];
}

std::uint16_t FakeTransport::dac_value(std::uint8_t pin) const {
  check_pin(pin);
  return _dac[pin];
}

void FakeTransport::set_digital(std::uint8_t pin, bool high) {
  check_pin(pin);
  const std::uint8_t new_level = high ? 1 : 0;
  if (_dio[pin] != new_level) {
    // This is poll()'s "shadow just refreshed, compare against the
    // previous one" moment (see usbio_protocol.h "Pin events"): only an
    // actual level change is a candidate edge.
    std::lock_guard<std::mutex> lock(_mutex);
    handle_edge_locked(pin, high);
  }
  _dio[pin] = new_level;
}

void FakeTransport::set_analog(std::uint8_t pin, std::uint16_t raw) {
  check_pin(pin);
  _ain[pin] = raw;
}

// ---- Device clock and pin events --------------------------------------------

void FakeTransport::set_millis(std::uint32_t ms) noexcept {
  std::lock_guard<std::mutex> lock(_mutex);
  _millis = ms;
}

void FakeTransport::set_micros(std::uint32_t us) noexcept {
  std::lock_guard<std::mutex> lock(_mutex);
  _micros = us;
}

void FakeTransport::advance_millis(std::uint32_t delta_ms) noexcept {
  std::lock_guard<std::mutex> lock(_mutex);
  _millis += delta_ms;                     // wraps like the real millis()
  _micros += delta_ms * 1000u;            // wraps like the real micros()
}

std::uint32_t FakeTransport::fake_millis() const noexcept {
  std::lock_guard<std::mutex> lock(_mutex);
  return _millis;
}

std::uint32_t FakeTransport::fake_micros() const noexcept {
  std::lock_guard<std::mutex> lock(_mutex);
  return _micros;
}

std::vector<std::uint8_t> FakeTransport::watched_event_pins() const {
  std::lock_guard<std::mutex> lock(_mutex);
  std::vector<std::uint8_t> pins;
  pins.reserve(_event_watch.size());
  for (const EventWatch &w : _event_watch) {
    pins.push_back(w.pin);
  }
  return pins;
}

std::size_t FakeTransport::event_queue_size() const noexcept {
  std::lock_guard<std::mutex> lock(_mutex);
  return _event_queue.size();
}

std::uint8_t FakeTransport::pending_event_drops() const noexcept {
  std::lock_guard<std::mutex> lock(_mutex);
  return _event_dropped;
}

void FakeTransport::handle_edge_locked(std::uint8_t pin, bool new_level) {
  const auto it =
      std::find_if(_event_watch.begin(), _event_watch.end(),
                  [pin](const EventWatch &w) { return w.pin == pin; });
  if (it == _event_watch.end()) {
    return; // not watched
  }
  const EdgeMode edge = new_level ? EdgeMode::Rising : EdgeMode::Falling;
  if (it->edge != EdgeMode::Change && it->edge != edge) {
    return; // armed mode does not want this direction
  }
  // Debounce: "first edge wins" -- once accepted, further edges on this pin
  // are ignored until the window has passed. Unsigned subtraction wraps
  // correctly even across a millis() rollover, as long as the true elapsed
  // time is under ~49.7 days.
  if (it->debounce_ms != 0 && it->last_accept_ms.has_value()) {
    const auto elapsed =
        static_cast<std::uint32_t>(_millis - *it->last_accept_ms);
    if (elapsed < it->debounce_ms) {
      return;
    }
  }
  it->last_accept_ms = _millis;
  it->count = static_cast<std::uint16_t>(it->count + 1); // wraps; counts
                                                          // every accepted
                                                          // edge regardless
                                                          // of queue fate
  if (_event_queue.size() >= EventQueueDepth) {
    if (_event_dropped < 255) {
      ++_event_dropped;
    }
    return; // ring full: the newest edge is dropped (but already counted)
  }
  _event_queue.push_back({pin, edge, it->count, _millis});
}

void FakeTransport::unwatch_event_locked(std::uint8_t pin, bool purge_queue) {
  const auto it =
      std::find_if(_event_watch.begin(), _event_watch.end(),
                  [pin](const EventWatch &w) { return w.pin == pin; });
  if (it == _event_watch.end()) {
    return;
  }
  _event_watch.erase(it);
  if (purge_queue) {
    _event_queue.erase(
        std::remove_if(_event_queue.begin(), _event_queue.end(),
                       [pin](const QueuedEvent &e) { return e.pin == pin; }),
        _event_queue.end());
  }
}

void FakeTransport::handle_event_config(std::uint16_t value,
                                        std::uint16_t index) {
  // Order per usbio_protocol.h "Event requests": pin range, [mode, value for
  // arming], capacity. EdgeMode::Off (unwatch) is accepted unconditionally
  // once the pin index is valid: like STREAM_SELECT's "removing an
  // unselected pin is a no-op", disarming does not require the pin to
  // currently be a valid input, so "configure_event(pin, Off)" is always a
  // safe way to say "stop watching this pin" -- including from a clean
  // teardown that already moved the pin to a different mode, or an
  // EventWatcher's best-effort disarm on a pin it never actually got to arm.
  if (index >= _board.n_pins()) {
    stall(Status::BadPin);
  }
  const auto pin = static_cast<std::uint8_t>(index);
  const auto edge_code = static_cast<std::uint8_t>(value & 0xFFu);
  if (edge_code == USBIO_EDGE_OFF) {
    std::lock_guard<std::mutex> lock(_mutex);
    unwatch_event_locked(pin, /*purge_queue=*/true);
    return;
  }
  if (!(PinCaps{_board.caps[pin]}.dio() && is_input_mode(_mode[pin]))) {
    stall(Status::BadMode);
  }
  if (edge_code >= USBIO_EDGE_COUNT) {
    stall(Status::BadValue);
  }
  const auto edge = static_cast<EdgeMode>(edge_code);
  const auto debounce_ms = static_cast<std::uint8_t>((value >> 8) & 0xFFu);

  std::lock_guard<std::mutex> lock(_mutex);
  const auto it =
      std::find_if(_event_watch.begin(), _event_watch.end(),
                  [pin](const EventWatch &w) { return w.pin == pin; });
  const bool already_watched = it != _event_watch.end();
  if (!already_watched && _event_watch.size() >= _board.event_max_pins) {
    stall(Status::BadValue);
  }
  if (already_watched) {
    it->edge = edge;
    it->debounce_ms = debounce_ms;
    it->count = 0;
    it->last_accept_ms.reset();
    // Re-arming replaces mode/debounce and resets the counter, but does not
    // discard already-queued events for the pin (only EdgeMode::Off does).
  } else {
    _event_watch.push_back({pin, edge, debounce_ms, 0, std::nullopt});
  }
}

std::vector<LoggedRequest> FakeTransport::log() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _log;
}

LoggedRequest FakeTransport::last() const {
  std::lock_guard<std::mutex> lock(_mutex);
  if (_log.empty()) {
    throw std::logic_error("FakeTransport: the request log is empty");
  }
  return _log.back();
}

std::size_t FakeTransport::count(Request request) const {
  const auto code = static_cast<std::uint8_t>(request);
  std::lock_guard<std::mutex> lock(_mutex);
  return static_cast<std::size_t>(
      std::count_if(_log.begin(), _log.end(), [code](const LoggedRequest &r) {
        return r.request == code;
      }));
}

void FakeTransport::clear_log() {
  std::lock_guard<std::mutex> lock(_mutex);
  _log.clear();
}

// ---- Internals --------------------------------------------------------------

void FakeTransport::stall(Status reason) {
  _last_error = reason;
  throw StallError("FakeTransport: request STALLed (" +
                   std::string(to_string(reason)) + ")");
}

bool FakeTransport::take_busy() noexcept {
  if (_busy_reads == 0) {
    return false;
  }
  --_busy_reads;
  return true;
}

std::size_t FakeTransport::encode_info(std::span<std::byte> out) {
  // Before begin() the firmware knows nothing about its pins.
  const bool ready = _not_ready == 0;
  if (!ready) {
    --_not_ready;
  }
  for (std::size_t i = 0; i < _magic.size(); ++i) {
    out[InfoOffset::Magic + i] = static_cast<std::byte>(_magic[i]);
  }
  write_u16le(out, InfoOffset::ProtocolVersion, _protocol_version);
  write_u16le(out, InfoOffset::BoardId,
              static_cast<std::uint16_t>(_board.board_id));
  write_u8(out, InfoOffset::NPins, ready ? _board.n_pins() : 0);
  write_u8(out, InfoOffset::NAin, ready ? _board.n_ain() : 0);
  write_u8(out, InfoOffset::AdcBits, _board.adc_bits);
  write_u8(out, InfoOffset::PwmBits, _board.pwm_bits);
  write_u8(out, InfoOffset::DacBits, _board.dac_bits);
  write_u8(out, InfoOffset::QueueDepth, _board.queue_depth);
  write_u16le(out, InfoOffset::VrefMv, _board.vref_mv);
  write_u16le(out, InfoOffset::IoMv, _board.io_mv);
  write_u16le(out, InfoOffset::Flags, _board.flags);
  write_u8(out, InfoOffset::StreamMaxChannels, _board.stream_max_channels);
  write_u8(out, InfoOffset::EventMaxPins, _board.event_max_pins);
  for (std::size_t i = 0; i < 2; ++i) {
    write_u8(out, InfoOffset::Reserved + i, 0);
  }
  return InfoLen;
}

// ---- Streaming ----------------------------------------------------------

void FakeTransport::handle_stream_select(std::uint16_t value,
                                         std::uint16_t index) {
  // Order per usbio_protocol.h: stream stopped, pin range, mode, value,
  // channel-count limit.
  if (_stream_running) {
    stall(Status::Busy);
  }
  if (index >= _board.n_pins()) {
    stall(Status::BadPin);
  }
  if (!(is_input_mode(_mode[index]) || _mode[index] == PinMode::AnalogIn)) {
    stall(Status::BadMode);
  }
  if (value > 1) {
    stall(Status::BadValue);
  }
  const auto pin = static_cast<std::uint8_t>(index);
  const auto it =
      std::find(_stream_selected.begin(), _stream_selected.end(), pin);
  const bool already_selected = it != _stream_selected.end();
  if (value != 0) { // add
    if (!already_selected) {
      if (_stream_selected.size() >= _board.stream_max_channels) {
        stall(Status::BadValue);
      }
      _stream_selected.push_back(pin);
    } // else: no-op, already selected
  } else if (already_selected) { // remove
    _stream_selected.erase(it);
  } // else: no-op, was not selected
}

void FakeTransport::handle_stream_start(std::uint16_t value,
                                        std::uint16_t index) {
  // wValue = period_us, wIndex = flags (usbio_stream_flags).
  if (_stream_running) {
    stall(Status::Busy);
  }
  if (_stream_selected.empty()) {
    stall(Status::BadValue);
  }
  if (value != 0 && value < StreamMinPeriodUs) {
    stall(Status::BadValue);
  }
  constexpr auto known_flags = static_cast<std::uint16_t>(
      USBIO_STREAM_FLAG_DIGITAL | USBIO_STREAM_FLAG_STOP_ON_OVERRUN);
  if ((index & ~known_flags) != 0) {
    stall(Status::BadValue);
  }
  _stream_flags = static_cast<std::uint8_t>(index);
  _stream_period_us = value;
  _stream_seq = 0;
  _stream_overruns = 0;
  _stream_running = true;
}

std::size_t FakeTransport::bulk_in(std::span<std::byte> data,
                                   std::chrono::milliseconds timeout) {
  if ((_board.flags & USBIO_FLAG_STREAMING) == 0) {
    throw NotSupported(
        "FakeTransport: board has no bulk IN endpoint (no USBIO_FLAG_STREAMING)");
  }
  std::unique_lock<std::mutex> lock(_mutex);
  if (_bulk_queue.empty() && _bulk_chunks.empty()) {
    // Idle: behave like a real endpoint with nothing to deliver. A Stream's
    // worker calls this in a tight loop, so wait a little (not the full
    // timeout -- that would make every test that starts and promptly stops
    // a stream pay for it) to avoid spinning the CPU while polling.
    lock.unlock();
    std::this_thread::sleep_for(std::min(timeout, std::chrono::milliseconds{5}));
    return 0;
  }
  std::size_t n = std::min(_bulk_queue.size(), data.size());
  if (!_bulk_chunks.empty()) {
    n = std::min(n, _bulk_chunks.front());
    _bulk_chunks.pop_front();
  }
  std::copy_n(_bulk_queue.begin(), n, data.begin());
  _bulk_queue.erase(_bulk_queue.begin(),
                    _bulk_queue.begin() + static_cast<std::ptrdiff_t>(n));
  return n;
}

void FakeTransport::set_stream_ramp(std::uint16_t start, std::uint16_t step,
                                    std::uint32_t t0_us,
                                    std::uint32_t dt_us) noexcept {
  _ramp_start = start;
  _ramp_step = step;
  _ramp_t0_us = t0_us;
  _ramp_dt_us = dt_us;
  _ramp_record_index = 0;
}

void FakeTransport::queue_stream_records(
    std::size_t count, std::uint32_t seq_step,
    std::initializer_list<std::size_t> chunk_plan) {
  const bool digital = (_stream_flags & USBIO_STREAM_FLAG_DIGITAL) != 0;
  const auto n_samples = static_cast<std::uint16_t>(_stream_selected.size());
  const std::size_t record_len =
      stream_record_len(n_samples, digital, _board.n_pins());
  std::vector<std::byte> record(record_len);
  std::lock_guard<std::mutex> lock(_mutex);
  for (std::size_t r = 0; r < count; ++r) {
    const StreamHeader header{
        StreamMagic, n_samples, _stream_seq,
        _ramp_t0_us + _ramp_dt_us * _ramp_record_index};
    encode_stream_header(record, header);
    for (std::uint16_t i = 0; i < n_samples; ++i) {
      const auto raw = static_cast<std::uint16_t>(
          _ramp_start +
          _ramp_step * (_ramp_record_index * n_samples + i));
      write_u16le(record, StreamHeaderLen + 2 * static_cast<std::size_t>(i),
                 raw);
    }
    if (digital) {
      std::fill(record.begin() +
                   static_cast<std::ptrdiff_t>(StreamHeaderLen + 2 * n_samples),
               record.end(), std::byte{0});
    }
    _bulk_queue.insert(_bulk_queue.end(), record.begin(), record.end());
    _stream_seq += seq_step;
    ++_ramp_record_index;
  }
  for (const std::size_t n : chunk_plan) {
    _bulk_chunks.push_back(n);
  }
}

void FakeTransport::queue_bulk_garbage(std::size_t n) {
  std::lock_guard<std::mutex> lock(_mutex);
  for (std::size_t i = 0; i < n; ++i) {
    _bulk_queue.push_back(static_cast<std::byte>(0xAAu));
  }
}

void FakeTransport::queue_bulk_bytes(std::span<const std::byte> bytes) {
  std::lock_guard<std::mutex> lock(_mutex);
  _bulk_queue.insert(_bulk_queue.end(), bytes.begin(), bytes.end());
}

void FakeTransport::queue_bulk_chunk(std::size_t n) {
  std::lock_guard<std::mutex> lock(_mutex);
  _bulk_chunks.push_back(n);
}

std::uint32_t FakeTransport::stream_seq() const noexcept {
  std::lock_guard<std::mutex> lock(_mutex);
  return _stream_seq;
}

void FakeTransport::set_stream_overruns(std::uint32_t n) noexcept {
  std::lock_guard<std::mutex> lock(_mutex);
  _stream_overruns = n;
}

std::size_t FakeTransport::bulk_queue_size() const noexcept {
  std::lock_guard<std::mutex> lock(_mutex);
  return _bulk_queue.size();
}

} // namespace ArduinoDriver::Testing
