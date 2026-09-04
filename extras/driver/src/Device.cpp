// Device.cpp - local validation, request encoding, reply decoding, BUSY and
// not-ready retry, STALL translation.
#include "arduino_driver/Device.h"

#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace ArduinoDriver {

namespace {

constexpr std::uint8_t request_code(Request request) noexcept {
  return static_cast<std::uint8_t>(request);
}

/// "DIO_WRITE (pin 2)" for pin-addressed requests, "RESET" for the others.
std::string describe_request(Request request, std::uint16_t index) {
  switch (request) {
  case Request::GetInfo:
  case Request::DioReadAll:
  case Request::AiReadAll:
  case Request::GetStatus:
  case Request::StreamStop:
  case Request::StreamStatus:
  case Request::GetTime:
  case Request::EventCounts:
  case Request::Reset:
    return std::string(to_string(request));
  case Request::StreamStart:
    return fmt::format("{} (flags 0x{:02X})", to_string(request), index);
  case Request::EventPop:
    return fmt::format("{} (max {})", to_string(request), index);
  default:
    return fmt::format("{} (pin {})", to_string(request), index);
  }
}

/// Maps a rejection reported by the device to the matching exception.
[[noreturn]] void throw_for_status(Status status, const std::string &context) {
  const std::string message =
      fmt::format("{} rejected by the device: {} ({})", context,
                  to_string(status), describe(status));
  switch (status) {
  case Status::BadPin:
    throw InvalidPin(message);
  case Status::BadMode:
    throw InvalidMode(message);
  case Status::Unsupported:
    throw NotSupported(message);
  case Status::BadValue:
    throw InvalidValue(message);
  case Status::QueueFull:
    throw QueueFull(message);
  case Status::Busy:
    throw DeviceBusy(message);
  case Status::BadCmd:
    throw ProtocolError(message);
  case Status::Ok:
    break;
  }
  throw ProtocolError(fmt::format("{}: unexpected status code {}", context,
                                  static_cast<unsigned>(status)));
}

/// fraction (0..1) -> nearest code in 0..full_scale.
std::uint16_t scale_to_code(double fraction, std::uint16_t full_scale) {
  const long code = std::lround(fraction * static_cast<double>(full_scale));
  return static_cast<std::uint16_t>(
      std::clamp<long>(code, 0, static_cast<long>(full_scale)));
}

void pause(std::chrono::microseconds delay) {
  if (delay > std::chrono::microseconds::zero()) {
    std::this_thread::sleep_for(delay);
  }
}

} // namespace

// ---- Construction -----------------------------------------------------------

Device::Device(std::unique_ptr<Transport> transport, Options options)
    : _transport(std::move(transport)), _options(options) {
  if (!_transport) {
    throw std::invalid_argument("Device: transport must not be null");
  }
  load_info();
  load_caps();
}

void Device::load_info() {
  const unsigned attempts = std::max(1u, _options.ready_max_attempts);
  for (unsigned attempt = 1;; ++attempt) {
    std::array<std::byte, InfoLen> reply{};
    const std::size_t n = raw_in(Request::GetInfo, 0, reply);
    if (n < InfoLen) {
      throw ProtocolError(
          fmt::format("GET_INFO: short reply ({} of {} bytes)", n, InfoLen));
    }
    _info = decode_info(reply);
    if (_info.protocol_version != ProtocolVersion) {
      throw ProtocolError(fmt::format("unsupported protocol version 0x{:04X} "
                                      "(this driver speaks 0x{:04X})",
                                      _info.protocol_version, ProtocolVersion));
    }
    if (_info.n_pins != 0) {
      break;
    }
    // USB enumerated before the sketch reached UsbIo.begin(): wait for it.
    if (attempt >= attempts) {
      throw NotReady(fmt::format(
          "device enumerated but the sketch has not called UsbIo.begin() "
          "(GET_INFO reported n_pins == 0 in {} attempts over {} ms)",
          attempts, (attempts - 1) * _options.ready_delay.count()));
    }
    pause(_options.ready_delay);
  }
  if (_info.n_pins > MaxPins) {
    throw ProtocolError(
        fmt::format("GET_INFO: n_pins = {} exceeds {}", _info.n_pins, MaxPins));
  }
  if (_info.n_ain > MaxAin || _info.n_ain > _info.n_pins) {
    throw ProtocolError(fmt::format(
        "GET_INFO: n_ain = {} is inconsistent (n_pins = {}, limit {})",
        _info.n_ain, _info.n_pins, MaxAin));
  }
}

void Device::load_caps() {
  // wIndex = first pin (0), wLength = n_pins: the device clamps the reply to
  // min(wLength, n_pins - first), so a complete table is exactly n_pins bytes.
  std::array<std::byte, MaxPins> buffer{};
  const std::span<std::byte> reply(buffer.data(), _info.n_pins);
  const std::size_t n = raw_in(Request::GetPinCaps, 0, reply);
  if (n < _info.n_pins) {
    throw ProtocolError(fmt::format(
        "GET_PIN_CAPS: short reply ({} of {} bytes)", n, _info.n_pins));
  }
  _caps.resize(_info.n_pins);
  _analog_pins.clear();
  for (std::size_t pin = 0; pin < _caps.size(); ++pin) {
    _caps[pin].bits = read_u8(reply, pin);
    if (_caps[pin].ain()) {
      _analog_pins.push_back(static_cast<std::uint8_t>(pin));
    }
  }
  if (_analog_pins.size() != _info.n_ain) {
    throw ProtocolError(fmt::format(
        "GET_PIN_CAPS: {} pins carry the AIN capability but GET_INFO reports "
        "n_ain = {}",
        _analog_pins.size(), _info.n_ain));
  }
}

// ---- Static information -----------------------------------------------------

PinCaps Device::pin_caps(std::uint8_t pin) const {
  if (pin >= _caps.size()) {
    throw InvalidPin(fmt::format(
        "pin {} is out of range: the {} has {} pins (0..{})", pin,
        board_name(_info.board_id), _caps.size(), _caps.size() - 1));
  }
  return _caps[pin];
}

PinCaps Device::require_caps(std::uint8_t pin, std::uint8_t cap_mask,
                             std::string_view feature) const {
  const PinCaps caps = pin_caps(pin);
  if (!caps.has(cap_mask)) {
    throw NotSupported(fmt::format("pin {} of the {} has no {} capability", pin,
                                   board_name(_info.board_id), feature));
  }
  return caps;
}

// ---- Configuration ----------------------------------------------------------

void Device::pin_mode(std::uint8_t pin, PinMode mode) {
  check_not_streaming("pin_mode");
  const PinCaps caps = pin_caps(pin);
  const auto code = static_cast<std::uint8_t>(mode);
  if (code >= PinModeCount) {
    throw InvalidMode(fmt::format("unknown pin mode {}", code));
  }
  if (!caps.has(required_caps(mode))) {
    throw NotSupported(fmt::format("pin {} of the {} does not support mode {}",
                                   pin, board_name(_info.board_id),
                                   to_string(mode)));
  }
  if (mode == PinMode::InputPulldown && !_info.supports_pulldown()) {
    throw NotSupported(fmt::format("the {} does not support INPUT_PULLDOWN",
                                   board_name(_info.board_id)));
  }
  send_out(Request::PinMode, code, pin);
}

// ---- Digital I/O ------------------------------------------------------------

void Device::digital_write(std::uint8_t pin, bool high) {
  check_not_streaming("digital_write");
  require_caps(pin, PinCaps::Dio, "digital I/O");
  send_out(Request::DioWrite, static_cast<std::uint16_t>(high ? 1 : 0), pin);
}

bool Device::digital_read(std::uint8_t pin) {
  check_not_streaming("digital_read");
  require_caps(pin, PinCaps::Dio, "digital I/O");
  std::array<std::byte, DioReplyLen> reply{};
  read_in(Request::DioRead, pin, reply, DioReplyLen);
  return read_u8(reply, 1) != 0;
}

std::vector<bool> Device::read_all_digital() {
  check_not_streaming("read_all_digital");
  std::array<std::byte, dio_read_all_len(MaxPins)> buffer{};
  const std::size_t len = dio_read_all_len(_info.n_pins);
  const std::span<std::byte> reply(buffer.data(), len);
  read_in(Request::DioReadAll, 0, reply, len);
  std::vector<bool> values(_info.n_pins);
  for (std::size_t pin = 0; pin < values.size(); ++pin) {
    const std::uint8_t packed = read_u8(reply, AllHeaderLen + pin / 8);
    values[pin] = ((packed >> (pin % 8)) & 1u) != 0;
  }
  return values;
}

// ---- Analog input -----------------------------------------------------------

std::uint16_t Device::analog_read(std::uint8_t pin) {
  check_not_streaming("analog_read");
  require_caps(pin, PinCaps::Ain, "analog input");
  std::array<std::byte, AiReplyLen> reply{};
  read_in(Request::AiRead, pin, reply, AiReplyLen);
  return read_u16le(reply, 2);
}

double Device::analog_read_volts(std::uint8_t pin) {
  return to_volts(analog_read(pin));
}

double Device::to_volts(std::uint16_t raw) const noexcept {
  const std::uint16_t full_scale = max_value(_info.adc_bits);
  if (full_scale == 0) {
    return 0.0;
  }
  return static_cast<double>(raw) * _info.vref_mv / 1000.0 / full_scale;
}

std::vector<std::uint16_t> Device::read_all_analog() {
  check_not_streaming("read_all_analog");
  std::array<std::byte, ai_read_all_len(MaxAin)> buffer{};
  const std::size_t len = ai_read_all_len(_info.n_ain);
  const std::span<std::byte> reply(buffer.data(), len);
  read_in(Request::AiReadAll, 0, reply, len);
  std::vector<std::uint16_t> samples(_info.n_ain);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    samples[i] = read_u16le(reply, AllHeaderLen + 2 * i);
  }
  return samples;
}

// ---- PWM / DAC output -------------------------------------------------------

void Device::pwm_write(std::uint8_t pin, std::uint16_t duty) {
  check_not_streaming("pwm_write");
  require_caps(pin, PinCaps::Pwm, "PWM");
  const std::uint16_t full_scale = max_value(_info.pwm_bits);
  if (duty > full_scale) {
    throw InvalidValue(fmt::format("PWM duty {} exceeds the {}-bit range 0..{}",
                                   duty, _info.pwm_bits, full_scale));
  }
  send_out(Request::PwmWrite, duty, pin);
}

void Device::pwm_write_fraction(std::uint8_t pin, double fraction) {
  require_caps(pin, PinCaps::Pwm, "PWM");
  if (!(fraction >= 0.0 && fraction <= 1.0)) { // also rejects NaN
    throw InvalidValue(
        fmt::format("PWM duty fraction {} is outside 0..1", fraction));
  }
  pwm_write(pin, scale_to_code(fraction, max_value(_info.pwm_bits)));
}

void Device::dac_write(std::uint8_t pin, std::uint16_t value) {
  check_not_streaming("dac_write");
  require_caps(pin, PinCaps::Dac, "DAC");
  if (!_info.has_dac()) {
    throw NotSupported(fmt::format("the {} reports no DAC resolution",
                                   board_name(_info.board_id)));
  }
  const std::uint16_t full_scale = max_value(_info.dac_bits);
  if (value > full_scale) {
    throw InvalidValue(
        fmt::format("DAC value {} exceeds the {}-bit range 0..{}", value,
                    _info.dac_bits, full_scale));
  }
  send_out(Request::DacWrite, value, pin);
}

void Device::dac_write_volts(std::uint8_t pin, double volts) {
  require_caps(pin, PinCaps::Dac, "DAC");
  if (!_info.has_dac() || _info.vref_mv == 0) {
    throw NotSupported(fmt::format("the {} reports no DAC reference voltage",
                                   board_name(_info.board_id)));
  }
  const double vref = _info.vref_mv / 1000.0;
  if (!(volts >= 0.0 && volts <= vref)) { // also rejects NaN
    throw InvalidValue(
        fmt::format("DAC voltage {} V is outside 0..{} V", volts, vref));
  }
  dac_write(pin, scale_to_code(volts / vref, max_value(_info.dac_bits)));
}

// ---- Control ----------------------------------------------------------------

Status Device::status(std::uint8_t *queue_pending) {
  check_not_streaming("status");
  std::array<std::byte, StatusReplyLen> reply{};
  read_in(Request::GetStatus, 0, reply, StatusReplyLen);
  if (queue_pending != nullptr) {
    *queue_pending = read_u8(reply, 1);
  }
  return static_cast<Status>(read_u8(reply, 2));
}

void Device::sync() {
  const unsigned attempts = std::max(1u, _options.busy_max_attempts);
  for (unsigned attempt = 1;; ++attempt) {
    std::uint8_t pending = 0;
    status(&pending);
    if (pending == 0) {
      return;
    }
    if (attempt >= attempts) {
      throw DeviceBusy(fmt::format(
          "sync: {} commands still pending after {} GET_STATUS polls", pending,
          attempts));
    }
    pause(_options.busy_delay);
  }
}

void Device::reset() {
  check_not_streaming("reset");
  send_out(Request::Reset, 0, 0);
  _stream->selected.clear();
}

// ---- Streaming (Phase 2) -----------------------------------------------------

void Device::check_not_streaming(std::string_view what) const {
  if (_stream->streaming.load(std::memory_order_acquire)) {
    throw DeviceBusy(
        fmt::format("{}: a Stream is running on this device; only "
                    "Stream::stats() and Stream::stop() may be used until "
                    "it stops",
                    what));
  }
}

Stream Device::start_stream(StreamConfig config) {
  check_not_streaming("start_stream");
  if (!_info.streaming()) {
    throw NotSupported(
        fmt::format("the {} firmware does not report USBIO_FLAG_STREAMING",
                    board_name(_info.board_id)));
  }
  if (config.pins.empty()) {
    throw InvalidValue("start_stream: the pin list must not be empty");
  }
  if (config.pins.size() > _info.stream_max_channels) {
    throw InvalidValue(fmt::format(
        "start_stream: {} pins exceeds stream_max_channels ({})",
        config.pins.size(), _info.stream_max_channels));
  }
  for (const std::uint8_t pin : config.pins) {
    pin_caps(pin); // range check only (InvalidPin); mode is device-checked
  }
  {
    // A duplicate would make STREAM_SELECT a no-op for the repeat (the
    // device only ever adds a pin once), so the device's real channel
    // count would fall short of config.pins.size() and every record would
    // permanently fail Stream's n_samples check.
    std::vector<std::uint8_t> sorted = config.pins;
    std::sort(sorted.begin(), sorted.end());
    if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end()) {
      throw InvalidValue("start_stream: the pin list must not repeat a pin");
    }
  }
  const auto period_raw = config.period.count();
  if (period_raw < 0 || period_raw > 0xFFFF ||
      (period_raw != 0 && period_raw < StreamMinPeriodUs)) {
    throw InvalidValue(
        fmt::format("start_stream: period {} us is outside 0 (free running) "
                    "or {}..65535",
                    period_raw, StreamMinPeriodUs));
  }
  const auto period_us = static_cast<std::uint16_t>(period_raw);

  std::lock_guard<std::mutex> lock(_stream->mutex);
  // STREAM_STOP keeps the selection: drop pins an earlier stream on this
  // Device selected but the new configuration does not want.
  for (const std::uint8_t pin : _stream->selected) {
    if (std::find(config.pins.begin(), config.pins.end(), pin) ==
        config.pins.end()) {
      try {
        send_out(Request::StreamSelect, 0, pin);
      } catch (const Error &) {
        // best effort; a stale pin here should not block starting the
        // stream the caller actually asked for
      }
    }
  }
  std::vector<std::uint8_t> added;
  try {
    for (const std::uint8_t pin : config.pins) {
      send_out(Request::StreamSelect, 1, pin);
      added.push_back(pin);
    }
    send_out(Request::StreamStart, period_us, config.flags);
  } catch (...) {
    for (auto it = added.rbegin(); it != added.rend(); ++it) {
      try {
        send_out(Request::StreamSelect, 0, *it);
      } catch (...) {
        // best effort rollback: the original failure is what matters
      }
    }
    throw;
  }
  _stream->selected = config.pins;
  _stream->streaming.store(true, std::memory_order_release);
  return Stream(*this, std::move(config));
}

StreamStatus Device::poll_stream_status() {
  std::lock_guard<std::mutex> lock(_stream->mutex);
  std::array<std::byte, StreamStatusLen> reply{};
  const std::size_t n = raw_in(Request::StreamStatus, 0, reply);
  if (n < StreamStatusLen) {
    throw ProtocolError(fmt::format(
        "GET_STREAM_STATUS: short reply ({} of {} bytes)", n, StreamStatusLen));
  }
  return decode_stream_status(reply);
}

void Device::end_stream() noexcept {
  std::lock_guard<std::mutex> lock(_stream->mutex);
  if (!_stream->streaming.load(std::memory_order_acquire)) {
    return;
  }
  try {
    send_out(Request::StreamStop, 0, 0);
  } catch (...) {
    // best effort: this runs from Stream's destructor and must not throw
  }
  _stream->streaming.store(false, std::memory_order_release);
}

// ---- Device time --------------------------------------------------------

DeviceTime Device::read_time() {
  check_not_streaming("read_time");
  const auto t0 = std::chrono::steady_clock::now();
  std::array<std::byte, TimeReplyLen> reply{};
  read_in(Request::GetTime, 0, reply, TimeReplyLen);
  const auto t1 = std::chrono::steady_clock::now();
  const TimeReply parsed = decode_time_reply(reply);

  DeviceTime result;
  result.millis = parsed.millis;
  result.micros = parsed.micros;
  result.micros64 = reconstruct_micros64(parsed.millis, parsed.micros);
  result.round_trip = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);
  result.host_time = t0 + (t1 - t0) / 2;
  return result;
}

// ---- Pin events -----------------------------------------------------------

void Device::require_events(std::string_view what) const {
  if (!_info.events()) {
    throw NotSupported(
        fmt::format("{}: the {} firmware does not report USBIO_FLAG_EVENTS",
                    what, board_name(_info.board_id)));
  }
}

void Device::configure_event(std::uint8_t pin, EdgeMode edge,
                             std::chrono::milliseconds debounce) {
  check_not_streaming("configure_event");
  require_events("configure_event");
  pin_caps(pin); // range check only (InvalidPin); DIO + INPUT* mode is
                // device-checked, like STREAM_SELECT's pin (see start_stream)
  const auto edge_code = static_cast<std::uint8_t>(edge);
  if (edge_code >= EdgeModeCount) {
    throw InvalidValue(fmt::format("configure_event: unknown edge mode {}",
                                   static_cast<unsigned>(edge_code)));
  }
  if (debounce.count() < 0 ||
      debounce.count() > static_cast<std::chrono::milliseconds::rep>(MaxDebounceMs)) {
    throw InvalidValue(fmt::format(
        "configure_event: debounce {} ms is outside 0..{}", debounce.count(),
        static_cast<unsigned>(MaxDebounceMs)));
  }
  const auto value =
      encode_event_config_value(static_cast<std::uint8_t>(debounce.count()), edge);
  send_out(Request::EventConfig, value, pin);
}

void Device::pop_events_once(std::uint16_t max_events,
                             std::vector<PinEvent> &out, std::uint8_t *dropped,
                             bool *pending) {
  std::array<std::byte, event_pop_len(MaxEventsPerPop)> reply{};
  const std::size_t n =
      read_in(Request::EventPop, max_events, reply, EventHeaderLen);
  const EventHeader header = decode_event_header(reply);
  // header.count entries are expected to follow; a reply too short to hold
  // them all (truncation, or -- since header.count is attacker/bug-facing
  // wire data -- a bogus count) is a protocol error, checked before any
  // decode_event() call would read past what the transport actually
  // delivered.
  const std::size_t expected = event_pop_len(header.count);
  if (n < expected) {
    throw ProtocolError(fmt::format(
        "EVENT_POP: short reply ({} of {} bytes for {} events)", n, expected,
        static_cast<unsigned>(header.count)));
  }
  for (std::size_t i = 0; i < header.count; ++i) {
    out.push_back(decode_event(reply, i));
  }
  if (dropped != nullptr) {
    *dropped = header.dropped;
  }
  if (pending != nullptr) {
    *pending = header.pending != 0;
  }
}

std::vector<PinEvent> Device::poll_events(std::uint8_t *dropped) {
  check_not_streaming("poll_events");
  require_events("poll_events");
  std::vector<PinEvent> events;
  unsigned total_dropped = 0;
  bool pending = true;
  while (pending) {
    std::uint8_t batch_dropped = 0;
    pop_events_once(0, events, &batch_dropped, &pending);
    total_dropped = std::min(255u, total_dropped + batch_dropped);
  }
  if (dropped != nullptr) {
    *dropped = static_cast<std::uint8_t>(total_dropped);
  }
  return events;
}

std::vector<EventCount> Device::event_counts() {
  check_not_streaming("event_counts");
  require_events("event_counts");
  std::array<std::byte, event_counts_len(MaxEventPins)> reply{};
  const std::size_t n =
      read_in(Request::EventCounts, 0, reply, EventHeaderLen);
  const EventHeader header = decode_event_header(reply);
  const std::size_t expected = event_counts_len(header.count);
  if (n < expected) {
    throw ProtocolError(fmt::format(
        "EVENT_COUNTS: short reply ({} of {} bytes for {} pins)", n, expected,
        static_cast<unsigned>(header.count)));
  }
  std::vector<EventCount> result;
  result.reserve(header.count);
  for (std::size_t i = 0; i < header.count; ++i) {
    result.push_back(decode_event_count(reply, i));
  }
  return result;
}

std::optional<PinEvent> Device::wait_event(std::chrono::milliseconds timeout,
                                           std::chrono::milliseconds poll_interval) {
  check_not_streaming("wait_event");
  require_events("wait_event");
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  for (;;) {
    std::vector<PinEvent> single;
    pop_events_once(1, single, nullptr, nullptr);
    if (!single.empty()) {
      return single.front();
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return std::nullopt;
    }
    pause(std::chrono::duration_cast<std::chrono::microseconds>(
        std::min(poll_interval,
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    deadline - now))));
  }
}

// ---- Transfers --------------------------------------------------------------

void Device::send_out(Request request, std::uint16_t value,
                      std::uint16_t index) {
  try {
    {
      // Locked around exactly this one wire transfer -- see the threading
      // contract in Device.h. Released before the GET_STATUS follow-up
      // below, which takes the same lock again through raw_in().
      std::lock_guard<std::mutex> io_lock(_io->mutex);
      _transport->control_out(request_code(request), value, index,
                              _options.timeout);
    }
  } catch (const StallError &) {
    // The firmware rejected the request; GET_STATUS.last_error tells why.
    std::array<std::byte, StatusReplyLen> reply{};
    const std::size_t n = raw_in(Request::GetStatus, 0, reply);
    if (n < StatusReplyLen) {
      throw ProtocolError(fmt::format(
          "{} was rejected (STALL) and GET_STATUS answered {} of {} bytes",
          describe_request(request, index), n, StatusReplyLen));
    }
    const auto reason = static_cast<Status>(read_u8(reply, 2));
    if (reason == Status::Ok) {
      throw; // nothing recorded on the device: surface the raw STALL
    }
    throw_for_status(reason, describe_request(request, index));
  }
}

std::size_t Device::read_in(Request request, std::uint16_t index,
                            std::span<std::byte> reply, std::size_t min_len) {
  const unsigned attempts = std::max(1u, _options.busy_max_attempts);
  for (unsigned attempt = 1;; ++attempt) {
    const std::size_t n = raw_in(request, index, reply);
    if (n == 0) {
      throw ProtocolError(
          fmt::format("{}: empty reply", describe_request(request, index)));
    }
    const auto status = static_cast<Status>(read_u8(reply, 0));
    if (status == Status::Busy) {
      if (attempt >= attempts) {
        throw DeviceBusy(fmt::format("{}: device still busy after {} attempts",
                                     describe_request(request, index),
                                     attempts));
      }
      pause(_options.busy_delay);
      continue;
    }
    if (status != Status::Ok) {
      throw_for_status(status, describe_request(request, index));
    }
    if (n < min_len) {
      throw ProtocolError(fmt::format("{}: short reply ({} of {} bytes)",
                                      describe_request(request, index), n,
                                      min_len));
    }
    return n;
  }
}

std::size_t Device::raw_in(Request request, std::uint16_t index,
                           std::span<std::byte> reply) {
  std::size_t n = 0;
  {
    // Locked around exactly this one wire transfer -- see the threading
    // contract in Device.h.
    std::lock_guard<std::mutex> io_lock(_io->mutex);
    n = _transport->control_in(request_code(request), 0, index, reply,
                               _options.timeout);
  }
  if (n > reply.size()) {
    throw ProtocolError(
        fmt::format("{}: transport returned {} bytes for a {}-byte request",
                    describe_request(request, index), n, reply.size()));
  }
  return n;
}

} // namespace ArduinoDriver
