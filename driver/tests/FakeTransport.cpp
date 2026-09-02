// FakeTransport.cpp - see FakeTransport.h.
#include "FakeTransport.h"

#include <algorithm>
#include <stdexcept>
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
  _log.push_back(
      {USBIO_REQTYPE_IN, request, value, index, clamp_u16(data.size())});

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
  _log.push_back({USBIO_REQTYPE_OUT, request, value, index, 0});

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
    // analog-only pads back to unconfigured, analog shadow cleared.
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
  _dio[pin] = high ? 1 : 0;
}

void FakeTransport::set_analog(std::uint8_t pin, std::uint16_t raw) {
  check_pin(pin);
  _ain[pin] = raw;
}

const LoggedRequest &FakeTransport::last() const {
  if (_log.empty()) {
    throw std::logic_error("FakeTransport: the request log is empty");
  }
  return _log.back();
}

std::size_t FakeTransport::count(Request request) const noexcept {
  const auto code = static_cast<std::uint8_t>(request);
  return static_cast<std::size_t>(
      std::count_if(_log.begin(), _log.end(), [code](const LoggedRequest &r) {
        return r.request == code;
      }));
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
  for (std::size_t i = 0; i < 4; ++i) {
    write_u8(out, InfoOffset::Reserved + i, 0);
  }
  return InfoLen;
}

} // namespace ArduinoDriver::Testing
