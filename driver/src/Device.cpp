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
  case Request::Reset:
    return std::string(to_string(request));
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
  require_caps(pin, PinCaps::Dio, "digital I/O");
  send_out(Request::DioWrite, static_cast<std::uint16_t>(high ? 1 : 0), pin);
}

bool Device::digital_read(std::uint8_t pin) {
  require_caps(pin, PinCaps::Dio, "digital I/O");
  std::array<std::byte, DioReplyLen> reply{};
  read_in(Request::DioRead, pin, reply, DioReplyLen);
  return read_u8(reply, 1) != 0;
}

std::vector<bool> Device::read_all_digital() {
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

void Device::reset() { send_out(Request::Reset, 0, 0); }

// ---- Transfers --------------------------------------------------------------

void Device::send_out(Request request, std::uint16_t value,
                      std::uint16_t index) {
  try {
    _transport->control_out(request_code(request), value, index,
                            _options.timeout);
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
  const std::size_t n = _transport->control_in(request_code(request), 0, index,
                                               reply, _options.timeout);
  if (n > reply.size()) {
    throw ProtocolError(
        fmt::format("{}: transport returned {} bytes for a {}-byte request",
                    describe_request(request, index), n, reply.size()));
  }
  return n;
}

} // namespace ArduinoDriver
