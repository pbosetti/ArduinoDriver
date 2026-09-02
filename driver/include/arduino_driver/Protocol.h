// Protocol.h - C++ view of usbio_protocol.h: scoped enums, capability bits,
// reply layouts, little-endian helpers and the GET_INFO decoder.
//
// usbio_protocol.h (shared with the firmware) stays the single source of
// truth; everything here is defined in terms of its constants.
#pragma once

#include "usbio_protocol.h"

#include "arduino_driver/Errors.h"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

namespace ArduinoDriver {

// ---- Codes ------------------------------------------------------------------

/// bRequest codes.
enum class Request : std::uint8_t {
  GetInfo = USBIO_REQ_GET_INFO,
  GetPinCaps = USBIO_REQ_GET_PIN_CAPS,
  PinMode = USBIO_REQ_PIN_MODE,
  DioRead = USBIO_REQ_DIO_READ,
  DioWrite = USBIO_REQ_DIO_WRITE,
  AiRead = USBIO_REQ_AI_READ,
  PwmWrite = USBIO_REQ_PWM_WRITE,
  DacWrite = USBIO_REQ_DAC_WRITE,
  DioReadAll = USBIO_REQ_DIO_READ_ALL,
  AiReadAll = USBIO_REQ_AI_READ_ALL,
  GetStatus = USBIO_REQ_GET_STATUS,
  Reset = USBIO_REQ_RESET,
};

/// Status byte of IN replies and GET_STATUS.last_error.
enum class Status : std::uint8_t {
  Ok = USBIO_OK,
  Busy = USBIO_BUSY,
  BadPin = USBIO_BAD_PIN,
  BadMode = USBIO_BAD_MODE,
  BadCmd = USBIO_BAD_CMD,
  Unsupported = USBIO_UNSUPPORTED,
  QueueFull = USBIO_QUEUE_FULL,
  BadValue = USBIO_BAD_VALUE,
};

/// wValue of PIN_MODE.
enum class PinMode : std::uint8_t {
  Input = USBIO_MODE_INPUT,
  Output = USBIO_MODE_OUTPUT,
  InputPullup = USBIO_MODE_INPUT_PULLUP,
  InputPulldown = USBIO_MODE_INPUT_PULLDOWN,
  AnalogIn = USBIO_MODE_ANALOG_IN,
  Pwm = USBIO_MODE_PWM,
  Dac = USBIO_MODE_DAC,
};
inline constexpr std::uint8_t PinModeCount = USBIO_MODE_COUNT;

/// Board identifier reported by GET_INFO (high byte = architecture family).
enum class BoardId : std::uint16_t {
  Unknown = USBIO_BOARD_UNKNOWN,
  UnoR4Minima = USBIO_BOARD_UNO_R4_MINIMA,
  NanoR4 = USBIO_BOARD_NANO_R4,
  RenesasGeneric = USBIO_BOARD_RENESAS_GENERIC,
  PortentaH7 = USBIO_BOARD_PORTENTA_H7,
  GigaR1 = USBIO_BOARD_GIGA_R1,
  Nano33Ble = USBIO_BOARD_NANO_33_BLE,
  MbedGeneric = USBIO_BOARD_MBED_GENERIC,
  Zero = USBIO_BOARD_ZERO,
  Mkr = USBIO_BOARD_MKR,
  Nano33Iot = USBIO_BOARD_NANO_33_IOT,
  SamdGeneric = USBIO_BOARD_SAMD_GENERIC,
  NanoRp2040Connect = USBIO_BOARD_NANO_RP2040_CONNECT,
  Rp2040Generic = USBIO_BOARD_RP2040_GENERIC,
  NanoEsp32 = USBIO_BOARD_NANO_ESP32,
  Esp32Generic = USBIO_BOARD_ESP32_GENERIC,
};

// ---- Sizes and limits -------------------------------------------------------

inline constexpr std::uint16_t ProtocolVersion = USBIO_PROTOCOL_VERSION;
inline constexpr std::string_view Magic{USBIO_MAGIC, USBIO_MAGIC_LEN};

inline constexpr std::size_t InfoLen = sizeof(usbio_info_t);            // 24
inline constexpr std::size_t DioReplyLen = sizeof(usbio_dio_reply_t);   // 2
inline constexpr std::size_t AiReplyLen = sizeof(usbio_ai_reply_t);     // 4
inline constexpr std::size_t AllHeaderLen = sizeof(usbio_all_header_t); // 2
inline constexpr std::size_t StatusReplyLen = sizeof(usbio_status_reply_t); // 4

inline constexpr std::size_t MaxPins = USBIO_MAX_PINS;
inline constexpr std::size_t MaxAin = USBIO_MAX_AIN;
inline constexpr std::size_t MaxReplyLen = USBIO_MAX_REPLY_LEN;
inline constexpr std::size_t QueueDepth = USBIO_QUEUE_DEPTH;

/// Byte offsets inside the GET_INFO reply (usbio_info_t, packed by natural
/// alignment: every uint16_t sits at an even offset).
namespace InfoOffset {
inline constexpr std::size_t Magic = 0;
inline constexpr std::size_t ProtocolVersion = 4;
inline constexpr std::size_t BoardId = 6;
inline constexpr std::size_t NPins = 8;
inline constexpr std::size_t NAin = 9;
inline constexpr std::size_t AdcBits = 10;
inline constexpr std::size_t PwmBits = 11;
inline constexpr std::size_t DacBits = 12;
inline constexpr std::size_t QueueDepth = 13;
inline constexpr std::size_t VrefMv = 14;
inline constexpr std::size_t IoMv = 16;
inline constexpr std::size_t Flags = 18;
inline constexpr std::size_t Reserved = 20;
} // namespace InfoOffset

/// Length of the DIO_READ_ALL bitmap for a board with `n_pins` pins.
constexpr std::size_t dio_bitmap_len(std::size_t n_pins) noexcept {
  return (n_pins + 7) / 8;
}
/// Total DIO_READ_ALL reply length (header + bitmap).
constexpr std::size_t dio_read_all_len(std::size_t n_pins) noexcept {
  return AllHeaderLen + dio_bitmap_len(n_pins);
}
/// Total AI_READ_ALL reply length (header + one uint16_t per analog pin).
constexpr std::size_t ai_read_all_len(std::size_t n_ain) noexcept {
  return AllHeaderLen + 2 * n_ain;
}
/// Largest value representable with `bits` bits (0 for bits == 0, capped at
/// 16 bits, the width of wValue and of the ADC samples).
constexpr std::uint16_t max_value(unsigned bits) noexcept {
  if (bits == 0) {
    return 0;
  }
  if (bits >= 16) {
    return 0xFFFFu;
  }
  return static_cast<std::uint16_t>((1u << bits) - 1u);
}

// ---- Requests ---------------------------------------------------------------

/// True for host->device requests (no data stage), false for IN requests.
constexpr bool is_out(Request request) noexcept {
  switch (request) {
  case Request::PinMode:
  case Request::DioWrite:
  case Request::PwmWrite:
  case Request::DacWrite:
  case Request::Reset:
    return true;
  case Request::GetInfo:
  case Request::GetPinCaps:
  case Request::DioRead:
  case Request::AiRead:
  case Request::DioReadAll:
  case Request::AiReadAll:
  case Request::GetStatus:
    return false;
  }
  return false;
}

/// Recipient field of bmRequestType. Device is the form every board accepts;
/// Interface routes the request through the dedicated vendor interface
/// (wIndex = (pin << 8) | interface_number) and is accepted only by boards
/// that expose one.
enum class Recipient : std::uint8_t { Device, Interface };

/// bmRequestType of a transfer in the given direction and recipient form.
constexpr std::uint8_t bm_request_type(bool out, Recipient recipient) noexcept {
  if (recipient == Recipient::Interface) {
    return out ? static_cast<std::uint8_t>(USBIO_REQTYPE_OUT_ITF)
               : static_cast<std::uint8_t>(USBIO_REQTYPE_IN_ITF);
  }
  return out ? static_cast<std::uint8_t>(USBIO_REQTYPE_OUT)
             : static_cast<std::uint8_t>(USBIO_REQTYPE_IN);
}

/// bmRequestType the transport must use for the request.
constexpr std::uint8_t
request_type(Request request,
             Recipient recipient = Recipient::Device) noexcept {
  return bm_request_type(is_out(request), recipient);
}

/// wIndex of the interface-recipient form: pin (or first pin) in the high
/// byte, interface number in the low byte.
constexpr std::uint16_t
interface_index(std::uint16_t index, std::uint8_t interface_number) noexcept {
  return static_cast<std::uint16_t>((index << 8) | interface_number);
}

// ---- Capabilities -----------------------------------------------------------

/// Capability byte of one pin (GET_PIN_CAPS).
struct PinCaps {
  static constexpr std::uint8_t Dio = USBIO_CAP_DIO;
  static constexpr std::uint8_t Ain = USBIO_CAP_AIN;
  static constexpr std::uint8_t Pwm = USBIO_CAP_PWM;
  static constexpr std::uint8_t Dac = USBIO_CAP_DAC;

  std::uint8_t bits{0};

  /// True when every bit of `mask` is set.
  constexpr bool has(std::uint8_t mask) const noexcept {
    return (bits & mask) == mask;
  }
  constexpr bool dio() const noexcept { return has(Dio); }
  constexpr bool ain() const noexcept { return has(Ain); }
  constexpr bool pwm() const noexcept { return has(Pwm); }
  constexpr bool dac() const noexcept { return has(Dac); }

  constexpr bool operator==(const PinCaps &) const noexcept = default;
};

/// Capability bits a pin must carry to accept `mode` (0xFF for an unknown
/// mode, which no pin satisfies). INPUT_PULLDOWN additionally needs
/// Info::supports_pulldown().
constexpr std::uint8_t required_caps(PinMode mode) noexcept {
  switch (mode) {
  case PinMode::Input:
  case PinMode::Output:
  case PinMode::InputPullup:
  case PinMode::InputPulldown:
    return PinCaps::Dio;
  case PinMode::AnalogIn:
    return PinCaps::Ain;
  case PinMode::Pwm:
    return PinCaps::Pwm;
  case PinMode::Dac:
    return PinCaps::Dac;
  }
  return 0xFFu;
}

/// True when a pin with `caps` can be put in `mode` on a board that does
/// (or does not) support INPUT_PULLDOWN.
constexpr bool mode_supported(PinMode mode, PinCaps caps,
                              bool pulldown_supported) noexcept {
  if (!caps.has(required_caps(mode))) {
    return false;
  }
  return mode != PinMode::InputPulldown || pulldown_supported;
}

// ---- Device information (GET_INFO) ------------------------------------------

/// Decoded usbio_info_t.
struct Info {
  std::uint16_t protocol_version{0};
  BoardId board_id{BoardId::Unknown};
  std::uint8_t n_pins{0};      ///< pins are addressed 0 .. n_pins-1
  std::uint8_t n_ain{0};       ///< number of pins carrying PinCaps::Ain
  std::uint8_t adc_bits{0};    ///< analogRead() resolution
  std::uint8_t pwm_bits{0};    ///< PWM duty resolution
  std::uint8_t dac_bits{0};    ///< 0 when the board has no DAC
  std::uint8_t queue_depth{0}; ///< firmware command queue capacity
  std::uint16_t vref_mv{0};    ///< ADC full-scale voltage, millivolts
  std::uint16_t io_mv{0};      ///< digital logic level, millivolts
  std::uint16_t flags{0};      ///< USBIO_FLAG_* bits

  constexpr bool has_vendor_interface() const noexcept {
    return (flags & USBIO_FLAG_VENDOR_INTERFACE) != 0;
  }
  constexpr bool supports_pulldown() const noexcept {
    return (flags & USBIO_FLAG_PULLDOWN) != 0;
  }
  constexpr bool has_dac() const noexcept { return dac_bits != 0; }
};

// ---- Little-endian byte access ----------------------------------------------
// ----------------------------------------------- Wire structs are decoded
// field by field; the caller guarantees the offsets are inside the span.

constexpr std::uint8_t read_u8(std::span<const std::byte> bytes,
                               std::size_t offset) noexcept {
  return std::to_integer<std::uint8_t>(bytes[offset]);
}

constexpr std::uint16_t read_u16le(std::span<const std::byte> bytes,
                                   std::size_t offset) noexcept {
  return static_cast<std::uint16_t>(
      std::to_integer<std::uint16_t>(bytes[offset]) |
      (std::to_integer<std::uint16_t>(bytes[offset + 1]) << 8));
}

constexpr void write_u8(std::span<std::byte> bytes, std::size_t offset,
                        std::uint8_t value) noexcept {
  bytes[offset] = static_cast<std::byte>(value);
}

constexpr void write_u16le(std::span<std::byte> bytes, std::size_t offset,
                           std::uint16_t value) noexcept {
  bytes[offset] = static_cast<std::byte>(value & 0xFFu);
  bytes[offset + 1] = static_cast<std::byte>((value >> 8) & 0xFFu);
}

/// Decodes a GET_INFO reply. Throws ProtocolError when the buffer is shorter
/// than InfoLen or does not start with the "UIO1" magic. The protocol
/// version is returned as-is; Device checks it against ProtocolVersion.
inline Info decode_info(std::span<const std::byte> bytes) {
  if (bytes.size() < InfoLen) {
    throw ProtocolError(
        "GET_INFO reply is too short: " + std::to_string(bytes.size()) +
        " of " + std::to_string(InfoLen) + " bytes");
  }
  for (std::size_t i = 0; i < Magic.size(); ++i) {
    if (std::to_integer<char>(bytes[InfoOffset::Magic + i]) != Magic[i]) {
      throw ProtocolError("GET_INFO reply does not carry the \"" +
                          std::string(Magic) + "\" magic");
    }
  }
  Info info;
  info.protocol_version = read_u16le(bytes, InfoOffset::ProtocolVersion);
  info.board_id = static_cast<BoardId>(read_u16le(bytes, InfoOffset::BoardId));
  info.n_pins = read_u8(bytes, InfoOffset::NPins);
  info.n_ain = read_u8(bytes, InfoOffset::NAin);
  info.adc_bits = read_u8(bytes, InfoOffset::AdcBits);
  info.pwm_bits = read_u8(bytes, InfoOffset::PwmBits);
  info.dac_bits = read_u8(bytes, InfoOffset::DacBits);
  info.queue_depth = read_u8(bytes, InfoOffset::QueueDepth);
  info.vref_mv = read_u16le(bytes, InfoOffset::VrefMv);
  info.io_mv = read_u16le(bytes, InfoOffset::IoMv);
  info.flags = read_u16le(bytes, InfoOffset::Flags);
  return info;
}

// ---- Names ------------------------------------------------------------------

/// Protocol name of a request, e.g. "DIO_WRITE".
constexpr std::string_view to_string(Request request) noexcept {
  switch (request) {
  case Request::GetInfo:
    return "GET_INFO";
  case Request::GetPinCaps:
    return "GET_PIN_CAPS";
  case Request::PinMode:
    return "PIN_MODE";
  case Request::DioRead:
    return "DIO_READ";
  case Request::DioWrite:
    return "DIO_WRITE";
  case Request::AiRead:
    return "AI_READ";
  case Request::PwmWrite:
    return "PWM_WRITE";
  case Request::DacWrite:
    return "DAC_WRITE";
  case Request::DioReadAll:
    return "DIO_READ_ALL";
  case Request::AiReadAll:
    return "AI_READ_ALL";
  case Request::GetStatus:
    return "GET_STATUS";
  case Request::Reset:
    return "RESET";
  }
  return "UNKNOWN";
}

/// Protocol name of a status code, e.g. "BAD_MODE".
constexpr std::string_view to_string(Status status) noexcept {
  switch (status) {
  case Status::Ok:
    return "OK";
  case Status::Busy:
    return "BUSY";
  case Status::BadPin:
    return "BAD_PIN";
  case Status::BadMode:
    return "BAD_MODE";
  case Status::BadCmd:
    return "BAD_CMD";
  case Status::Unsupported:
    return "UNSUPPORTED";
  case Status::QueueFull:
    return "QUEUE_FULL";
  case Status::BadValue:
    return "BAD_VALUE";
  }
  return "UNKNOWN";
}

/// Human-readable meaning of a status code.
constexpr std::string_view describe(Status status) noexcept {
  switch (status) {
  case Status::Ok:
    return "no error";
  case Status::Busy:
    return "queued commands still pending";
  case Status::BadPin:
    return "pin index out of range";
  case Status::BadMode:
    return "request not valid for the pin's current mode";
  case Status::BadCmd:
    return "unknown request";
  case Status::Unsupported:
    return "capability not available on this pin or board";
  case Status::QueueFull:
    return "command queue full";
  case Status::BadValue:
    return "value out of range";
  }
  return "unknown status code";
}

/// Protocol name of a pin mode, e.g. "INPUT_PULLUP".
constexpr std::string_view to_string(PinMode mode) noexcept {
  switch (mode) {
  case PinMode::Input:
    return "INPUT";
  case PinMode::Output:
    return "OUTPUT";
  case PinMode::InputPullup:
    return "INPUT_PULLUP";
  case PinMode::InputPulldown:
    return "INPUT_PULLDOWN";
  case PinMode::AnalogIn:
    return "ANALOG_IN";
  case PinMode::Pwm:
    return "PWM";
  case PinMode::Dac:
    return "DAC";
  }
  return "UNKNOWN";
}

/// Marketing name of a board, "unknown" for identifiers this driver does not
/// know.
constexpr std::string_view board_name(BoardId board) noexcept {
  switch (board) {
  case BoardId::Unknown:
    return "unknown";
  case BoardId::UnoR4Minima:
    return "UNO R4 Minima";
  case BoardId::NanoR4:
    return "Nano R4";
  case BoardId::RenesasGeneric:
    return "Renesas board";
  case BoardId::PortentaH7:
    return "Portenta H7";
  case BoardId::GigaR1:
    return "GIGA R1";
  case BoardId::Nano33Ble:
    return "Nano 33 BLE";
  case BoardId::MbedGeneric:
    return "mbed board";
  case BoardId::Zero:
    return "Zero";
  case BoardId::Mkr:
    return "MKR";
  case BoardId::Nano33Iot:
    return "Nano 33 IoT";
  case BoardId::SamdGeneric:
    return "SAMD board";
  case BoardId::NanoRp2040Connect:
    return "Nano RP2040 Connect";
  case BoardId::Rp2040Generic:
    return "RP2040 board";
  case BoardId::NanoEsp32:
    return "Nano ESP32";
  case BoardId::Esp32Generic:
    return "ESP32 board";
  }
  return "unknown";
}

constexpr std::string_view to_string(BoardId board) noexcept {
  return board_name(board);
}

} // namespace ArduinoDriver
