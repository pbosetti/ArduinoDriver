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
  StreamSelect = USBIO_REQ_STREAM_SELECT,
  StreamStart = USBIO_REQ_STREAM_START,
  StreamStop = USBIO_REQ_STREAM_STOP,
  StreamStatus = USBIO_REQ_STREAM_STATUS,
  GetTime = USBIO_REQ_GET_TIME,
  EventConfig = USBIO_REQ_EVENT_CONFIG,
  EventPop = USBIO_REQ_EVENT_POP,
  EventCounts = USBIO_REQ_EVENT_COUNTS,
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

/// Low byte of EVENT_CONFIG's wValue: which edges are reported for a pin.
/// Off removes the pin from the watch set.
enum class EdgeMode : std::uint8_t {
  Off = USBIO_EDGE_OFF,
  Rising = USBIO_EDGE_RISING,
  Falling = USBIO_EDGE_FALLING,
  Change = USBIO_EDGE_CHANGE,
};
inline constexpr std::uint8_t EdgeModeCount = USBIO_EDGE_COUNT;

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
inline constexpr std::size_t StreamStatusLen = sizeof(usbio_stream_status_t); // 16
inline constexpr std::size_t StreamHeaderLen = sizeof(usbio_stream_header_t);  // 12
inline constexpr std::size_t TimeReplyLen = sizeof(usbio_time_reply_t);       // 12
inline constexpr std::size_t EventHeaderLen = sizeof(usbio_event_header_t);   // 4
inline constexpr std::size_t EventLen = sizeof(usbio_event_t);               // 8
inline constexpr std::size_t EventCountLen = sizeof(usbio_event_count_t);    // 4

inline constexpr std::size_t MaxPins = USBIO_MAX_PINS;
inline constexpr std::size_t MaxAin = USBIO_MAX_AIN;
inline constexpr std::size_t MaxReplyLen = USBIO_MAX_REPLY_LEN;
inline constexpr std::size_t QueueDepth = USBIO_QUEUE_DEPTH;

/// Streaming limits (see usbio_protocol.h "Streaming").
inline constexpr std::uint16_t StreamMagic = USBIO_STREAM_MAGIC;
inline constexpr std::size_t StreamEpSize = USBIO_STREAM_EP_SIZE;
inline constexpr std::size_t MaxStreamChannels = USBIO_MAX_STREAM_CHANNELS;
inline constexpr std::uint16_t StreamMinPeriodUs = USBIO_STREAM_MIN_PERIOD_US;

/// Pin event limits (see usbio_protocol.h "Pin events").
inline constexpr std::size_t MaxEventPins = USBIO_MAX_EVENT_PINS;
inline constexpr std::size_t EventQueueDepth = USBIO_EVENT_QUEUE_DEPTH;
inline constexpr std::size_t MaxEventsPerPop = USBIO_MAX_EVENTS_PER_POP;
inline constexpr std::uint8_t MaxDebounceMs = USBIO_MAX_DEBOUNCE_MS;

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
inline constexpr std::size_t StreamMaxChannels = 20;
inline constexpr std::size_t EventMaxPins = 21;
inline constexpr std::size_t Reserved = 22;
} // namespace InfoOffset

/// Byte offsets inside usbio_time_reply_t (GET_TIME reply).
namespace TimeReplyOffset {
inline constexpr std::size_t Status = 0;
inline constexpr std::size_t Reserved = 1;
inline constexpr std::size_t Reserved2 = 2;
inline constexpr std::size_t Millis = 4;
inline constexpr std::size_t Micros = 8;
} // namespace TimeReplyOffset

/// Byte offsets inside usbio_event_header_t (the header of EVENT_POP and
/// EVENT_COUNTS replies; the array of usbio_event_t / usbio_event_count_t
/// entries follows at EventHeaderLen).
namespace EventHeaderOffset {
inline constexpr std::size_t Status = 0;
inline constexpr std::size_t Count = 1;
inline constexpr std::size_t Dropped = 2;
inline constexpr std::size_t Pending = 3;
} // namespace EventHeaderOffset

/// Byte offsets of one usbio_event_t entry, relative to that entry's start.
namespace EventOffset {
inline constexpr std::size_t Pin = 0;
inline constexpr std::size_t Edge = 1;
inline constexpr std::size_t Seq = 2;
inline constexpr std::size_t TMs = 4;
} // namespace EventOffset

/// Byte offsets of one usbio_event_count_t entry, relative to that entry's
/// start.
namespace EventCountOffset {
inline constexpr std::size_t Pin = 0;
inline constexpr std::size_t Mode = 1;
inline constexpr std::size_t Count = 2;
} // namespace EventCountOffset

/// Byte offsets inside usbio_stream_status_t (GET_STREAM_STATUS reply).
namespace StreamStatusOffset {
inline constexpr std::size_t Status = 0;
inline constexpr std::size_t Running = 1;
inline constexpr std::size_t NChannels = 2;
inline constexpr std::size_t Flags = 3;
inline constexpr std::size_t PeriodUs = 4;
inline constexpr std::size_t Reserved = 6;
inline constexpr std::size_t Seq = 8;
inline constexpr std::size_t Overruns = 12;
} // namespace StreamStatusOffset

/// Byte offsets inside usbio_stream_header_t (the record header on the bulk
/// IN endpoint; the sample/bitmap payload follows at StreamHeaderLen).
namespace StreamHeaderOffset {
inline constexpr std::size_t Magic = 0;
inline constexpr std::size_t NSamples = 2;
inline constexpr std::size_t Seq = 4;
inline constexpr std::size_t TUs = 8;
} // namespace StreamHeaderOffset

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
/// Length of the digital bitmap appended to a stream record
/// (USBIO_STREAM_FLAG_DIGITAL), padded with a zero byte to an even length.
constexpr std::size_t stream_digital_bitmap_len(std::size_t n_pins) noexcept {
  const std::size_t raw = dio_bitmap_len(n_pins);
  return raw + (raw % 2);
}
/// Total length of one stream record: header + n_samples uint16_t samples
/// [+ the padded digital bitmap]. Always even.
constexpr std::size_t stream_record_len(std::size_t n_samples, bool digital,
                                        std::size_t n_pins) noexcept {
  std::size_t len = StreamHeaderLen + 2 * n_samples;
  if (digital) {
    len += stream_digital_bitmap_len(n_pins);
  }
  return len;
}
/// Total EVENT_POP reply length (header + `count` usbio_event_t entries).
constexpr std::size_t event_pop_len(std::size_t count) noexcept {
  return EventHeaderLen + EventLen * count;
}
/// Total EVENT_COUNTS reply length (header + `count` usbio_event_count_t
/// entries).
constexpr std::size_t event_counts_len(std::size_t count) noexcept {
  return EventHeaderLen + EventCountLen * count;
}
/// wValue of EVENT_CONFIG: debounce in the high byte, edge mode in the low.
constexpr std::uint16_t encode_event_config_value(std::uint8_t debounce_ms,
                                                   EdgeMode edge) noexcept {
  return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(debounce_ms) << 8) |
      static_cast<std::uint8_t>(edge));
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
  case Request::StreamSelect:
  case Request::StreamStart:
  case Request::StreamStop:
  case Request::EventConfig:
  case Request::Reset:
    return true;
  case Request::GetInfo:
  case Request::GetPinCaps:
  case Request::DioRead:
  case Request::AiRead:
  case Request::DioReadAll:
  case Request::AiReadAll:
  case Request::GetStatus:
  case Request::StreamStatus:
  case Request::GetTime:
  case Request::EventPop:
  case Request::EventCounts:
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

/// wIndex of STREAM_START / flags byte of usbio_stream_status_t.
struct StreamFlags {
  static constexpr std::uint8_t Digital = USBIO_STREAM_FLAG_DIGITAL;
  static constexpr std::uint8_t StopOnOverrun = USBIO_STREAM_FLAG_STOP_ON_OVERRUN;

  std::uint8_t bits{0};

  constexpr bool has(std::uint8_t mask) const noexcept {
    return (bits & mask) == mask;
  }
  /// The digital bitmap is appended to every record.
  constexpr bool digital() const noexcept { return has(Digital); }
  /// The stream stops instead of dropping records when the ring fills.
  constexpr bool stop_on_overrun() const noexcept { return has(StopOnOverrun); }

  constexpr bool operator==(const StreamFlags &) const noexcept = default;
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
  std::uint8_t stream_max_channels{0}; ///< pins STREAM_SELECT accepts at
                                       ///< once; 0 when streaming() is false
  std::uint8_t event_max_pins{0};     ///< pins EVENT_CONFIG watches at once;
                                       ///< 0 when events() is false

  constexpr bool has_vendor_interface() const noexcept {
    return (flags & USBIO_FLAG_VENDOR_INTERFACE) != 0;
  }
  constexpr bool supports_pulldown() const noexcept {
    return (flags & USBIO_FLAG_PULLDOWN) != 0;
  }
  constexpr bool has_dac() const noexcept { return dac_bits != 0; }
  /// True when the board exposes the bulk IN endpoint and STREAM_* requests.
  constexpr bool streaming() const noexcept {
    return (flags & USBIO_FLAG_STREAMING) != 0;
  }
  /// True when the board exposes the EVENT_* requests.
  constexpr bool events() const noexcept {
    return (flags & USBIO_FLAG_EVENTS) != 0;
  }
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

constexpr std::uint32_t read_u32le(std::span<const std::byte> bytes,
                                   std::size_t offset) noexcept {
  return static_cast<std::uint32_t>(read_u16le(bytes, offset)) |
        (static_cast<std::uint32_t>(read_u16le(bytes, offset + 2)) << 16);
}

constexpr void write_u32le(std::span<std::byte> bytes, std::size_t offset,
                           std::uint32_t value) noexcept {
  write_u16le(bytes, offset, static_cast<std::uint16_t>(value & 0xFFFFu));
  write_u16le(bytes, offset + 2,
             static_cast<std::uint16_t>((value >> 16) & 0xFFFFu));
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
  info.stream_max_channels = read_u8(bytes, InfoOffset::StreamMaxChannels);
  info.event_max_pins = read_u8(bytes, InfoOffset::EventMaxPins);
  return info;
}

// ---- Streaming ---------------------------------------------------------------

/// Decoded usbio_stream_status_t (GET_STREAM_STATUS reply).
struct StreamStatus {
  Status status{Status::Ok};
  bool running{false};
  std::uint8_t n_channels{0};
  StreamFlags flags{};
  std::uint16_t period_us{0}; ///< 0 = free running
  std::uint32_t seq{0};       ///< seq of the most recent record produced
  std::uint32_t overruns{0};  ///< records dropped by the device since START
};

/// Decodes a GET_STREAM_STATUS reply. Throws ProtocolError when the buffer is
/// shorter than StreamStatusLen.
inline StreamStatus decode_stream_status(std::span<const std::byte> bytes) {
  if (bytes.size() < StreamStatusLen) {
    throw ProtocolError(
        "GET_STREAM_STATUS reply is too short: " +
        std::to_string(bytes.size()) + " of " +
        std::to_string(StreamStatusLen) + " bytes");
  }
  StreamStatus s;
  s.status = static_cast<Status>(read_u8(bytes, StreamStatusOffset::Status));
  s.running = read_u8(bytes, StreamStatusOffset::Running) != 0;
  s.n_channels = read_u8(bytes, StreamStatusOffset::NChannels);
  s.flags.bits = read_u8(bytes, StreamStatusOffset::Flags);
  s.period_us = read_u16le(bytes, StreamStatusOffset::PeriodUs);
  s.seq = read_u32le(bytes, StreamStatusOffset::Seq);
  s.overruns = read_u32le(bytes, StreamStatusOffset::Overruns);
  return s;
}

/// Header of one record on the bulk IN endpoint (usbio_stream_header_t); the
/// sample payload (and the optional digital bitmap) follows at
/// StreamHeaderLen, see stream_record_len().
struct StreamHeader {
  std::uint16_t magic{0}; ///< StreamMagic when this is a real record header
  std::uint16_t n_samples{0};
  std::uint32_t seq{0};
  std::uint32_t t_us{0};

  constexpr bool has_magic() const noexcept { return magic == StreamMagic; }
};

/// Decodes the 12-byte header of one stream record. Caller guarantees
/// `bytes.size() >= StreamHeaderLen`.
constexpr StreamHeader decode_stream_header(std::span<const std::byte> bytes) noexcept {
  StreamHeader h;
  h.magic = read_u16le(bytes, StreamHeaderOffset::Magic);
  h.n_samples = read_u16le(bytes, StreamHeaderOffset::NSamples);
  h.seq = read_u32le(bytes, StreamHeaderOffset::Seq);
  h.t_us = read_u32le(bytes, StreamHeaderOffset::TUs);
  return h;
}

/// Encodes the 12-byte header of one stream record. Caller guarantees
/// `bytes.size() >= StreamHeaderLen`.
constexpr void encode_stream_header(std::span<std::byte> bytes,
                                    const StreamHeader &header) noexcept {
  write_u16le(bytes, StreamHeaderOffset::Magic, header.magic);
  write_u16le(bytes, StreamHeaderOffset::NSamples, header.n_samples);
  write_u32le(bytes, StreamHeaderOffset::Seq, header.seq);
  write_u32le(bytes, StreamHeaderOffset::TUs, header.t_us);
}

// ---- Device time --------------------------------------------------------

/// Decoded usbio_time_reply_t (GET_TIME reply).
struct TimeReply {
  Status status{Status::Ok};
  std::uint32_t millis{0};
  std::uint32_t micros{0};
};

/// Decodes a GET_TIME reply. Throws ProtocolError when the buffer is shorter
/// than TimeReplyLen.
inline TimeReply decode_time_reply(std::span<const std::byte> bytes) {
  if (bytes.size() < TimeReplyLen) {
    throw ProtocolError("GET_TIME reply is too short: " +
                        std::to_string(bytes.size()) + " of " +
                        std::to_string(TimeReplyLen) + " bytes");
  }
  TimeReply t;
  t.status = static_cast<Status>(read_u8(bytes, TimeReplyOffset::Status));
  t.millis = read_u32le(bytes, TimeReplyOffset::Millis);
  t.micros = read_u32le(bytes, TimeReplyOffset::Micros);
  return t;
}

/// Rebuilds a 64-bit microsecond clock from one GET_TIME reply's millis()
/// and micros() (see usbio_protocol.h's GET_TIME comment): micros() wraps
/// every ~71.6 minutes but millis() every ~49.7 days, and the two agree
/// modulo 2^32 (micros == millis * 1000 mod 2^32), so millis() pins down
/// which ~71.6-minute wrap of micros() the reply belongs to.
///
/// Algorithm: `millis * 1000` (widened to 64 bits, no overflow risk since
/// millis is 32 bits) is within 1000 us of the true elapsed time -- that is
/// the size of the truncation `millis()` itself performs. Rounding that
/// estimate down to the nearest multiple of 2^32 and adding the exact
/// `micros` reading reconstructs the true value, except right at a 2^32
/// boundary, where the sub-1000us estimation error can place the naive
/// result one whole wrap away from the truth; the two branches below detect
/// that (the naive result would differ from the millis-based estimate by
/// nearly a full wrap, not by a small truncation error) and correct it.
///
/// Valid for ~49.7 days after the device booted, i.e. until millis() itself
/// wraps -- exactly the range GET_TIME's own documentation promises. Past
/// that, or if millis and micros were not read from the same GET_TIME reply,
/// the result is silently wrong: there is no way to detect it from these two
/// values alone.
constexpr std::uint64_t reconstruct_micros64(std::uint32_t millis_value,
                                             std::uint32_t micros_value) noexcept {
  constexpr std::uint64_t Wrap = std::uint64_t{1} << 32; // micros()'s period
  const std::uint64_t coarse = static_cast<std::uint64_t>(millis_value) * 1000ull;
  const std::uint64_t base = (coarse / Wrap) * Wrap; // coarse, epoch-aligned
  std::uint64_t candidate = base + micros_value;
  if (candidate > coarse && candidate - coarse > Wrap / 2) {
    candidate -= Wrap;
  } else if (coarse > candidate && coarse - candidate > Wrap / 2) {
    candidate += Wrap;
  }
  return candidate;
}

// ---- Pin events -----------------------------------------------------------

/// Decoded header of an EVENT_POP / EVENT_COUNTS reply (usbio_event_header_t);
/// the array of entries follows at EventHeaderLen.
struct EventHeader {
  Status status{Status::Ok};
  std::uint8_t count{0};
  std::uint8_t dropped{0}; ///< EVENT_POP only; 0 in an EVENT_COUNTS reply
  std::uint8_t pending{0}; ///< EVENT_POP only; 0 in an EVENT_COUNTS reply
};

/// Decodes the 4-byte header shared by EVENT_POP and EVENT_COUNTS replies.
/// Throws ProtocolError when the buffer is shorter than EventHeaderLen.
inline EventHeader decode_event_header(std::span<const std::byte> bytes) {
  if (bytes.size() < EventHeaderLen) {
    throw ProtocolError("EVENT_POP/EVENT_COUNTS reply is too short: " +
                        std::to_string(bytes.size()) + " of " +
                        std::to_string(EventHeaderLen) + " bytes");
  }
  EventHeader h;
  h.status = static_cast<Status>(read_u8(bytes, EventHeaderOffset::Status));
  h.count = read_u8(bytes, EventHeaderOffset::Count);
  h.dropped = read_u8(bytes, EventHeaderOffset::Dropped);
  h.pending = read_u8(bytes, EventHeaderOffset::Pending);
  return h;
}

/// One decoded pin edge (usbio_event_t), as EVENT_POP reports it. `edge` is
/// Rising or Falling (never Change: the device only ever reports the edge it
/// actually saw).
struct PinEvent {
  std::uint8_t pin{0};
  EdgeMode edge{EdgeMode::Off};
  std::uint16_t seq{0};  ///< per-pin edge counter after this edge; matches
                         ///< EventCount::count, and wraps with it
  std::uint32_t t_ms{0}; ///< device millis() when poll() detected the edge
};

/// Decodes the usbio_event_t at index `index` of an EVENT_POP reply (i.e. at
/// byte offset `EventHeaderLen + index * EventLen`). Caller guarantees the
/// span reaches that far.
constexpr PinEvent decode_event(std::span<const std::byte> bytes,
                                std::size_t index) noexcept {
  const std::size_t base = EventHeaderLen + EventLen * index;
  PinEvent e;
  e.pin = read_u8(bytes, base + EventOffset::Pin);
  e.edge = static_cast<EdgeMode>(read_u8(bytes, base + EventOffset::Edge));
  e.seq = read_u16le(bytes, base + EventOffset::Seq);
  e.t_ms = read_u32le(bytes, base + EventOffset::TMs);
  return e;
}

/// One watched pin's counter (usbio_event_count_t), as EVENT_COUNTS reports
/// it.
struct EventCount {
  std::uint8_t pin{0};
  EdgeMode mode{EdgeMode::Off}; ///< edge mode currently armed for the pin
  std::uint16_t count{0};       ///< accepted edges since the pin was
                                ///< configured; wraps
};

/// Decodes the usbio_event_count_t at index `index` of an EVENT_COUNTS reply
/// (i.e. at byte offset `EventHeaderLen + index * EventCountLen`). Caller
/// guarantees the span reaches that far.
constexpr EventCount decode_event_count(std::span<const std::byte> bytes,
                                        std::size_t index) noexcept {
  const std::size_t base = EventHeaderLen + EventCountLen * index;
  EventCount c;
  c.pin = read_u8(bytes, base + EventCountOffset::Pin);
  c.mode = static_cast<EdgeMode>(read_u8(bytes, base + EventCountOffset::Mode));
  c.count = read_u16le(bytes, base + EventCountOffset::Count);
  return c;
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
  case Request::StreamSelect:
    return "STREAM_SELECT";
  case Request::StreamStart:
    return "STREAM_START";
  case Request::StreamStop:
    return "STREAM_STOP";
  case Request::StreamStatus:
    return "GET_STREAM_STATUS";
  case Request::GetTime:
    return "GET_TIME";
  case Request::EventConfig:
    return "EVENT_CONFIG";
  case Request::EventPop:
    return "EVENT_POP";
  case Request::EventCounts:
    return "EVENT_COUNTS";
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

/// Protocol name of an edge mode, e.g. "FALLING".
constexpr std::string_view to_string(EdgeMode mode) noexcept {
  switch (mode) {
  case EdgeMode::Off:
    return "OFF";
  case EdgeMode::Rising:
    return "RISING";
  case EdgeMode::Falling:
    return "FALLING";
  case EdgeMode::Change:
    return "CHANGE";
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
