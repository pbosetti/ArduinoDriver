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

  // ---- Wire log -------------------------------------------------------------
  const std::vector<LoggedRequest> &log() const noexcept { return _log; }
  /// Most recent request; throws std::logic_error when the log is empty.
  const LoggedRequest &last() const;
  std::size_t count(Request request) const noexcept;
  void clear_log() noexcept { _log.clear(); }

private:
  struct Truncation {
    Request request;
    std::size_t max_len;
  };

  [[noreturn]] void stall(Status reason);
  bool take_busy() noexcept;
  std::size_t encode_info(std::span<std::byte> out);
  void check_pin(std::uint8_t pin) const;

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
};

} // namespace ArduinoDriver::Testing
