// Device.h - high-level, synchronous view of one UsbIo device.
#pragma once

#include "arduino_driver/Errors.h"
#include "arduino_driver/Protocol.h"
#include "arduino_driver/Transport.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
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
/// Device is NOT thread-safe: use one instance per thread or serialise the
/// calls externally.
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

private:
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

  std::unique_ptr<Transport> _transport;
  Options _options;
  Info _info{};
  std::vector<PinCaps> _caps;
  std::vector<std::uint8_t> _analog_pins;
};

} // namespace ArduinoDriver
