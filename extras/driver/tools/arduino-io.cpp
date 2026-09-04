// arduino-io.cpp - command line tool for boards running the UsbIo firmware.
//
//   arduino-io [options] <command> [args...]
//
// Exit codes: 0 success, 1 device or protocol error (message on stderr),
// 2 usage error.
#include "arduino_driver/Enumerator.h"
#include "arduino_driver/EventWatcher.h"
#include "arduino_driver/Stream.h"

#include <cxxopts.hpp>
#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

using namespace ArduinoDriver;
using namespace std::chrono_literals;

namespace {

// The only global: set by the SIGINT handler, polled by `monitor`.
std::atomic<bool> StopRequested{false};

extern "C" void on_interrupt(int) { StopRequested.store(true); }

constexpr int ExitOk = 0;
constexpr int ExitDevice = 1;
constexpr int ExitUsage = 2;

/// Bad command line: message and exit code 2.
class UsageError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

struct Settings {
  std::optional<std::string> serial;
  EnumerateOptions enumerate;
  DeviceOptions device;
  LibusbTransportOptions transport;
  bool volts{false};
  double hz{10.0};
  bool verbose{false};

  // `stream` only.
  bool hz_given{false};                  ///< --hz was explicitly passed
  std::optional<unsigned> period_us;     ///< --period-us
  std::optional<std::string> csv_path;   ///< --csv

  // `stream` / `watch`.
  double run_seconds{0.0}; ///< --seconds (0: run until Ctrl-C)

  // `watch` only.
  EdgeMode watch_edge{EdgeMode::Change}; ///< --edge
  unsigned watch_debounce_ms{0};         ///< --debounce
};

// ---- Argument parsing -------------------------------------------------------

std::string lower(std::string_view text) {
  std::string out(text);
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return out;
}

/// Whole-string unsigned integer; base 0 accepts 0x prefixes.
std::optional<unsigned long> parse_unsigned(std::string_view text, int base) {
  const std::string s(text);
  if (s.empty() || s.front() == '-') {
    return std::nullopt;
  }
  char *end = nullptr;
  errno = 0;
  const unsigned long v = std::strtoul(s.c_str(), &end, base);
  if (errno != 0 || end == s.c_str() || *end != '\0') {
    return std::nullopt;
  }
  return v;
}

unsigned long require_unsigned(std::string_view text, unsigned long max,
                               std::string_view what, int base = 0) {
  const auto v = parse_unsigned(text, base);
  if (!v || *v > max) {
    throw UsageError(
        fmt::format("invalid {} \"{}\" (expected 0..{})", what, text, max));
  }
  return *v;
}

std::uint8_t parse_pin(std::string_view text) {
  return static_cast<std::uint8_t>(require_unsigned(text, 255, "pin", 10));
}

std::uint16_t parse_id(std::string_view text, std::string_view what) {
  return static_cast<std::uint16_t>(require_unsigned(text, 0xFFFF, what, 16));
}

PinMode parse_mode(std::string_view text) {
  const std::string m = lower(text);
  if (m == "input" || m == "in") {
    return PinMode::Input;
  }
  if (m == "output" || m == "out") {
    return PinMode::Output;
  }
  if (m == "pullup" || m == "input_pullup") {
    return PinMode::InputPullup;
  }
  if (m == "pulldown" || m == "input_pulldown") {
    return PinMode::InputPulldown;
  }
  if (m == "analog" || m == "analog_in" || m == "ain") {
    return PinMode::AnalogIn;
  }
  if (m == "pwm") {
    return PinMode::Pwm;
  }
  if (m == "dac") {
    return PinMode::Dac;
  }
  throw UsageError(fmt::format(
      "unknown mode \"{}\" (input|output|pullup|pulldown|analog|pwm|dac)",
      text));
}

EdgeMode parse_edge_mode(std::string_view text) {
  const std::string m = lower(text);
  if (m == "rising" || m == "rise" || m == "r") {
    return EdgeMode::Rising;
  }
  if (m == "falling" || m == "fall" || m == "f") {
    return EdgeMode::Falling;
  }
  if (m == "change" || m == "both" || m == "c") {
    return EdgeMode::Change;
  }
  throw UsageError(
      fmt::format("unknown edge \"{}\" (rising|falling|change)", text));
}

bool parse_level(std::string_view text) {
  const std::string v = lower(text);
  if (v == "1" || v == "high" || v == "on" || v == "true") {
    return true;
  }
  if (v == "0" || v == "low" || v == "off" || v == "false") {
    return false;
  }
  throw UsageError(fmt::format("invalid level \"{}\" (0|1|high|low)", text));
}

std::optional<double> parse_double(std::string_view text) {
  const std::string s(text);
  char *end = nullptr;
  errno = 0;
  const double v = std::strtod(s.c_str(), &end);
  if (s.empty() || errno != 0 || end == s.c_str() || *end != '\0') {
    return std::nullopt;
  }
  return v;
}

/// Number with an optional unit suffix ("50%", "1.5V"); returns the numeric
/// part and whether the suffix was present.
std::pair<double, bool> parse_with_suffix(std::string_view text, char suffix,
                                          std::string_view what) {
  bool has_suffix = false;
  if (!text.empty() && std::tolower(static_cast<unsigned char>(text.back())) ==
                           std::tolower(static_cast<unsigned char>(suffix))) {
    has_suffix = true;
    text.remove_suffix(1);
  }
  const auto v = parse_double(text);
  if (!v) {
    throw UsageError(fmt::format("invalid {} \"{}\"", what, text));
  }
  return {*v, has_suffix};
}

void require_args(const std::vector<std::string> &args, std::size_t n,
                  std::string_view usage) {
  if (args.size() < n) {
    throw UsageError(fmt::format("usage: arduino-io {}", usage));
  }
}

// ---- Output helpers ---------------------------------------------------------

std::string caps_flags(PinCaps caps) {
  return fmt::format("{}{}{}{}", caps.dio() ? 'D' : '-', caps.ain() ? 'A' : '-',
                     caps.pwm() ? 'P' : '-', caps.dac() ? 'C' : '-');
}

std::string analog_value(const Device &dev, std::uint16_t raw, bool volts) {
  return volts ? fmt::format("{:.4f}", dev.to_volts(raw))
               : fmt::format("{}", raw);
}

// ---- Commands ---------------------------------------------------------------

int cmd_list(const std::shared_ptr<Context> &context, const Settings &s) {
  const std::vector<DeviceInfo> devices = list_devices(context, s.enumerate);
  fmt::print("{:<8} {:<9} {:<20} {:<20} {:<3} {}\n", "BUS:ADDR", "VID:PID",
             "BOARD", "SERIAL", "ITF", "PINS");
  for (const DeviceInfo &d : devices) {
    if (!d.identified) {
      continue;
    }
    std::string board = "?";
    std::string pins = "?";
    if (d.info) {
      board = d.info->n_pins == 0 ? "(not ready)"
                                  : std::string(board_name(d.info->board_id));
      pins = fmt::format("{}", d.info->n_pins);
    }
    fmt::print("{:03}:{:03}  {:04x}:{:04x} {:<20} {:<20} {:<3} {}\n", d.bus,
               d.address, d.vid, d.pid, board, d.serial,
               d.interface_number ? fmt::format("{}", *d.interface_number)
                                  : "-",
               pins);
    if (!d.probe_error.empty()) {
      fmt::print("    (not probed: {})\n", d.probe_error);
    }
  }
  // Candidates that could not be opened: maybe ours, maybe not.
  const bool unprobed =
      std::any_of(devices.begin(), devices.end(),
                  [](const DeviceInfo &d) { return !d.identified; });
  if (unprobed) {
    fmt::print("\nNot probed (may or may not run UsbIo):\n");
    for (const DeviceInfo &d : devices) {
      if (!d.identified) {
        fmt::print("{:03}:{:03}  {:04x}:{:04x} {}\n", d.bus, d.address, d.vid,
                   d.pid, d.probe_error);
      }
    }
    fmt::print("Linux: install extras/driver/etc/99-arduino-usbio.rules (udev). "
               "Windows: bind WinUSB to the UsbIo interface with Zadig. "
               "Otherwise close the program holding the device.\n");
  }
  return ExitOk;
}

int cmd_info(Device &dev) {
  const Info &info = dev.info();
  fmt::print("board:            {} (id 0x{:04X})\n", board_name(info.board_id),
             static_cast<unsigned>(info.board_id));
  fmt::print("protocol version: 0x{:04X}\n", info.protocol_version);
  fmt::print("pins:             {} (analog: {})\n", info.n_pins, info.n_ain);
  fmt::print(
      "resolution:       adc {} bits,  mode <pin> <mode>         "
      "input|output|pullup|pulldown|analog|pwm|dac{} bits, dac {} bits{}\n",
      info.adc_bits, info.pwm_bits, info.dac_bits,
      info.has_dac() ? "" : " (no DAC)");
  fmt::print("voltages:         vref {} mV, io {} mV\n", info.vref_mv,
             info.io_mv);
  fmt::print("flags:            0x{:04X}{}{}\n", info.flags,
             info.has_vendor_interface() ? " vendor-interface" : "",
             info.supports_pulldown() ? " pulldown" : "");
  fmt::print("queue depth:      {}\n", info.queue_depth);
  if (const auto *usb =
          dynamic_cast<const LibusbTransport *>(&dev.transport())) {
    fmt::print(
        "transport:        {} recipient, interface {}{}\n",
        usb->recipient() == Recipient::Interface ? "interface" : "device",
        usb->interface_number() ? fmt::format("{}", *usb->interface_number())
                                : "none",
        usb->interface_claimed() ? " (claimed)" : "");
  }
  return ExitOk;
}

int cmd_caps(Device &dev) {
  fmt::print("{:<4} {:<5} {}\n", "PIN", "CAPS", "NOTES");
  const std::vector<std::uint8_t> &analog = dev.analog_pins();
  for (std::uint8_t pin = 0; pin < dev.pin_count(); ++pin) {
    const PinCaps caps = dev.pin_caps(pin);
    std::string notes;
    if (caps.ain()) {
      const auto it = std::find(analog.begin(), analog.end(), pin);
      notes += fmt::format("analog #{}", it - analog.begin());
      if (!caps.dio()) {
        notes += " (analog-only pad)";
      }
    }
    if (caps.dac()) {
      notes += notes.empty() ? "DAC" : ", DAC";
    }
    fmt::print("{:<4} {:<5} {}\n", pin, caps_flags(caps), notes);
  }
  fmt::print("D digital I/O, A analog input, P PWM, C DAC\n");
  return ExitOk;
}

int cmd_mode(Device &dev, const std::vector<std::string> &args) {
  require_args(args, 2,
               "mode <pin> "
               "<input|output|pullup|pulldown|analog|pwm|dac>");
  dev.pin_mode(parse_pin(args[0]), parse_mode(args[1]));
  return ExitOk;
}

int cmd_read(Device &dev, const std::vector<std::string> &args) {
  require_args(args, 1, "read <pin>");
  fmt::print("{}\n", dev.digital_read(parse_pin(args[0])) ? 1 : 0);
  return ExitOk;
}

int cmd_write(Device &dev, const std::vector<std::string> &args) {
  require_args(args, 2, "write <pin> <0|1|high|low>");
  dev.digital_write(parse_pin(args[0]), parse_level(args[1]));
  return ExitOk;
}

int cmd_aread(Device &dev, const std::vector<std::string> &args,
              const Settings &s) {
  require_args(args, 1, "aread <pin> [--volts]");
  const std::uint16_t raw = dev.analog_read(parse_pin(args[0]));
  fmt::print("{}\n", analog_value(dev, raw, s.volts));
  return ExitOk;
}

int cmd_readall(Device &dev) {
  const std::vector<bool> levels = dev.read_all_digital();
  for (std::size_t pin = 0; pin < levels.size(); ++pin) {
    fmt::print("{}{}", pin == 0 ? "" : " ", levels[pin] ? 1 : 0);
  }
  fmt::print("\n");
  return ExitOk;
}

int cmd_areadall(Device &dev, const Settings &s) {
  const std::vector<std::uint16_t> samples = dev.read_all_analog();
  const std::vector<std::uint8_t> &pins = dev.analog_pins();
  for (std::size_t i = 0; i < samples.size(); ++i) {
    fmt::print("{}{}={}", i == 0 ? "" : " ", pins[i],
               analog_value(dev, samples[i], s.volts));
  }
  fmt::print("\n");
  return ExitOk;
}

int cmd_pwm(Device &dev, const std::vector<std::string> &args) {
  require_args(args, 2, "pwm <pin> <duty code | percent%>");
  const std::uint8_t pin = parse_pin(args[0]);
  const auto [value, percent] = parse_with_suffix(args[1], '%', "duty");
  if (percent) {
    dev.pwm_write_fraction(pin, value / 100.0);
  } else {
    dev.pwm_write(pin, static_cast<std::uint16_t>(
                           require_unsigned(args[1], 0xFFFF, "duty", 10)));
  }
  return ExitOk;
}

int cmd_dac(Device &dev, const std::vector<std::string> &args) {
  require_args(args, 2, "dac <pin> <code | voltsV>");
  const std::uint8_t pin = parse_pin(args[0]);
  const auto [value, volts] = parse_with_suffix(args[1], 'V', "DAC value");
  if (volts) {
    dev.dac_write_volts(pin, value);
  } else {
    dev.dac_write(pin, static_cast<std::uint16_t>(
                           require_unsigned(args[1], 0xFFFF, "DAC value", 10)));
  }
  return ExitOk;
}

int cmd_status(Device &dev) {
  std::uint8_t pending = 0;
  const Status last_error = dev.status(&pending);
  fmt::print("last error:    {} ({})\n", to_string(last_error),
             describe(last_error));
  fmt::print("queue pending: {}\n", pending);
  return ExitOk;
}

/// GET_TIME plus the host-side anchor: device millis()/micros(), the
/// reconstructed 64-bit microsecond clock, and the round-trip-bounded
/// estimate of how well host_time and micros64 line up (see DeviceTime).
int cmd_time(Device &dev) {
  const DeviceTime t = dev.read_time();
  const double rtt_ms =
      std::chrono::duration<double, std::milli>(t.round_trip).count();
  fmt::print("millis:        {}\n", t.millis);
  fmt::print("micros:        {}\n", t.micros);
  fmt::print("micros64:      {} us\n", t.micros64);
  fmt::print("round trip:    {:.3f} ms (host offset uncertainty +/- {:.3f} ms)\n",
             rtt_ms, rtt_ms / 2.0);
  return ExitOk;
}

/// Prints DIO levels and analog samples of the selected pins (all pins when
/// none is given) until Ctrl-C. Pins must have been configured with `mode`
/// beforehand: unconfigured pins read 0.
int cmd_monitor(Device &dev, const std::vector<std::string> &args,
                const Settings &s) {
  std::vector<std::uint8_t> pins;
  for (const std::string &arg : args) {
    const std::uint8_t pin = parse_pin(arg);
    dev.pin_caps(pin); // range check before starting
    pins.push_back(pin);
  }
  if (pins.empty()) {
    for (std::uint8_t pin = 0; pin < dev.pin_count(); ++pin) {
      pins.push_back(pin);
    }
  }
  const std::vector<std::uint8_t> &analog = dev.analog_pins();
  const auto period = std::chrono::duration<double>(1.0 / s.hz);
  std::signal(SIGINT, on_interrupt);
  const auto start = std::chrono::steady_clock::now();
  while (!StopRequested.load()) {
    const auto now = std::chrono::steady_clock::now();
    const std::vector<bool> levels = dev.read_all_digital();
    const std::vector<std::uint16_t> samples = dev.read_all_analog();
    std::string line = fmt::format(
        "{:>9.3f}s", std::chrono::duration<double>(now - start).count());
    for (const std::uint8_t pin : pins) {
      const PinCaps caps = dev.pin_caps(pin);
      if (caps.dio()) {
        line += fmt::format(" D{}={}", pin, levels[pin] ? 1 : 0);
      }
      if (caps.ain()) {
        const auto it = std::find(analog.begin(), analog.end(), pin);
        const auto index = static_cast<std::size_t>(it - analog.begin());
        line += fmt::format(" A{}={}", pin,
                            analog_value(dev, samples[index], s.volts));
      }
    }
    fmt::print("{}\n", line);
    std::fflush(stdout);
    const auto deadline =
        now +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    while (!StopRequested.load() &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(
          std::min(50ms, std::chrono::duration_cast<std::chrono::milliseconds>(
                             deadline - std::chrono::steady_clock::now()) +
                             1ms));
    }
  }
  return ExitOk;
}

/// Comma-separated pin list, e.g. "14,15,16".
std::vector<std::uint8_t> parse_pin_list(std::string_view text) {
  std::vector<std::uint8_t> pins;
  std::size_t start = 0;
  while (start <= text.size()) {
    const std::size_t comma = text.find(',', start);
    const std::string_view token =
        text.substr(start, comma == std::string_view::npos
                              ? std::string_view::npos
                              : comma - start);
    if (!token.empty()) {
      pins.push_back(parse_pin(token));
    }
    if (comma == std::string_view::npos) {
      break;
    }
    start = comma + 1;
  }
  if (pins.empty()) {
    throw UsageError(fmt::format("invalid pin list \"{}\"", text));
  }
  return pins;
}

/// StreamConfig::period from --hz / --period-us (mutually exclusive; neither
/// given means free-running).
std::chrono::microseconds resolve_stream_period(const Settings &s) {
  if (s.hz_given && s.period_us) {
    throw UsageError("stream: --hz and --period-us are mutually exclusive");
  }
  if (s.period_us) {
    return std::chrono::microseconds(*s.period_us);
  }
  if (s.hz_given) {
    if (!(s.hz > 0.0)) {
      throw UsageError("--hz must be positive");
    }
    return std::chrono::microseconds(
        static_cast<long long>(1'000'000.0 / s.hz + 0.5));
  }
  return std::chrono::microseconds{0}; // free running
}

/// A little RAII wrapper so an early exception (a bad --csv path, a
/// rejected start_stream()) still closes the file.
struct FileCloser {
  void operator()(std::FILE *f) const noexcept {
    if (f != nullptr) {
      std::fclose(f);
    }
  }
};
using FilePtr = std::unique_ptr<std::FILE, FileCloser>;

/// Streams the selected pins to CSV (stdout, or --csv FILE) until --seconds
/// elapses or Ctrl-C, then prints a summary (achieved rate, drop counts) to
/// stderr so a plain `> file.csv` redirection stays clean CSV.
int cmd_stream(Device &dev, const std::vector<std::string> &args,
               const Settings &s) {
  require_args(args, 1,
               "stream <pins> [--hz N | --period-us N] [--seconds N] "
               "[--volts] [--csv FILE]");
  const std::vector<std::uint8_t> pins = parse_pin_list(args[0]);

  StreamConfig config;
  config.pins = pins;
  config.period = resolve_stream_period(s);
  Stream stream = dev.start_stream(config);

  FilePtr csv_guard;
  std::FILE *out = stdout;
  if (s.csv_path) {
    std::FILE *f = std::fopen(s.csv_path->c_str(), "w");
    if (f == nullptr) {
      throw UsageError(
          fmt::format("cannot open \"{}\" for writing", *s.csv_path));
    }
    csv_guard.reset(f);
    out = f;
  }
  fmt::print(out, "t_us,pin,{}\n", s.volts ? "volts" : "raw");

  std::signal(SIGINT, on_interrupt);
  const auto start = std::chrono::steady_clock::now();
  std::vector<Sample> batch(pins.size() * 16);
  std::uint64_t total_samples = 0;
  while (!StopRequested.load()) {
    if (s.run_seconds > 0.0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
                .count() >= s.run_seconds) {
      break;
    }
    const std::size_t n = stream.read(batch, 200ms);
    for (std::size_t i = 0; i < n; ++i) {
      const Sample &sample = batch[i];
      if (s.volts) {
        fmt::print(out, "{},{},{:.4f}\n", sample.t_us, sample.pin, sample.volts);
      } else {
        fmt::print(out, "{},{},{}\n", sample.t_us, sample.pin, sample.raw);
      }
    }
    total_samples += n;
  }
  std::fflush(out);

  const double elapsed = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - start)
                            .count();
  const StreamStats stats = stream.stats();
  const double rate = elapsed > 0.0 && !pins.empty()
                         ? static_cast<double>(total_samples) / elapsed /
                               static_cast<double>(pins.size())
                         : 0.0;
  fmt::print(stderr,
            "stream: {} records ({} samples) in {:.3f} s ({:.1f} Hz achieved "
            "per channel); device overruns {}, seq gaps {}, host drops {}, "
            "resyncs {}\n",
            stats.records_received, total_samples, elapsed, rate,
            stats.device_overruns, stats.seq_gaps, stats.host_drops,
            stats.resyncs);
  return ExitOk;
}

/// Arms the selected pins (EVENT_CONFIG via an EventWatcher, so ordinary
/// Device calls from elsewhere would keep working too -- there is just no
/// "elsewhere" in this CLI command) and prints each edge as it arrives until
/// --seconds elapses or Ctrl-C, then a summary: the device-side EVENT_COUNTS
/// per pin (exact even across drops) and the total the detailed feed had to
/// drop.
int cmd_watch(Device &dev, const std::vector<std::string> &args,
             const Settings &s) {
  require_args(args, 1,
               "watch <pins> [--edge rising|falling|change] "
               "[--debounce MS] [--seconds N]");
  const std::vector<std::uint8_t> pins = parse_pin_list(args[0]);

  EventWatcherConfig config;
  for (const std::uint8_t pin : pins) {
    config.pins.push_back(
        {pin, s.watch_edge, std::chrono::milliseconds(s.watch_debounce_ms)});
  }

  const auto start = std::chrono::steady_clock::now();
  EventWatcher watcher(dev, config, [&](const PinEvent &event) {
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
    fmt::print("{:>9.3f}s  pin {:<3} {:<7} (device t={} ms, edge #{})\n",
               elapsed, event.pin, to_string(event.edge), event.t_ms,
               event.seq);
    std::fflush(stdout);
  });

  std::signal(SIGINT, on_interrupt);
  while (!StopRequested.load()) {
    if (s.run_seconds > 0.0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
                .count() >= s.run_seconds) {
      break;
    }
    std::this_thread::sleep_for(50ms);
  }

  // Read the authoritative per-pin counters while the pins are still armed
  // (stop() unwatches them, which clears each counter on the device).
  const std::vector<EventCount> counts = dev.event_counts();
  const EventWatcherStats watch_stats = watcher.stats();
  watcher.stop();

  fmt::print("\nwatch summary:\n");
  for (const std::uint8_t pin : pins) {
    const auto it =
        std::find_if(counts.begin(), counts.end(),
                    [pin](const EventCount &c) { return c.pin == pin; });
    fmt::print("  pin {:<3} count {}\n", pin,
               it != counts.end() ? it->count : 0);
  }
  fmt::print("delivered {} events, {} dropped from the detailed feed\n",
            watch_stats.events_delivered, watch_stats.dropped);
  return ExitOk;
}

constexpr std::string_view Commands = R"(Commands:
  list                      devices (bus:addr vid:pid board serial itf pins)
  info                      GET_INFO of the selected device
  caps                      capability table (D digital, A analog, P PWM, C DAC)
  mode <pin> <mode>         input|output|pullup|pulldown|analog|pwm|dac
  read <pin>                digital level (pin in INPUT*/OUTPUT mode)
  write <pin> <0|1|high|low>
  aread <pin> [--volts]     analog sample (pin in ANALOG_IN mode)
  readall                   all digital levels, pin 0 first
  areadall [--volts]        all analog samples, as <pin>=<value>
  pwm <pin> <duty|N%>       raw duty code, or a percentage with a % suffix
  dac <pin> <code|N.NV>     raw DAC code, or volts with a V suffix
  status                    last error and queue occupancy (GET_STATUS)
  sync                      wait until the command queue is empty
  reset                     every DIO pin back to INPUT, queue cleared
  time                      device millis/micros, reconstructed 64-bit clock,
                            and the round-trip-bounded host offset estimate
  monitor [--hz N] [pins]   print levels/samples until Ctrl-C (set modes first)
  stream <pins> [--hz N | --period-us N] [--seconds N] [--volts] [--csv FILE]
                            continuous sampling over the bulk endpoint; pins
                            are comma-separated and must already be in
                            analog|input mode; CSV to stdout (or --csv FILE),
                            summary (rate, drops) to stderr; runs until
                            --seconds elapses or Ctrl-C
  watch <pins> [--edge rising|falling|change] [--debounce MS] [--seconds N]
                            print pin edges as they arrive (EVENT_CONFIG +
                            EVENT_POP); pins are comma-separated and must
                            already be in an INPUT* mode; final summary is
                            per-pin counts (EVENT_COUNTS) and any dropped
                            count; runs until --seconds elapses or Ctrl-C
)";

Device open_selected(const std::shared_ptr<Context> &context,
                     const Settings &s) {
  if (s.serial) {
    return open_by_serial(context, *s.serial, s.enumerate, s.device,
                          s.transport);
  }
  return open_first(context, s.enumerate, s.device, s.transport);
}

int run(int argc, char **argv) {
  cxxopts::Options options(
      "arduino-io",
      "Control an Arduino board running the UsbIo firmware over USB.");
  options.positional_help("<command> [args...]");
  options.custom_help("[options]");
  options.add_options()("s,serial", "select the device by USB serial number",
                        cxxopts::value<std::string>())(
      "vid", "USB vendor id to filter and probe (hex)",
      cxxopts::value<std::string>())("pid",
                                     "USB product id to filter and probe (hex)",
                                     cxxopts::value<std::string>())(
      "t,timeout", "control transfer timeout, ms",
      cxxopts::value<unsigned>()->default_value("100"))(
      "interface-recipient",
      "use the interface-recipient request form (bmRequestType 0x41/0xC1)")(
      "no-claim", "do not claim the vendor interface")(
      "volts", "print analog values in volts")(
      "hz", "monitor refresh rate, or stream sample rate",
      cxxopts::value<double>()->default_value("10"))(
      "period-us", "stream sampling period, microseconds (0 = free running)",
      cxxopts::value<unsigned>())(
      "seconds", "stream/watch duration; omit to run until Ctrl-C",
      cxxopts::value<double>())(
      "csv", "stream: write CSV here instead of stdout",
      cxxopts::value<std::string>())(
      "edge", "watch: edge(s) to report",
      cxxopts::value<std::string>()->default_value("change"))(
      "debounce", "watch: debounce window, ms (0..255)",
      cxxopts::value<unsigned>()->default_value("0"))(
      "v,verbose", "libusb debug output on stderr")("h,help", "show this help")(
      "args", "command and arguments",
      cxxopts::value<std::vector<std::string>>());
  options.parse_positional({"args"});

  cxxopts::ParseResult parsed;
  try {
    parsed = options.parse(argc, argv);
  } catch (const cxxopts::exceptions::exception &e) {
    throw UsageError(e.what());
  }
  if (parsed.count("help") != 0) {
    fmt::print("{}\n{}", options.help(), Commands);
    return ExitOk;
  }

  Settings s;
  if (parsed.count("serial") != 0) {
    s.serial = parsed["serial"].as<std::string>();
  }
  if (parsed.count("vid") != 0) {
    s.enumerate.vid = parse_id(parsed["vid"].as<std::string>(), "vid");
  }
  if (parsed.count("pid") != 0) {
    s.enumerate.pid = parse_id(parsed["pid"].as<std::string>(), "pid");
  }
  const unsigned timeout = parsed["timeout"].as<unsigned>();
  s.device.timeout = std::chrono::milliseconds(timeout);
  s.enumerate.probe_timeout = s.device.timeout;
  s.transport.recipient = parsed.count("interface-recipient") != 0
                              ? Recipient::Interface
                              : Recipient::Device;
  s.transport.claim_interface = parsed.count("no-claim") == 0;
  s.volts = parsed.count("volts") != 0;
  s.hz = parsed["hz"].as<double>();
  s.hz_given = parsed.count("hz") != 0;
  if (parsed.count("period-us") != 0) {
    s.period_us = parsed["period-us"].as<unsigned>();
  }
  if (parsed.count("seconds") != 0) {
    s.run_seconds = parsed["seconds"].as<double>();
  }
  if (parsed.count("csv") != 0) {
    s.csv_path = parsed["csv"].as<std::string>();
  }
  s.watch_edge = parse_edge_mode(parsed["edge"].as<std::string>());
  s.watch_debounce_ms = parsed["debounce"].as<unsigned>();
  s.verbose = parsed.count("verbose") != 0;

  std::vector<std::string> args;
  if (parsed.count("args") != 0) {
    args = parsed["args"].as<std::vector<std::string>>();
  }
  if (args.empty()) {
    throw UsageError(
        fmt::format("missing command\n\n{}\n{}", options.help(), Commands));
  }
  const std::string command = lower(args.front());
  args.erase(args.begin());
  // Command name, arity and options are checked before any USB traffic.
  struct Spec {
    std::string_view name;
    std::size_t min_args;
    std::string_view usage;
  };
  constexpr std::array<Spec, 18> specs{{
      {"list", 0, "list"},
      {"info", 0, "info"},
      {"caps", 0, "caps"},
      {"mode", 2, "mode <pin> <input|output|pullup|pulldown|analog|pwm|dac>"},
      {"read", 1, "read <pin>"},
      {"write", 2, "write <pin> <0|1|high|low>"},
      {"aread", 1, "aread <pin> [--volts]"},
      {"readall", 0, "readall"},
      {"areadall", 0, "areadall [--volts]"},
      {"pwm", 2, "pwm <pin> <duty code | percent%>"},
      {"dac", 2, "dac <pin> <code | voltsV>"},
      {"status", 0, "status"},
      {"sync", 0, "sync"},
      {"reset", 0, "reset"},
      {"time", 0, "time"},
      {"monitor", 0, "monitor [--hz N] [pins...]"},
      {"stream", 1,
       "stream <pins> [--hz N | --period-us N] [--seconds N] [--volts] "
       "[--csv FILE]"},
      {"watch", 1,
       "watch <pins> [--edge rising|falling|change] [--debounce MS] "
       "[--seconds N]"},
  }};
  const auto spec = std::find_if(
      specs.begin(), specs.end(),
      [&command](const Spec &candidate) { return candidate.name == command; });
  if (spec == specs.end()) {
    throw UsageError(
        fmt::format("unknown command \"{}\"\n\n{}", command, Commands));
  }
  require_args(args, spec->min_args, spec->usage);
  if (!(s.hz > 0.0)) {
    throw UsageError("--hz must be positive");
  }
  if (s.watch_debounce_ms > MaxDebounceMs) {
    throw UsageError(fmt::format("--debounce must be 0..{}", MaxDebounceMs));
  }

  auto context = std::make_shared<Context>();
  if (s.verbose) {
    context->set_log_level(4);
  }
  if (command == "list") {
    return cmd_list(context, s);
  }

  Device dev = open_selected(context, s);
  if (command == "info") {
    return cmd_info(dev);
  }
  if (command == "caps") {
    return cmd_caps(dev);
  }
  if (command == "mode") {
    return cmd_mode(dev, args);
  }
  if (command == "read") {
    return cmd_read(dev, args);
  }
  if (command == "write") {
    return cmd_write(dev, args);
  }
  if (command == "aread") {
    return cmd_aread(dev, args, s);
  }
  if (command == "readall") {
    return cmd_readall(dev);
  }
  if (command == "areadall") {
    return cmd_areadall(dev, s);
  }
  if (command == "pwm") {
    return cmd_pwm(dev, args);
  }
  if (command == "dac") {
    return cmd_dac(dev, args);
  }
  if (command == "status") {
    return cmd_status(dev);
  }
  if (command == "sync") {
    dev.sync();
    return ExitOk;
  }
  if (command == "reset") {
    dev.reset();
    return ExitOk;
  }
  if (command == "time") {
    return cmd_time(dev);
  }
  if (command == "monitor") {
    return cmd_monitor(dev, args, s);
  }
  if (command == "stream") {
    return cmd_stream(dev, args, s);
  }
  if (command == "watch") {
    return cmd_watch(dev, args, s);
  }
  throw UsageError(
      fmt::format("unknown command \"{}\"\n\n{}", command, Commands));
}

} // namespace

int main(int argc, char **argv) {
  try {
    return run(argc, argv);
  } catch (const UsageError &e) {
    fmt::print(stderr, "arduino-io: {}\n", e.what());
    return ExitUsage;
  } catch (const std::exception &e) {
    fmt::print(stderr, "arduino-io: {}\n", e.what());
    return ExitDevice;
  }
}
