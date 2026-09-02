// Stream.h - continuous sampling over the bulk IN endpoint (Phase 2).
//
// Device::start_stream() builds the channel selection, starts the device
// sampling and returns a Stream: an RAII handle around a background worker
// thread that pulls bytes off Transport::bulk_in(), reassembles
// usbio_stream_header_t + samples records out of the continuous byte stream
// (records may straddle packet boundaries, and the worker resyncs on
// USBIO_STREAM_MAGIC after any loss), and hands decoded Sample values to the
// consumer through read() and/or an on_records() callback.
//
// Threading contract
// -------------------
// Device stays NOT thread-safe. Starting a Stream marks its Device "busy":
// every other Device method (pin_mode, digital_read/write, analog_read,
// read_all_*, pwm/dac writes, status(), sync(), reset()) throws DeviceBusy
// for as long as the Stream is running, because the device firmware itself
// treats STREAM_SELECT/STREAM_START/STREAM_STOP/GET_STREAM_STATUS specially
// while sampling (see usbio_protocol.h). The two exceptions -
// GET_STREAM_STATUS (polled internally for stats()) and STREAM_STOP (issued
// by stop() / the destructor) - are serialised inside Device, so it is safe
// to call Stream::stop() (or drop the Stream) from a different thread than
// the one driving the rest of Device's API, as long as no other Device call
// races with it.
//
// A Stream must not outlive the Device that created it (start_stream()
// returns a handle into that Device, like an iterator into its container).
#pragma once

#include "arduino_driver/Protocol.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <vector>

namespace ArduinoDriver {

class Device;

/// One decoded channel reading from a stream record.
struct Sample {
  std::uint8_t pin{0};    ///< the pin this sample belongs to
  std::uint16_t raw{0};   ///< 0 .. 2^adc_bits-1 (analog) or 0/1 (digital)
  double volts{0.0};      ///< Device::to_volts(raw); only meaningful for an
                          ///< analog pin
  std::uint32_t t_us{0};  ///< device micros() when the record was sampled
};

/// Pins, rate and flags for Device::start_stream().
struct StreamConfig {
  /// Pins to sample, in the order samples appear within each record. Must be
  /// non-empty and no longer than Info::stream_max_channels; every pin must
  /// already be in ANALOG_IN or an INPUT* mode (else the device STALLs
  /// STREAM_SELECT with BAD_MODE, surfaced as InvalidMode).
  std::vector<std::uint8_t> pins;
  /// Sampling period; 0 means free-running. Otherwise clamped to
  /// StreamMinPeriodUs .. 65535 us (else InvalidValue).
  std::chrono::microseconds period{0};
  /// enum usbio_stream_flags bits (StreamFlags::Digital / StopOnOverrun).
  std::uint8_t flags{0};
};

/// Counters accumulated by a Stream since it started.
struct StreamStats {
  std::uint32_t device_overruns{0}; ///< usbio_stream_status_t.overruns, as of
                                    ///< the last internal poll
  std::uint64_t records_received{0}; ///< records successfully decoded
  std::uint64_t seq_gaps{0};    ///< records lost before reaching the host,
                                ///< inferred from gaps in usbio_stream_header_t
                                ///< .seq (device-side drops and lost packets)
  std::uint64_t host_drops{0};  ///< decoded records the host had to discard
                                ///< because the consumer was not keeping up
  std::uint64_t resyncs{0};     ///< times the byte stream lost sync on
                                ///< USBIO_STREAM_MAGIC and had to resync
};

/// RAII bulk-streaming session. Move-only; the destructor stops the device
/// stream and joins the worker thread. See the threading contract above.
class Stream {
public:
  Stream(Stream &&other) noexcept;
  Stream &operator=(Stream &&other) noexcept;
  Stream(const Stream &) = delete;
  Stream &operator=(const Stream &) = delete;
  /// Stops the device stream (best effort; errors are swallowed) and joins
  /// the worker thread.
  ~Stream();

  /// Copies up to `out.size()` decoded samples (as many whole records as
  /// fit) into `out`, blocking until at least one is available or `timeout`
  /// elapses. Returns the number of samples copied (0 on timeout). Samples
  /// from one record are always copied contiguously and in full: if the
  /// next queued record has more channels than the room left in `out`, it
  /// is left queued for the following call.
  std::size_t read(std::span<Sample> out, std::chrono::milliseconds timeout);

  /// Called from the worker thread with every decoded record's samples
  /// (contiguous, `pins().size()` long), in addition to whatever read()
  /// also delivers. Keep it fast: it runs inline in the worker's decode
  /// loop. Not set by default.
  using RecordCallback = std::function<void(std::span<const Sample>)>;
  void on_records(RecordCallback callback);

  /// Snapshot of the counters accumulated so far. Safe to call from any
  /// thread at any time.
  StreamStats stats() const;

  /// True until stop() has completed (or the destructor has run).
  bool running() const noexcept;

  /// Pins in selection order, as given to Device::start_stream().
  const std::vector<std::uint8_t> &pins() const noexcept;

  /// Stops the device stream and joins the worker thread. Idempotent; the
  /// destructor calls it if the caller does not. Safe to call from any
  /// thread.
  void stop();

private:
  friend class Device;
  Stream(Device &device, StreamConfig config);

  // Trampolines through which Impl (a nested class: it has the same access
  // to Stream's own members as Stream itself, but does not inherit Stream's
  // friendship with Device) reaches Device's two streaming-only private
  // methods.
  static StreamStatus poll_status(Device &device);
  static void end(Device &device) noexcept;

  struct Impl;
  std::unique_ptr<Impl> _impl;
};

} // namespace ArduinoDriver
