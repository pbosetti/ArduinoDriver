// LibusbTransport.h - Transport over libusb-1.0 (real hardware).
#pragma once

#include "arduino_driver/Errors.h"
#include "arduino_driver/Protocol.h"
#include "arduino_driver/Transport.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>

// libusb.h stays out of the public headers; these match its typedefs.
struct libusb_context;
struct libusb_device;
struct libusb_device_handle;

namespace ArduinoDriver {

/// RAII libusb_context. Create one per process (std::make_shared<Context>())
/// and hand it to the enumeration functions and transports, which keep it
/// alive for as long as they need it.
class Context {
public:
  /// Throws UsbError when libusb cannot be initialised.
  Context();
  ~Context();
  Context(const Context &) = delete;
  Context &operator=(const Context &) = delete;

  libusb_context *native() const noexcept { return _context; }
  /// libusb message verbosity: 0 none, 1 error, 2 warning, 3 info, 4 debug.
  void set_log_level(int level);

private:
  libusb_context *_context{nullptr};
};

struct LibusbTransportOptions {
  /// Request form; Interface needs the vendor interface (see Recipient).
  Recipient recipient{Recipient::Device};
  /// Claim the vendor interface when the device has one. Claiming gives
  /// exclusivity between processes using the board, is required for the
  /// Interface recipient form and on Windows (WinUSB handles are per
  /// interface). It is optional on macOS and Linux: EP0 vendor requests
  /// travel on the device handle, which libusb opens even when another
  /// driver (CDC) holds the other interfaces.
  bool claim_interface{true};
};

/// Vendor control transfers to one open libusb device. Opens the device on
/// construction, claims the UsbIo vendor interface when present (and
/// requested), releases and closes on destruction.
class LibusbTransport final : public Transport {
public:
  /// Throws std::invalid_argument for null arguments, NotSupported when the
  /// Interface form is requested on a device without the vendor interface,
  /// UsbError when the device cannot be opened or the interface claimed
  /// (permissions: udev rule on Linux, WinUSB binding on Windows).
  LibusbTransport(std::shared_ptr<Context> context, libusb_device *device,
                  LibusbTransportOptions options = {});
  ~LibusbTransport() override;

  std::size_t control_in(std::uint8_t request, std::uint16_t value,
                         std::uint16_t index, std::span<std::byte> data,
                         std::chrono::milliseconds timeout) override;
  void control_out(std::uint8_t request, std::uint16_t value,
                   std::uint16_t index,
                   std::chrono::milliseconds timeout) override;
  /// Reads decoded bytes off an internal ring of ~8 in-flight bulk transfers
  /// (a few packets each), kept continuously resubmitted so the device
  /// endpoint is never left waiting for the host to arm a read; this method
  /// itself just drains what has arrived, pumping libusb's event loop
  /// (libusb_handle_events_timeout) until data is available or `timeout`
  /// elapses. The ring is created lazily on the first call and lives until
  /// the transport is destroyed. Throws NotSupported when the device has no
  /// bulk IN endpoint (has_bulk_in() is false), UsbError on a fatal transfer
  /// error (e.g. the device was unplugged).
  std::size_t bulk_in(std::span<std::byte> data,
                      std::chrono::milliseconds timeout) override;

  /// True when the vendor interface descriptor carries a bulk IN endpoint.
  bool has_bulk_in() const noexcept { return _bulk_in_ep.has_value(); }
  /// Endpoint address (with the IN bit set), when has_bulk_in().
  std::optional<std::uint8_t> bulk_in_endpoint() const noexcept {
    return _bulk_in_ep;
  }
  /// wMaxPacketSize of the bulk IN endpoint, 0 when has_bulk_in() is false.
  std::uint16_t bulk_in_max_packet_size() const noexcept {
    return _bulk_in_max_packet;
  }

  /// True when the configuration descriptor carries the UsbIo interface.
  bool has_vendor_interface() const noexcept { return _interface.has_value(); }
  /// Number of the UsbIo interface, when present.
  std::optional<std::uint8_t> interface_number() const noexcept {
    return _interface;
  }
  /// True when the interface was claimed by this transport.
  bool interface_claimed() const noexcept { return _claimed; }
  Recipient recipient() const noexcept { return _options.recipient; }
  const std::shared_ptr<Context> &context() const noexcept { return _context; }
  libusb_device_handle *native_handle() const noexcept { return _handle; }

private:
  /// One libusb_control_transfer with error mapping; returns bytes moved.
  int transfer(std::uint8_t request_type, std::uint8_t request,
               std::uint16_t value, std::uint16_t index,
               std::span<std::byte> data, std::chrono::milliseconds timeout);
  /// wIndex on the wire for the selected recipient form.
  std::uint16_t wire_index(std::uint16_t index) const;
  void close() noexcept;

  class BulkRing; // async ring of bulk IN transfers backing bulk_in()

  std::shared_ptr<Context> _context;
  LibusbTransportOptions _options;
  libusb_device_handle *_handle{nullptr};
  std::optional<std::uint8_t> _interface;
  bool _claimed{false};
  std::optional<std::uint8_t> _bulk_in_ep;
  std::uint16_t _bulk_in_max_packet{0};
  std::unique_ptr<BulkRing> _bulk_ring;
};

} // namespace ArduinoDriver
