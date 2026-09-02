// Enumerator.h - finding UsbIo devices on the USB buses and opening them.
//
// A device is recognised (a) by the UsbIo vendor interface in its
// configuration descriptor (class FF, subclass 49 'I', protocol 4F 'O') or,
// for boards that cannot expose one (Renesas), (b) by an allow-listed vendor
// id plus a successful GET_INFO probe. The descriptor classification is pure
// (find_vendor_interface, is_probe_candidate) and unit-tested without libusb.
#pragma once

#include "arduino_driver/Device.h"
#include "arduino_driver/LibusbTransport.h"
#include "arduino_driver/Protocol.h"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

struct libusb_device;

namespace ArduinoDriver {

// ---- Pure classification ----------------------------------------------------

/// Number and class triple of one interface of a configuration descriptor.
struct InterfaceTriple {
  std::uint8_t number;
  std::uint8_t usb_class;
  std::uint8_t subclass;
  std::uint8_t protocol;
};

/// Interface number of the UsbIo vendor interface, when the configuration
/// has one (first match).
std::optional<std::uint8_t>
find_vendor_interface(std::span<const InterfaceTriple> interfaces) noexcept;

struct EnumerateOptions {
  /// Restrict to this vendor / product id; also marks matching devices as
  /// probe candidates even when their vendor id is not allow-listed.
  std::optional<std::uint16_t> vid;
  std::optional<std::uint16_t> pid;
  /// Open every candidate to read its strings and GET_INFO. With false, only
  /// devices carrying the vendor interface are listed, without strings.
  bool probe{true};
  std::chrono::milliseconds probe_timeout{100};
};

/// True when vid/pid pass the explicit filter of `options` (none: always).
bool matches_filter(std::uint16_t vid, std::uint16_t pid,
                    const EnumerateOptions &options) noexcept;

/// True when a device without the vendor interface deserves a GET_INFO
/// probe: it passes the filter and either an explicit vid/pid was given or
/// its vendor id is one of USBIO_VID_ARDUINO / RASPBERRY_PI / ESPRESSIF.
bool is_probe_candidate(std::uint16_t vid, std::uint16_t pid,
                        const EnumerateOptions &options) noexcept;

/// True when a probe candidate whose GET_INFO probe failed with this libusb
/// error code should stay listed (unidentified): the device could not even
/// be opened (LIBUSB_ERROR_ACCESS, NOT_SUPPORTED, BUSY), so it may or may
/// not run UsbIo. Any other failure (STALL, timeout, ...) means it answered
/// and is not one of ours.
bool keep_unprobed_candidate(int libusb_code) noexcept;

// ---- Enumeration ------------------------------------------------------------

/// Reference-counted libusb_device (libusb_ref_device / libusb_unref_device).
using DeviceRef = std::shared_ptr<libusb_device>;

/// One device found by list_devices().
struct DeviceInfo {
  std::uint16_t vid{0};
  std::uint16_t pid{0};
  std::uint8_t bus{0};
  std::uint8_t address{0};
  std::string serial;       ///< empty when not probed or not readable
  std::string manufacturer; ///< idem
  std::string product;      ///< idem
  /// True when the device is known to run UsbIo: it carries the vendor
  /// interface, or the GET_INFO probe succeeded. False for a candidate that
  /// could not be opened (see probe_error): it may or may not be ours.
  bool identified{false};
  bool has_vendor_interface{false};
  std::optional<std::uint8_t> interface_number;
  /// GET_INFO reply when the probe succeeded (n_pins == 0: not ready yet).
  std::optional<Info> info;
  /// Why the probe failed, for devices identified by their descriptor
  /// (typically a permission problem); empty otherwise.
  std::string probe_error;
  /// Context the entry belongs to; keeps it alive for `device`.
  std::shared_ptr<Context> context;
  /// Keeps the libusb_device alive so that the entry can be reopened later.
  /// Null in hand-built entries: open_transport() then re-finds the device
  /// by bus/address and vid/pid.
  DeviceRef device;
};

/// Scans the buses. Devices with the vendor interface are always listed
/// (with `probe_error` set when they could not be opened); other candidates
/// are listed when the GET_INFO probe succeeds (`identified`) or when they
/// could not be opened at all (`identified` false, `probe_error` set, see
/// keep_unprobed_candidate()). Throws UsbError when libusb cannot enumerate.
std::vector<DeviceInfo> list_devices(std::shared_ptr<Context> context,
                                     EnumerateOptions options = {});

/// Opens a transport to a listed device. Uses `info.device` when it belongs
/// to `context`, otherwise re-finds the device by bus/address and vid/pid
/// (DeviceNotFound when it is gone).
std::unique_ptr<LibusbTransport>
open_transport(std::shared_ptr<Context> context, const DeviceInfo &info,
               LibusbTransportOptions options = {});

/// open_transport() + Device on an entry returned by list_devices()
/// (std::invalid_argument when the entry has no context).
Device open_device(const DeviceInfo &info, DeviceOptions device_options = {},
                   LibusbTransportOptions transport_options = {});

/// Opens the first identified device (DeviceNotFound when there is none; the
/// message lists the candidates that could not be probed).
Device open_first(std::shared_ptr<Context> context,
                  EnumerateOptions enumerate_options = {},
                  DeviceOptions device_options = {},
                  LibusbTransportOptions transport_options = {});

/// Opens the identified device with this USB serial number (DeviceNotFound
/// otherwise).
Device open_by_serial(std::shared_ptr<Context> context, std::string_view serial,
                      EnumerateOptions enumerate_options = {},
                      DeviceOptions device_options = {},
                      LibusbTransportOptions transport_options = {});

} // namespace ArduinoDriver
