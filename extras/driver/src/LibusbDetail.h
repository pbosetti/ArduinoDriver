// LibusbDetail.h - helpers shared by LibusbTransport.cpp and Enumerator.cpp.
// Internal: not part of the public include directory.
#pragma once

#include "arduino_driver/Enumerator.h"

#include <libusb.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ArduinoDriver::Detail {

/// "LIBUSB_ERROR_ACCESS (Access denied (insufficient permissions))".
std::string describe_usb_error(int code);

/// Throws the exception matching a negative libusb return code: StallError
/// for LIBUSB_ERROR_PIPE, TimeoutError for LIBUSB_ERROR_TIMEOUT, UsbError
/// otherwise. `context` prefixes the message.
[[noreturn]] void throw_usb_error(int code, const std::string &context);

/// Interfaces (every alternate setting) of the active configuration, or of
/// the first one when the device is not configured; empty when no
/// configuration descriptor can be read. Does not open the device.
std::vector<InterfaceTriple> read_interfaces(libusb_device *device);

/// Address and max packet size of one endpoint of a config descriptor.
struct BulkEndpoint {
  std::uint8_t address;        ///< bEndpointAddress (direction bit set)
  std::uint16_t max_packet_size;
};

/// The first bulk IN endpoint of altsetting 0 of `interface_number`, from the
/// same config descriptor read_interfaces() uses. std::nullopt when the
/// descriptor cannot be read or the interface has no bulk IN endpoint. Does
/// not open the device.
std::optional<BulkEndpoint> find_bulk_in_endpoint(libusb_device *device,
                                                   std::uint8_t interface_number);

} // namespace ArduinoDriver::Detail
