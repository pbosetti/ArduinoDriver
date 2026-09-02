// LibusbDetail.h - helpers shared by LibusbTransport.cpp and Enumerator.cpp.
// Internal: not part of the public include directory.
#pragma once

#include "arduino_driver/Enumerator.h"

#include <libusb.h>

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

} // namespace ArduinoDriver::Detail
