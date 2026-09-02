// Transport.h - abstract vendor control-transfer channel to a UsbIo device.
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

namespace ArduinoDriver {

/// One control-transfer channel to a device. The protocol fixes
/// bmRequestType to USBIO_REQTYPE_IN / USBIO_REQTYPE_OUT (vendor, device
/// recipient), so implementations add it themselves; callers pass only
/// bRequest, wValue and wIndex.
///
/// Errors are reported by throwing: UsbError for any libusb failure, with
/// the subclasses StallError (LIBUSB_ERROR_PIPE: the device rejected the
/// request) and TimeoutError (LIBUSB_ERROR_TIMEOUT).
///
/// Implementations: LibusbTransport (real hardware) and, in the tests,
/// FakeTransport (an in-process model of the firmware).
class Transport {
public:
  virtual ~Transport() = default;
  Transport(const Transport &) = delete;
  Transport &operator=(const Transport &) = delete;

  /// IN transfer. `data.size()` is sent as wLength; returns the number of
  /// bytes actually received (0 .. data.size()). A short reply is legal on
  /// the wire: the caller validates the length.
  virtual std::size_t control_in(std::uint8_t request, std::uint16_t value,
                                 std::uint16_t index, std::span<std::byte> data,
                                 std::chrono::milliseconds timeout) = 0;

  /// OUT transfer without data stage (wLength = 0).
  virtual void control_out(std::uint8_t request, std::uint16_t value,
                           std::uint16_t index,
                           std::chrono::milliseconds timeout) = 0;

protected:
  Transport() = default;
};

} // namespace ArduinoDriver
