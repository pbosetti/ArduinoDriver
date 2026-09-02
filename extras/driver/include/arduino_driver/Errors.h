// Errors.h - exception hierarchy of the ArduinoDriver host library.
//
// Every failure is reported by throwing. All classes derive from Error, which
// derives from std::runtime_error, so `catch (const ArduinoDriver::Error &)`
// catches everything the library raises.
#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

namespace ArduinoDriver {

/// Base class of every exception thrown by the library.
class Error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

/// libusb error codes (`enum libusb_error`, a stable part of the libusb ABI),
/// mirrored here so that the header-only parts of the library and the unit
/// tests do not depend on libusb.h. LibusbTransport static_asserts them
/// against the real header.
namespace LibusbError {
inline constexpr int Success = 0;
inline constexpr int Io = -1;
inline constexpr int InvalidParam = -2;
inline constexpr int Access = -3;
inline constexpr int NoDevice = -4;
inline constexpr int NotFound = -5;
inline constexpr int Busy = -6;
inline constexpr int Timeout = -7;
inline constexpr int Overflow = -8;
inline constexpr int Pipe = -9;
inline constexpr int Interrupted = -10;
inline constexpr int NoMem = -11;
inline constexpr int NotSupported = -12;
inline constexpr int Other = -99;
} // namespace LibusbError

/// Name of a libusb error code, e.g. "LIBUSB_ERROR_PIPE".
constexpr std::string_view usb_error_name(int code) noexcept {
  switch (code) {
  case LibusbError::Success:
    return "LIBUSB_SUCCESS";
  case LibusbError::Io:
    return "LIBUSB_ERROR_IO";
  case LibusbError::InvalidParam:
    return "LIBUSB_ERROR_INVALID_PARAM";
  case LibusbError::Access:
    return "LIBUSB_ERROR_ACCESS";
  case LibusbError::NoDevice:
    return "LIBUSB_ERROR_NO_DEVICE";
  case LibusbError::NotFound:
    return "LIBUSB_ERROR_NOT_FOUND";
  case LibusbError::Busy:
    return "LIBUSB_ERROR_BUSY";
  case LibusbError::Timeout:
    return "LIBUSB_ERROR_TIMEOUT";
  case LibusbError::Overflow:
    return "LIBUSB_ERROR_OVERFLOW";
  case LibusbError::Pipe:
    return "LIBUSB_ERROR_PIPE";
  case LibusbError::Interrupted:
    return "LIBUSB_ERROR_INTERRUPTED";
  case LibusbError::NoMem:
    return "LIBUSB_ERROR_NO_MEM";
  case LibusbError::NotSupported:
    return "LIBUSB_ERROR_NOT_SUPPORTED";
  case LibusbError::Other:
    return "LIBUSB_ERROR_OTHER";
  default:
    return "LIBUSB_ERROR_UNKNOWN";
  }
}

/// A USB transfer failed at the libusb level.
class UsbError : public Error {
public:
  UsbError(const std::string &what, int code) : Error(what), _code(code) {}

  /// The libusb error code (see LibusbError).
  int code() const noexcept { return _code; }
  /// The libusb name of the code, e.g. "LIBUSB_ERROR_TIMEOUT".
  std::string_view name() const noexcept { return usb_error_name(_code); }

private:
  int _code;
};

/// The device STALLed the request (LIBUSB_ERROR_PIPE). For an OUT request
/// this means the firmware rejected the command; Device translates it into
/// the specific exception below by reading GET_STATUS.last_error.
class StallError : public UsbError {
public:
  explicit StallError(const std::string &what)
      : UsbError(what, LibusbError::Pipe) {}
};

/// The control transfer did not complete in time (LIBUSB_ERROR_TIMEOUT).
class TimeoutError : public UsbError {
public:
  explicit TimeoutError(const std::string &what)
      : UsbError(what, LibusbError::Timeout) {}
};

/// The device answered with something the protocol does not allow: bad magic,
/// unsupported protocol version, short or empty reply, unexpected status.
class ProtocolError : public Error {
public:
  using Error::Error;
};

/// The device kept answering BUSY (or, in sync(), kept a non-empty queue)
/// for Device::Options::busy_max_attempts attempts.
class DeviceBusy : public Error {
public:
  using Error::Error;
};

/// Pin index outside 0 .. pin_count()-1.
class InvalidPin : public Error {
public:
  using Error::Error;
};

/// Unknown mode, or request not valid for the pin's current mode (for
/// example DIO_WRITE on a pin that is not in OUTPUT mode).
class InvalidMode : public Error {
public:
  using Error::Error;
};

/// Value outside the accepted range (duty, DAC code, fraction, voltage).
class InvalidValue : public Error {
public:
  using Error::Error;
};

/// The pin, or the board, lacks the capability the call needs.
class NotSupported : public Error {
public:
  using Error::Error;
};

/// The firmware command queue is full; call sync() and retry.
class QueueFull : public Error {
public:
  using Error::Error;
};

/// GET_INFO kept reporting n_pins == 0: the device enumerated but the sketch
/// has not called UsbIo.begin() yet (see DeviceOptions::ready_max_attempts).
class NotReady : public Error {
public:
  using Error::Error;
};

/// No USB device matched the enumeration filter / serial number.
class DeviceNotFound : public Error {
public:
  using Error::Error;
};

} // namespace ArduinoDriver
