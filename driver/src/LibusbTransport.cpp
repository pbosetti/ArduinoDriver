// LibusbTransport.cpp - Context, LibusbTransport and the shared libusb helpers.
#include "arduino_driver/LibusbTransport.h"

#include "LibusbDetail.h"
#include "arduino_driver/Enumerator.h"

#include <fmt/format.h>
#include <libusb.h>

#include <climits>
#include <stdexcept>
#include <utility>

namespace ArduinoDriver {

// Errors.h mirrors these so that the header-only code stays libusb-free.
static_assert(LibusbError::Success == LIBUSB_SUCCESS);
static_assert(LibusbError::Io == LIBUSB_ERROR_IO);
static_assert(LibusbError::InvalidParam == LIBUSB_ERROR_INVALID_PARAM);
static_assert(LibusbError::Access == LIBUSB_ERROR_ACCESS);
static_assert(LibusbError::NoDevice == LIBUSB_ERROR_NO_DEVICE);
static_assert(LibusbError::NotFound == LIBUSB_ERROR_NOT_FOUND);
static_assert(LibusbError::Busy == LIBUSB_ERROR_BUSY);
static_assert(LibusbError::Timeout == LIBUSB_ERROR_TIMEOUT);
static_assert(LibusbError::Overflow == LIBUSB_ERROR_OVERFLOW);
static_assert(LibusbError::Pipe == LIBUSB_ERROR_PIPE);
static_assert(LibusbError::Interrupted == LIBUSB_ERROR_INTERRUPTED);
static_assert(LibusbError::NoMem == LIBUSB_ERROR_NO_MEM);
static_assert(LibusbError::NotSupported == LIBUSB_ERROR_NOT_SUPPORTED);
static_assert(LibusbError::Other == LIBUSB_ERROR_OTHER);

// ---- Shared helpers ---------------------------------------------------------

namespace Detail {

std::string describe_usb_error(int code) {
  return fmt::format("{} ({})", libusb_error_name(code),
                     libusb_strerror(static_cast<libusb_error>(code)));
}

void throw_usb_error(int code, const std::string &context) {
  const std::string what =
      fmt::format("{}: {}", context, describe_usb_error(code));
  switch (code) {
  case LIBUSB_ERROR_PIPE:
    throw StallError(what);
  case LIBUSB_ERROR_TIMEOUT:
    throw TimeoutError(what);
  default:
    throw UsbError(what, code);
  }
}

std::vector<InterfaceTriple> read_interfaces(libusb_device *device) {
  std::vector<InterfaceTriple> interfaces;
  libusb_config_descriptor *config = nullptr;
  int rc = libusb_get_active_config_descriptor(device, &config);
  if (rc < 0) {
    rc = libusb_get_config_descriptor(device, 0, &config);
  }
  if (rc < 0 || config == nullptr) {
    return interfaces;
  }
  for (int i = 0; i < config->bNumInterfaces; ++i) {
    const libusb_interface &interface = config->interface[i];
    for (int alt = 0; alt < interface.num_altsetting; ++alt) {
      const libusb_interface_descriptor &d = interface.altsetting[alt];
      interfaces.push_back({d.bInterfaceNumber, d.bInterfaceClass,
                            d.bInterfaceSubClass, d.bInterfaceProtocol});
    }
  }
  libusb_free_config_descriptor(config);
  return interfaces;
}

} // namespace Detail

namespace {

std::string describe_device(libusb_device *device) {
  libusb_device_descriptor desc{};
  libusb_get_device_descriptor(device, &desc);
  return fmt::format("USB device {:04X}:{:04X} (bus {}, address {})",
                     desc.idVendor, desc.idProduct,
                     libusb_get_bus_number(device),
                     libusb_get_device_address(device));
}

/// What the user can do about a permission-like failure.
const char *access_hint(int code) {
  switch (code) {
  case LIBUSB_ERROR_ACCESS:
    return " - on Linux install etc/99-arduino-usbio.rules (udev); on "
           "Windows bind WinUSB to the UsbIo interface with Zadig";
  case LIBUSB_ERROR_NOT_SUPPORTED:
    return " - on Windows the UsbIo interface needs the WinUSB driver "
           "(bind it with Zadig)";
  case LIBUSB_ERROR_BUSY:
    return " - another program holds the interface";
  default:
    return "";
  }
}

unsigned clamp_timeout(std::chrono::milliseconds timeout) noexcept {
  const auto ms = timeout.count();
  if (ms < 1) {
    return 1u; // 0 would mean "wait forever" to libusb
  }
  if (ms > static_cast<decltype(ms)>(UINT_MAX)) {
    return UINT_MAX;
  }
  return static_cast<unsigned>(ms);
}

} // namespace

// ---- Context ----------------------------------------------------------------

Context::Context() {
#if defined(LIBUSB_API_VERSION) && LIBUSB_API_VERSION >= 0x0100010A
  const int rc = libusb_init_context(&_context, nullptr, 0);
#else
  const int rc = libusb_init(&_context);
#endif
  if (rc < 0) {
    Detail::throw_usb_error(rc, "cannot initialise libusb");
  }
}

Context::~Context() {
  if (_context != nullptr) {
    libusb_exit(_context);
  }
}

void Context::set_log_level(int level) {
  const int rc = libusb_set_option(_context, LIBUSB_OPTION_LOG_LEVEL, level);
  if (rc < 0 && rc != LIBUSB_ERROR_NOT_SUPPORTED) {
    Detail::throw_usb_error(rc, "cannot set the libusb log level");
  }
}

// ---- LibusbTransport --------------------------------------------------------

LibusbTransport::LibusbTransport(std::shared_ptr<Context> context,
                                 libusb_device *device,
                                 LibusbTransportOptions options)
    : _context(std::move(context)), _options(options) {
  if (!_context) {
    throw std::invalid_argument("LibusbTransport: context must not be null");
  }
  if (device == nullptr) {
    throw std::invalid_argument("LibusbTransport: device must not be null");
  }

  // The descriptor is readable without opening the device.
  _interface = find_vendor_interface(Detail::read_interfaces(device));
  if (_options.recipient == Recipient::Interface && !_interface) {
    throw NotSupported(fmt::format(
        "{} has no UsbIo vendor interface: the interface-recipient request "
        "form is not available (Renesas boards cannot expose one)",
        describe_device(device)));
  }

  int rc = libusb_open(device, &_handle);
  if (rc < 0) {
    _handle = nullptr;
    Detail::throw_usb_error(rc, fmt::format("cannot open {}{}",
                                            describe_device(device),
                                            access_hint(rc)));
  }
  try {
    if (_interface && _options.claim_interface) {
      // Linux: let libusb detach a kernel driver bound to the interface;
      // elsewhere this is NOT_SUPPORTED and harmless.
      libusb_set_auto_detach_kernel_driver(_handle, 1);
      rc = libusb_claim_interface(_handle, *_interface);
      if (rc < 0) {
        Detail::throw_usb_error(
            rc,
            fmt::format("cannot claim UsbIo interface {} of {}{}", *_interface,
                        describe_device(device), access_hint(rc)));
      }
      _claimed = true;
    }
  } catch (...) {
    close();
    throw;
  }
}

LibusbTransport::~LibusbTransport() { close(); }

void LibusbTransport::close() noexcept {
  if (_handle == nullptr) {
    return;
  }
  if (_claimed) {
    libusb_release_interface(_handle, *_interface);
    _claimed = false;
  }
  libusb_close(_handle);
  _handle = nullptr;
}

std::size_t LibusbTransport::control_in(std::uint8_t request,
                                        std::uint16_t value,
                                        std::uint16_t index,
                                        std::span<std::byte> data,
                                        std::chrono::milliseconds timeout) {
  const int n = transfer(bm_request_type(false, _options.recipient), request,
                         value, wire_index(index), data, timeout);
  return static_cast<std::size_t>(n);
}

void LibusbTransport::control_out(std::uint8_t request, std::uint16_t value,
                                  std::uint16_t index,
                                  std::chrono::milliseconds timeout) {
  transfer(bm_request_type(true, _options.recipient), request, value,
           wire_index(index), {}, timeout);
}

std::uint16_t LibusbTransport::wire_index(std::uint16_t index) const {
  if (_options.recipient == Recipient::Device) {
    return index;
  }
  if (index > 0xFF) {
    throw std::invalid_argument(
        "LibusbTransport: wIndex does not fit the interface-recipient form");
  }
  return interface_index(index, *_interface);
}

int LibusbTransport::transfer(std::uint8_t request_type, std::uint8_t request,
                              std::uint16_t value, std::uint16_t index,
                              std::span<std::byte> data,
                              std::chrono::milliseconds timeout) {
  if (data.size() > 0xFFFF) {
    throw std::invalid_argument("LibusbTransport: wLength exceeds 65535");
  }
  const unsigned timeout_ms = clamp_timeout(timeout);
  auto *buffer = reinterpret_cast<unsigned char *>(data.data());
  const auto length = static_cast<std::uint16_t>(data.size());
  for (int attempt = 0;; ++attempt) {
    const int rc =
        libusb_control_transfer(_handle, request_type, request, value, index,
                                buffer, length, timeout_ms);
    if (rc >= 0) {
      return rc;
    }
    if (rc == LIBUSB_ERROR_INTERRUPTED && attempt == 0) {
      continue; // a signal landed in the middle of the transfer: once more
    }
    Detail::throw_usb_error(
        rc, fmt::format("control transfer bmRequestType=0x{:02X} "
                        "bRequest=0x{:02X} wValue=0x{:04X} wIndex=0x{:04X} "
                        "wLength={} failed",
                        request_type, request, value, index, length));
  }
}

} // namespace ArduinoDriver
