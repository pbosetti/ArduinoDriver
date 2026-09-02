// LibusbTransport.cpp - Context, LibusbTransport and the shared libusb helpers.
#include "arduino_driver/LibusbTransport.h"

#include "LibusbDetail.h"
#include "arduino_driver/Enumerator.h"

#include <fmt/format.h>
#include <libusb.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <climits>
#include <cstddef>
#include <deque>
#include <mutex>
#include <new>
#include <stdexcept>
#include <utility>
#include <vector>

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

std::optional<BulkEndpoint> find_bulk_in_endpoint(libusb_device *device,
                                                   std::uint8_t interface_number) {
  libusb_config_descriptor *config = nullptr;
  int rc = libusb_get_active_config_descriptor(device, &config);
  if (rc < 0) {
    rc = libusb_get_config_descriptor(device, 0, &config);
  }
  if (rc < 0 || config == nullptr) {
    return std::nullopt;
  }
  std::optional<BulkEndpoint> found;
  for (int i = 0; i < config->bNumInterfaces && !found; ++i) {
    const libusb_interface &interface = config->interface[i];
    for (int alt = 0; alt < interface.num_altsetting && !found; ++alt) {
      const libusb_interface_descriptor &d = interface.altsetting[alt];
      if (d.bInterfaceNumber != interface_number) {
        continue;
      }
      for (int e = 0; e < d.bNumEndpoints; ++e) {
        const libusb_endpoint_descriptor &ep = d.endpoint[e];
        const bool is_in =
            (ep.bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN;
        const bool is_bulk =
            (ep.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK;
        if (is_in && is_bulk) {
          found = BulkEndpoint{ep.bEndpointAddress, ep.wMaxPacketSize};
          break;
        }
      }
    }
  }
  libusb_free_config_descriptor(config);
  return found;
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

/// libusb_error matching a failed libusb_transfer::status, for reuse of
/// Detail::throw_usb_error's message and exception mapping (StallError,
/// TimeoutError, ...).
int transfer_status_error(libusb_transfer_status status) noexcept {
  switch (status) {
  case LIBUSB_TRANSFER_COMPLETED:
    return LIBUSB_SUCCESS;
  case LIBUSB_TRANSFER_ERROR:
    return LIBUSB_ERROR_IO;
  case LIBUSB_TRANSFER_TIMED_OUT:
    return LIBUSB_ERROR_TIMEOUT;
  case LIBUSB_TRANSFER_CANCELLED:
    return LIBUSB_ERROR_INTERRUPTED;
  case LIBUSB_TRANSFER_STALL:
    return LIBUSB_ERROR_PIPE;
  case LIBUSB_TRANSFER_NO_DEVICE:
    return LIBUSB_ERROR_NO_DEVICE;
  case LIBUSB_TRANSFER_OVERFLOW:
    return LIBUSB_ERROR_OVERFLOW;
  default:
    return LIBUSB_ERROR_OTHER;
  }
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

// ---- BulkRing -----------------------------------------------------------
//
// Backs LibusbTransport::bulk_in() with a handful of always-in-flight bulk
// transfers so the device's IN endpoint is never left waiting for the host
// to arm a read between calls: the moment one transfer completes, its bytes
// are queued and it is resubmitted immediately from the completion
// callback -- before bulk_in() has even returned to its caller. bulk_in()
// itself only drains the queue, pumping libusb_handle_events_timeout() to
// let completions (and therefore resubmissions) happen.
class LibusbTransport::BulkRing {
public:
  BulkRing(libusb_context *context, libusb_device_handle *handle,
           std::uint8_t endpoint, std::uint16_t max_packet_size)
      : _context(context) {
    const std::size_t chunk =
        static_cast<std::size_t>(std::max<std::uint16_t>(max_packet_size, 1)) *
        PacketsPerTransfer;
    for (Buf &buf : _bufs) {
      buf.owner = this;
      buf.data.resize(chunk);
      buf.transfer = libusb_alloc_transfer(0);
      if (buf.transfer == nullptr) {
        throw std::bad_alloc();
      }
      libusb_fill_bulk_transfer(buf.transfer, handle, endpoint, buf.data.data(),
                                static_cast<int>(buf.data.size()), on_complete,
                                &buf, 0 /* no per-transfer timeout: it is
                                        meant to sit armed indefinitely */);
    }
    for (Buf &buf : _bufs) {
      submit(buf);
    }
  }

  ~BulkRing() {
    std::unique_lock<std::mutex> lock(_mutex);
    _stopping = true;
    for (Buf &buf : _bufs) {
      if (buf.submitted) {
        libusb_cancel_transfer(buf.transfer);
      }
    }
    lock.unlock();
    // Every submitted transfer eventually completes (cancelled, or with a
    // device error): wait for all of them before freeing their buffers.
    while (_outstanding.load(std::memory_order_acquire) > 0) {
      timeval tv{};
      tv.tv_sec = 1;
      tv.tv_usec = 0;
      libusb_handle_events_timeout(_context, &tv);
    }
    for (Buf &buf : _bufs) {
      libusb_free_transfer(buf.transfer);
    }
  }

  BulkRing(const BulkRing &) = delete;
  BulkRing &operator=(const BulkRing &) = delete;

  /// Copies decoded bytes into `data`, pumping libusb events until some are
  /// available or `timeout` elapses. Returns 0 on a plain timeout; throws
  /// UsbError when a ring transfer failed fatally (e.g. the device was
  /// unplugged).
  std::size_t read(std::span<std::byte> data,
                   std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    for (;;) {
      {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_fatal_error) {
          const int code = *_fatal_error;
          _fatal_error.reset();
          Detail::throw_usb_error(code, "bulk IN transfer failed");
        }
        if (!_queue.empty()) {
          const std::size_t n = std::min(data.size(), _queue.size());
          std::copy_n(_queue.begin(), n, data.begin());
          _queue.erase(_queue.begin(),
                      _queue.begin() + static_cast<std::ptrdiff_t>(n));
          return n;
        }
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        return 0;
      }
      const auto slice = std::min(
          std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now),
          std::chrono::milliseconds{50});
      timeval tv{};
      tv.tv_sec = static_cast<decltype(tv.tv_sec)>(slice.count() / 1000);
      tv.tv_usec = static_cast<decltype(tv.tv_usec)>((slice.count() % 1000) * 1000);
      libusb_handle_events_timeout(_context, &tv);
    }
  }

private:
  static constexpr int RingSize = 8;
  static constexpr int PacketsPerTransfer = 16;

  struct Buf {
    std::vector<unsigned char> data;
    libusb_transfer *transfer{nullptr};
    BulkRing *owner{nullptr};
    bool submitted{false};
  };

  void submit(Buf &buf) {
    const int rc = libusb_submit_transfer(buf.transfer);
    if (rc < 0) {
      buf.submitted = false;
      if (!_fatal_error) {
        _fatal_error = rc;
      }
      return;
    }
    buf.submitted = true;
    _outstanding.fetch_add(1, std::memory_order_relaxed);
  }

  static void LIBUSB_CALL on_complete(libusb_transfer *transfer) {
    auto *buf = static_cast<Buf *>(transfer->user_data);
    buf->owner->handle_complete(*buf);
  }

  void handle_complete(Buf &buf) {
    // submit() also touches buf.submitted / _outstanding without its own
    // locking (it is called unlocked from the constructor, before the ring
    // is visible to any other thread), so it must run under _mutex here too.
    std::lock_guard<std::mutex> lock(_mutex);
    buf.submitted = false;
    _outstanding.fetch_sub(1, std::memory_order_relaxed);
    if (_stopping) {
      return; // shutting down: leave the queue alone, do not resubmit
    }
    if (buf.transfer->status == LIBUSB_TRANSFER_COMPLETED) {
      const auto *bytes =
          reinterpret_cast<const std::byte *>(buf.transfer->buffer);
      _queue.insert(_queue.end(), bytes, bytes + buf.transfer->actual_length);
      submit(buf); // keep the endpoint continuously armed
      return;
    }
    if (buf.transfer->status != LIBUSB_TRANSFER_CANCELLED && !_fatal_error) {
      _fatal_error = transfer_status_error(buf.transfer->status);
    }
  }

  libusb_context *_context;
  std::array<Buf, RingSize> _bufs;
  std::mutex _mutex;
  std::deque<std::byte> _queue;
  std::atomic<int> _outstanding{0};
  bool _stopping{false};
  std::optional<int> _fatal_error;
};

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
  if (_interface) {
    if (const auto bulk = Detail::find_bulk_in_endpoint(device, *_interface)) {
      _bulk_in_ep = bulk->address;
      _bulk_in_max_packet = bulk->max_packet_size;
    }
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
  // Tear the ring down (cancels its transfers) before the handle they were
  // submitted against goes away.
  _bulk_ring.reset();
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

std::size_t LibusbTransport::bulk_in(std::span<std::byte> data,
                                     std::chrono::milliseconds timeout) {
  if (!_bulk_in_ep) {
    throw NotSupported(
        fmt::format("{} has no bulk IN endpoint: streaming is not available",
                    describe_device(libusb_get_device(_handle))));
  }
  if (!_bulk_ring) {
    _bulk_ring = std::make_unique<BulkRing>(_context->native(), _handle,
                                            *_bulk_in_ep, _bulk_in_max_packet);
  }
  return _bulk_ring->read(data, timeout);
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
