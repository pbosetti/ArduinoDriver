// Enumerator.cpp - see Enumerator.h.
#include "arduino_driver/Enumerator.h"

#include "LibusbDetail.h"

#include <fmt/format.h>
#include <libusb.h>

#include <array>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace ArduinoDriver {

// ---- Pure classification ----------------------------------------------------

std::optional<std::uint8_t>
find_vendor_interface(std::span<const InterfaceTriple> interfaces) noexcept {
  for (const InterfaceTriple &itf : interfaces) {
    if (itf.usb_class == USBIO_ITF_CLASS &&
        itf.subclass == USBIO_ITF_SUBCLASS &&
        itf.protocol == USBIO_ITF_PROTOCOL) {
      return itf.number;
    }
  }
  return std::nullopt;
}

bool matches_filter(std::uint16_t vid, std::uint16_t pid,
                    const EnumerateOptions &options) noexcept {
  return (!options.vid || *options.vid == vid) &&
         (!options.pid || *options.pid == pid);
}

bool is_probe_candidate(std::uint16_t vid, std::uint16_t pid,
                        const EnumerateOptions &options) noexcept {
  if (!matches_filter(vid, pid, options)) {
    return false;
  }
  if (options.vid || options.pid) {
    return true; // the caller vouches for the device
  }
  return vid == USBIO_VID_ARDUINO || vid == USBIO_VID_RASPBERRY_PI ||
         vid == USBIO_VID_ESPRESSIF;
}

bool keep_unprobed_candidate(int libusb_code) noexcept {
  return libusb_code == LibusbError::Access ||
         libusb_code == LibusbError::NotSupported ||
         libusb_code == LibusbError::Busy;
}

// ---- Enumeration ------------------------------------------------------------

namespace {

/// Frees a libusb device list (and its references) on scope exit.
struct DeviceList {
  libusb_device **list{nullptr};
  ~DeviceList() {
    if (list != nullptr) {
      libusb_free_device_list(list, 1);
    }
  }
};

DeviceRef make_ref(libusb_device *device) {
  return DeviceRef(libusb_ref_device(device),
                   [](libusb_device *d) { libusb_unref_device(d); });
}

std::string read_string(libusb_device_handle *handle, std::uint8_t index) {
  if (index == 0) {
    return {};
  }
  std::array<unsigned char, 256> buffer{};
  const int n = libusb_get_string_descriptor_ascii(
      handle, index, buffer.data(), static_cast<int>(buffer.size()));
  if (n <= 0) {
    return {};
  }
  return std::string(reinterpret_cast<const char *>(buffer.data()),
                     static_cast<std::size_t>(n));
}

/// Opens the device, reads its strings and GET_INFO into `info`. Throws
/// UsbError / ProtocolError when the device is not (or not yet) reachable.
void probe(const std::shared_ptr<Context> &context, libusb_device *device,
           const libusb_device_descriptor &desc, DeviceInfo &info,
           const EnumerateOptions &options) {
  LibusbTransport transport(context, device, LibusbTransportOptions{});
  libusb_device_handle *handle = transport.native_handle();
  info.manufacturer = read_string(handle, desc.iManufacturer);
  info.product = read_string(handle, desc.iProduct);
  info.serial = read_string(handle, desc.iSerialNumber);

  std::array<std::byte, InfoLen> reply{};
  const std::size_t n = transport.control_in(USBIO_REQ_GET_INFO, 0, 0, reply,
                                             options.probe_timeout);
  if (n < InfoLen) {
    throw ProtocolError(
        fmt::format("GET_INFO: short reply ({} of {} bytes)", n, InfoLen));
  }
  info.info = decode_info(reply); // throws ProtocolError on a bad magic
}

DeviceRef find_device(Context &context, const DeviceInfo &info) {
  DeviceList devices;
  const auto count = libusb_get_device_list(context.native(), &devices.list);
  if (count < 0) {
    Detail::throw_usb_error(static_cast<int>(count),
                            "cannot enumerate USB devices");
  }
  for (std::decay_t<decltype(count)> i = 0; i < count; ++i) {
    libusb_device *device = devices.list[i];
    if (libusb_get_bus_number(device) != info.bus ||
        libusb_get_device_address(device) != info.address) {
      continue;
    }
    libusb_device_descriptor desc{};
    if (libusb_get_device_descriptor(device, &desc) < 0 ||
        desc.idVendor != info.vid || desc.idProduct != info.pid) {
      continue;
    }
    return make_ref(device);
  }
  throw DeviceNotFound(
      fmt::format("USB device {:04X}:{:04X} at bus {} address {} is gone",
                  info.vid, info.pid, info.bus, info.address));
}

} // namespace

std::vector<DeviceInfo> list_devices(std::shared_ptr<Context> context,
                                     EnumerateOptions options) {
  if (!context) {
    throw std::invalid_argument("list_devices: context must not be null");
  }
  DeviceList devices;
  const auto count = libusb_get_device_list(context->native(), &devices.list);
  if (count < 0) {
    Detail::throw_usb_error(static_cast<int>(count),
                            "cannot enumerate USB devices");
  }

  std::vector<DeviceInfo> found;
  for (std::decay_t<decltype(count)> i = 0; i < count; ++i) {
    libusb_device *device = devices.list[i];
    libusb_device_descriptor desc{};
    if (libusb_get_device_descriptor(device, &desc) < 0) {
      continue;
    }
    if (!matches_filter(desc.idVendor, desc.idProduct, options)) {
      continue;
    }

    DeviceInfo info;
    info.vid = desc.idVendor;
    info.pid = desc.idProduct;
    info.bus = libusb_get_bus_number(device);
    info.address = libusb_get_device_address(device);
    info.interface_number =
        find_vendor_interface(Detail::read_interfaces(device));
    info.has_vendor_interface = info.interface_number.has_value();

    // Without the interface, only a successful GET_INFO proves identity.
    info.identified = info.has_vendor_interface;
    if (!info.identified &&
        !(options.probe &&
          is_probe_candidate(desc.idVendor, desc.idProduct, options))) {
      continue;
    }
    if (options.probe) {
      try {
        probe(context, device, desc, info, options);
        info.identified = true;
      } catch (const Error &e) {
        // A candidate that could not even be opened may still be one of
        // ours: keep it visible, unidentified. A STALL, a timeout or a bad
        // reply means the device answered and is not one of ours.
        const auto *usb = dynamic_cast<const UsbError *>(&e);
        const bool keep =
            info.has_vendor_interface ||
            (usb != nullptr && keep_unprobed_candidate(usb->code()));
        if (!keep) {
          continue;
        }
        info.probe_error = e.what();
      }
    }
    info.context = context;
    info.device = make_ref(device);
    found.push_back(std::move(info));
  }
  return found;
}

std::unique_ptr<LibusbTransport>
open_transport(std::shared_ptr<Context> context, const DeviceInfo &info,
               LibusbTransportOptions options) {
  if (!context) {
    throw std::invalid_argument("open_transport: context must not be null");
  }
  const DeviceRef device = (info.device && info.context == context)
                               ? info.device
                               : find_device(*context, info);
  return std::make_unique<LibusbTransport>(std::move(context), device.get(),
                                           options);
}

Device open_device(const DeviceInfo &info, DeviceOptions device_options,
                   LibusbTransportOptions transport_options) {
  if (!info.context) {
    throw std::invalid_argument(
        "open_device: the DeviceInfo has no context (it did not come from "
        "list_devices); use open_transport with an explicit context");
  }
  return Device(open_transport(info.context, info, transport_options),
                device_options);
}

Device open_first(std::shared_ptr<Context> context,
                  EnumerateOptions enumerate_options,
                  DeviceOptions device_options,
                  LibusbTransportOptions transport_options) {
  const std::vector<DeviceInfo> devices =
      list_devices(std::move(context), enumerate_options);
  for (const DeviceInfo &info : devices) {
    if (info.identified) {
      return open_device(info, device_options, transport_options);
    }
  }
  std::string message = "no UsbIo device found";
  if (enumerate_options.vid || enumerate_options.pid) {
    message += " matching";
    if (enumerate_options.vid) {
      message += fmt::format(" vid=0x{:04X}", *enumerate_options.vid);
    }
    if (enumerate_options.pid) {
      message += fmt::format(" pid=0x{:04X}", *enumerate_options.pid);
    }
  }
  for (const DeviceInfo &info : devices) {
    message += fmt::format("; {:04X}:{:04X} at bus {} address {} could not "
                           "be probed ({})",
                           info.vid, info.pid, info.bus, info.address,
                           info.probe_error);
  }
  throw DeviceNotFound(message);
}

Device open_by_serial(std::shared_ptr<Context> context, std::string_view serial,
                      EnumerateOptions enumerate_options,
                      DeviceOptions device_options,
                      LibusbTransportOptions transport_options) {
  const std::vector<DeviceInfo> devices =
      list_devices(std::move(context), enumerate_options);
  std::size_t identified = 0;
  for (const DeviceInfo &info : devices) {
    if (!info.identified) {
      continue;
    }
    ++identified;
    if (info.serial == serial) {
      return open_device(info, device_options, transport_options);
    }
  }
  throw DeviceNotFound(fmt::format(
      "no UsbIo device with serial number \"{}\" ({} identified, {} not "
      "probed)",
      serial, identified, devices.size() - identified));
}

} // namespace ArduinoDriver
