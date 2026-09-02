/*
 * pluggable_mbed.cpp - UsbIo transport for the Arduino mbed cores (Portenta
 * H7, GIGA R1, Nano 33 BLE, Nano RP2040 Connect).
 *
 * A PluggableUSBModule adds one endpoint-less vendor interface to the
 * composite device the core builds (CDC + ours) and receives every EP0 SETUP
 * packet: mbed-os USBDevice::_control_setup() calls callback_request() for
 * all request types before any standard processing, and
 * PluggableUSBDevice.cpp:142 offers the packet to each plugged module until
 * one answers something other than PassThrough.
 *
 * The module must be plugged before main.cpp:40 `PluggableUSBD().begin()`
 * runs, i.e. from a static constructor, exactly like the core's own
 * `_SerialUSB` (USB/USBSerial.cpp:129). All callbacks run in the USB
 * interrupt; they only shuffle bytes between the packet and UsbIo.
 */
#if defined(ARDUINO_ARCH_MBED)

#include <Arduino.h>
#include <string.h>

#include "USB/PluggableUSBDevice.h"

#include "../UsbIo.h"
#include "UsbIoTransport.h"

namespace {

class UsbIoMbedModule : public arduino::internal::PluggableUSBModule {
public:
  UsbIoMbedModule() : arduino::internal::PluggableUSBModule(1) {
    PluggableUSBD().plug(this);
  }

protected:
  /* The core keeps only bytes 9.. of what we return (PluggableUSBDevice.cpp:
   * 295), so the configuration header merely needs a correct wTotalLength. */
  const uint8_t *configuration_desc(uint8_t index) override {
    if (index != 0) {
      return NULL;
    }
    const uint8_t desc[DescriptorLength] = {
        CONFIGURATION_DESCRIPTOR_LENGTH, // bLength
        CONFIGURATION_DESCRIPTOR,        // bDescriptorType
        LSB(DescriptorLength),          // wTotalLength (LSB)
        MSB(DescriptorLength),          // wTotalLength (MSB)
        0x01,                            // bNumInterfaces
        DefaultConfiguration,           // bConfigurationValue
        0x00,                            // iConfiguration
        C_RESERVED | C_SELF_POWERED,     // bmAttributes
        C_POWER(0),                      // bMaxPower

        INTERFACE_DESCRIPTOR_LENGTH, // bLength
        INTERFACE_DESCRIPTOR,        // bDescriptorType
        pluggedInterface,            // bInterfaceNumber
        0x00,                        // bAlternateSetting
        0x00,                        // bNumEndpoints
        USBIO_ITF_CLASS,             // bInterfaceClass
        USBIO_ITF_SUBCLASS,          // bInterfaceSubClass
        USBIO_ITF_PROTOCOL,          // bInterfaceProtocol
        STRING_OFFSET_IINTERFACE,    // iInterface
    };
    memcpy(_configuration_descriptor, desc, sizeof(desc));
    return _configuration_descriptor;
  }

  /* PluggableUSBDevice.cpp:237 feeds this through TO_UNICODE(), so it expects
   * a plain C string, not a string descriptor. */
  const uint8_t *string_iinterface_desc() override {
    return reinterpret_cast<const uint8_t *>(USBIO_ITF_STRING);
  }

  uint8_t getProductVersion() override { return ProductVersion; }

  uint32_t callback_request(const USBDevice::setup_packet_t *setup,
                            USBDevice::RequestResult *result,
                            uint8_t **data) override {
    *result = USBDevice::PassThrough;
    if (!is_ours(setup)) {
      return 0;
    }
    const bool in = setup->bmRequestType.dataTransferDirection == DEVICE_TO_HOST;
    /* mbed pre-splits bmRequestType; rebuild the raw byte, the core applies
     * the recipient policy (device: pin = wIndex, interface: pin = wIndex >> 8). */
    const uint8_t bm = (uint8_t)((in ? 0x80u : 0u) |
                                 (uint8_t)(setup->bmRequestType.Type << 5) |
                                 (setup->bmRequestType.Recipient & 0x1Fu));
    const uint8_t *reply = NULL;
    uint16_t len = 0;
    if (!UsbIo.handle_setup(bm, setup->bRequest, setup->wValue, setup->wIndex,
                            setup->wLength, &reply, &len)) {
      *result = USBDevice::Failure; /* STALL */
      return 0;
    }
    if (in) {
      /* mbed clamps to wLength and emits the ZLP itself when len == 0. */
      *result = USBDevice::Send;
      *data = const_cast<uint8_t *>(reply);
      return len;
    }
    *result = USBDevice::Success; /* ACK, no data stage */
    return 0;
  }

  /* PluggableUSBDevice.cpp:167 stalls the status stage unless one module
   * claims the transfer, so claim ours. */
  bool callback_request_xfer_done(const USBDevice::setup_packet_t *setup,
                                  bool) override {
    return is_ours(setup);
  }

  /* PluggableUSBDevice.cpp:177 forwards the LAST module's answer to
   * complete_set_configuration(); mirror USBCDC.cpp:286 which returns true. */
  bool callback_set_configuration(uint8_t) override { return true; }
  void callback_set_interface(uint16_t, uint8_t) override {}
  void callback_state_change(USBDevice::DeviceState) override {}
  void init(EndpointResolver &) override {} /* no endpoints to claim */

private:
  /* Every vendor-type packet is ours, except an interface-recipient one
   * addressed to another interface (low byte of wIndex per the USB spec). */
  bool is_ours(const USBDevice::setup_packet_t *setup) const {
    if (setup->bmRequestType.Type != VENDOR_TYPE) {
      return false;
    }
    return setup->bmRequestType.Recipient != INTERFACE_RECIPIENT ||
           (setup->wIndex & 0xFFu) == pluggedInterface;
  }

  static const uint8_t DescriptorLength =
      CONFIGURATION_DESCRIPTOR_LENGTH + INTERFACE_DESCRIPTOR_LENGTH;
  static const uint8_t DefaultConfiguration = 1;
  /* Summed into bcdDevice by plug(); CDC uses 1, HID 4, MSD 8. */
  static const uint8_t ProductVersion = 0x20;

  uint8_t _configuration_descriptor[DescriptorLength];
};

/* Static instance: constructed (and plugged) during static initialisation. */
UsbIoMbedModule module;

} // namespace

uint16_t usbio_transport_begin() {
  return USBIO_FLAG_VENDOR_INTERFACE;
}

#endif /* ARDUINO_ARCH_MBED */
