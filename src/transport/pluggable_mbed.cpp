/*
 * pluggable_mbed.cpp - UsbIo transport for the Arduino mbed cores (Portenta
 * H7, GIGA R1, Nano 33 BLE, Nano RP2040 Connect).
 *
 * A PluggableUSBModule adds one vendor interface to the composite device the
 * core builds (CDC + ours) and receives every EP0 SETUP packet: mbed-os
 * USBDevice::_control_setup() calls callback_request() for all request types
 * before any standard processing, and PluggableUSBDevice.cpp:142 offers the
 * packet to each plugged module until one answers something other than
 * PassThrough.
 *
 * The module must be plugged before main.cpp:40 `PluggableUSBD().begin()`
 * runs, i.e. from a static constructor, exactly like the core's own
 * `_SerialUSB` (USB/USBSerial.cpp:129). All callbacks run in the USB
 * interrupt; they only shuffle bytes between the packet and UsbIo.
 *
 * Streaming (usbio_protocol.h, "Streaming"): the interface also claims one
 * bulk IN endpoint, the same way USBCDC.cpp claims its three - init() calls
 * EndpointResolver::endpoint_in(), callback_set_configuration() wires the
 * endpoint up with PluggableUSBD().endpoint_add(), and sends go through
 * write_start()/write_finish(). stream_write() below is the non-blocking send
 * poll() uses; the completion callback only clears the in-flight flag, never
 * touches UsbIo state (still interrupt context, same rule as
 * callback_request()). This transport always has the endpoint - mbed is
 * USBIO_HAS_STREAM_TRANSPORT unconditionally (UsbIo.h) - so nothing here is
 * conditional on it.
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
  UsbIoMbedModule()
      : arduino::internal::PluggableUSBModule(1), _tx_in_progress(false) {
    PluggableUSBD().plug(this);
  }

  /* Non-blocking write of one stream record; see UsbIoTransport.h. Called
   * from poll() only.
   *
   * The flag is raised BEFORE the transfer is armed: a 44-byte packet can
   * complete while this function is still running, and _stream_send_done()
   * (USB interrupt, higher priority than loop()) only ever lowers it. Arming
   * first would let that completion land between write_start() and the
   * assignment, leaving the flag raised with no transfer in flight and the
   * stream stalled for good. USBCDC.cpp:377-404 avoids the same race with
   * lock()/assert_locked(), which a PluggableUSBModule cannot take. */
  uint8_t stream_write(const uint8_t *data, uint16_t len) {
    if (_tx_in_progress || len > sizeof(_tx_buffer)) {
      return USBIO_STREAM_WRITE_BUSY;
    }
    memcpy(_tx_buffer, data, len);
    _tx_in_progress = true;
    if (!write_start(_bulk_in, _tx_buffer, len)) {
      _tx_in_progress = false;
      return USBIO_STREAM_WRITE_BUSY;
    }
    return USBIO_STREAM_WRITE_SENT;
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
        0x01,                        // bNumEndpoints
        USBIO_ITF_CLASS,             // bInterfaceClass
        USBIO_ITF_SUBCLASS,          // bInterfaceSubClass
        USBIO_ITF_PROTOCOL,          // bInterfaceProtocol
        STRING_OFFSET_IINTERFACE,    // iInterface

        ENDPOINT_DESCRIPTOR_LENGTH, // bLength
        ENDPOINT_DESCRIPTOR,        // bDescriptorType
        _bulk_in,                   // bEndpointAddress
        E_BULK,                     // bmAttributes
        LSB(USBIO_STREAM_EP_SIZE),  // wMaxPacketSize (LSB)
        MSB(USBIO_STREAM_EP_SIZE),  // wMaxPacketSize (MSB)
        0x00,                       // bInterval
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
   * complete_set_configuration(); mirror USBCDC.cpp:286 which returns true.
   * USBCDC.cpp:293-295: endpoints > 0 are only usable once the host has set a
   * configuration, so (re-)wire ours here too, exactly like CDC does for its
   * own bulk/interrupt endpoints. */
  bool callback_set_configuration(uint8_t) override {
    PluggableUSBD().endpoint_add(
        _bulk_in, USBIO_STREAM_EP_SIZE, USB_EP_TYPE_BULK,
        ::mbed::callback(this, &UsbIoMbedModule::_stream_send_done));
    _tx_in_progress = false;
    return true;
  }
  void callback_set_interface(uint16_t, uint8_t) override {}
  /* usbio_protocol.h, "Streaming": suspend/disconnect stop a running stream
   * (RESET's semantics; the selection does not survive re-enumeration
   * either). Leaving Configured also means the endpoint above is about to be
   * torn down, so drop any in-flight write with it. */
  void callback_state_change(USBDevice::DeviceState new_state) override {
    if (new_state != USBDevice::Configured) {
      _tx_in_progress = false;
      UsbIo.handle_usb_disconnected();
    }
  }
  void init(EndpointResolver &resolver) override {
    _bulk_in = resolver.endpoint_in(USB_EP_TYPE_BULK, USBIO_STREAM_EP_SIZE);
    MBED_ASSERT(resolver.valid());
  }

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

  /* Endpoint completion callback, INTERRUPT CONTEXT (USBCDC.cpp's
   * _send_isr() is the model): only releases the hardware transfer and
   * clears the in-flight flag so the next poll() can send. Never touches
   * UsbIo state. */
  void _stream_send_done() {
    write_finish(_bulk_in);
    _tx_in_progress = false;
  }

  static const uint8_t DescriptorLength = CONFIGURATION_DESCRIPTOR_LENGTH +
                                          INTERFACE_DESCRIPTOR_LENGTH +
                                          ENDPOINT_DESCRIPTOR_LENGTH;
  static const uint8_t DefaultConfiguration = 1;
  /* Summed into bcdDevice by plug(); CDC uses 1, HID 4, MSD 8. */
  static const uint8_t ProductVersion = 0x20;

  uint8_t _configuration_descriptor[DescriptorLength];
  usb_ep_t _bulk_in;
  uint8_t _tx_buffer[USBIO_STREAM_EP_SIZE];
  /* Raised by stream_write() in loop() context, lowered by the completion
   * callback and callback_state_change() in interrupt context; volatile
   * because, unlike USBCDC's, these accesses are not serialised by lock(). */
  volatile bool _tx_in_progress;
};

/* Static instance: constructed (and plugged) during static initialisation. */
UsbIoMbedModule module;

} // namespace

uint16_t usbio_transport_begin() {
  return USBIO_FLAG_VENDOR_INTERFACE | USBIO_FLAG_STREAMING;
}

uint8_t usbio_transport_stream_write(const uint8_t *data, uint16_t len) {
  /* Never FAILED: write_start() refusing only ever means "still in flight",
   * and a host that stops draining simply leaves the completion callback
   * pending - it cannot block loop() the way SAMD's blocking send can. */
  return module.stream_write(data, len);
}

#endif /* ARDUINO_ARCH_MBED */
