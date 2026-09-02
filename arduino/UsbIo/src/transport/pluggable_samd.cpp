/*
 * pluggable_samd.cpp - UsbIo transport for the SAMD21 core (Zero, MKR family,
 * Nano 33 IoT).
 *
 * USBCore.cpp:373-381 hands every non-standard SETUP packet (class or vendor,
 * any recipient) to PluggableUSB().setup(), which offers it to each plugged
 * module in turn (api/PluggableUSB.cpp:59-68); when no module claims it the
 * core STALLs EP0 (USBCore.cpp:896-903). Our module adds one endpoint-less
 * vendor interface (api/PluggableUSB.cpp:70-73 accepts numEndpoints == 0) so
 * the host can identify the device from the configuration descriptor.
 *
 * main.cpp:45-46 runs USBDevice.init()/attach() before setup(), and
 * descriptors are assembled lazily on GET_DESCRIPTOR, so plugging from a
 * static constructor - as the core's own Serial_ does (CDC.cpp:170-175) - is
 * all that is needed. setup() is called from the USB_Handler IRQ: ISR context.
 */
#if defined(ARDUINO_ARCH_SAMD)

#include <Arduino.h>

#include "api/PluggableUSB.h"

#include "../UsbIo.h"
#include "UsbIoTransport.h"

namespace {

class UsbIoSamdModule : public arduino::PluggableUSBModule {
public:
  UsbIoSamdModule() : arduino::PluggableUSBModule(0, 1, NULL) {
    PluggableUSB().plug(this);
  }

protected:
  /* Called twice per configuration descriptor request: a dry run to size it
   * and a packing pass (USBCore.cpp:688-700); sendControl() handles both. */
  int getInterface(uint8_t *interfaceCount) override {
    *interfaceCount += 1;
    const InterfaceDescriptor itf =
        D_INTERFACE(pluggedInterface, 0, USBIO_ITF_CLASS, USBIO_ITF_SUBCLASS,
                    USBIO_ITF_PROTOCOL);
    return USBDevice.sendControl(&itf, sizeof(itf));
  }

  int getDescriptor(USBSetup &) override {
    return 0; /* no class-specific descriptors */
  }

  bool setup(USBSetup &s) override {
    if ((s.bmRequestType & REQUEST_TYPE) != REQUEST_VENDOR) {
      return false; /* not ours; the core tries the next module (CDC) */
    }
    /* An interface-recipient packet for another interface is not ours; the
     * core applies the recipient policy to everything else (raw packet). */
    if ((s.bmRequestType & REQUEST_RECIPIENT) == REQUEST_INTERFACE &&
        (s.wIndex & 0xFFu) != pluggedInterface) {
      return false;
    }
    const bool in = (s.bmRequestType & REQUEST_DIRECTION) == REQUEST_DEVICETOHOST;
    const uint16_t value = (uint16_t)(s.wValueL | (s.wValueH << 8));
    const uint8_t *reply = NULL;
    uint16_t len = 0;
    if (!UsbIo.handle_setup(s.bmRequestType, s.bRequest, value, s.wIndex,
                            s.wLength, &reply, &len)) {
      return false; /* USBCore.cpp:903 stall(0) */
    }
    if (in && len > 0) {
      /* armSend() copies the whole reply into EP0's 64-byte row of
       * udd_ep_in_cache_buffer (USBCore.cpp:100, 676-686); a longer reply
       * would spill into EP1's row. No reply exceeds 33 bytes on today's SAMD
       * boards; the clamp turns a future overflow into a short reply that the
       * host detects instead of silent corruption. */
      if (len > Ep0CacheSize) {
        len = Ep0CacheSize;
      }
      USBDevice.sendControl(reply, len);
    } else {
      /* No data to send (OUT acknowledge, or an empty IN reply): zero the IN
       * byte count so the stage the ISR arms next is a ZLP - the same thing
       * the core does for SET_CONFIGURATION / SET_INTERFACE
       * (USBCore.cpp:806,818). */
      USBDevice.sendZlp(0);
    }
    return true;
  }

private:
  static const uint16_t Ep0CacheSize = 64; /* USBCore.cpp:100 */
};

/* Static instance: constructed (and plugged) during static initialisation. */
UsbIoSamdModule module;

} // namespace

uint16_t usbio_transport_begin() {
  return USBIO_FLAG_VENDOR_INTERFACE;
}

#endif /* ARDUINO_ARCH_SAMD */
