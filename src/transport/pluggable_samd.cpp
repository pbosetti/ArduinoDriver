/*
 * pluggable_samd.cpp - UsbIo transport for the SAMD21 core (Zero, MKR family,
 * Nano 33 IoT).
 *
 * USBCore.cpp:373-381 hands every non-standard SETUP packet (class or vendor,
 * any recipient) to PluggableUSB().setup(), which offers it to each plugged
 * module in turn (api/PluggableUSB.cpp:59-68); when no module claims it the
 * core STALLs EP0 (USBCore.cpp:896-903). Our module adds one vendor interface
 * so the host can identify the device from the configuration descriptor.
 *
 * main.cpp:45-46 runs USBDevice.init()/attach() before setup(), and
 * descriptors are assembled lazily on GET_DESCRIPTOR, so plugging from a
 * static constructor - as the core's own Serial_ does (CDC.cpp:170-175) - is
 * all that is needed. setup() is called from the USB_Handler IRQ: ISR context.
 *
 * Streaming (usbio_protocol.h, "Streaming"): the interface also claims one
 * bulk IN endpoint, a lower tier than mbed's: one 64-byte record per poll()
 * call, ~64 kB/s at a fast loop(). PluggableUSBModule's ctor takes
 * numEndpoints + an epType table (api/PluggableUSB.h:29-33); plug()
 * (api/PluggableUSB.cpp:70-95) reserves the endpoint eagerly at that point -
 * unlike mbed, there is no separate resolver/configuration-callback step.
 * Sends use the core's own USBDeviceClass::send(), the same call CDC.cpp uses
 * for its IN endpoint: it is a BLOCKING call (USBCore.cpp:607-674, up to
 * TX_TIMEOUT_MS = 70 ms if the host stops draining the endpoint) rather than
 * the fire-and-forget write mbed gets from write_start()/write_finish(). The
 * core exposes no non-blocking "is this endpoint free" query outside that
 * function, and reaching into its internal bank/cache-buffer state (as
 * send() itself does) to build one would mean depending on undocumented
 * internals of a core we do not own.
 *
 * A host that stops reading the endpoint therefore costs loop() one 70 ms
 * timeout per record. To keep that bounded, a short write is reported as
 * USBIO_STREAM_WRITE_FAILED and the core stops the stream after
 * StreamTxFailureLimit consecutive ones (UsbIo.h): an abandoned stream costs
 * ~210 ms once, not 70 ms on every poll() call for ever. mbed needs none of
 * this - its write is genuinely fire-and-forget and reports BUSY instead.
 */
#if defined(ARDUINO_ARCH_SAMD)

#include <Arduino.h>

#include "api/PluggableUSB.h"

#include "../UsbIo.h"
#include "UsbIoTransport.h"

namespace {

class UsbIoSamdModule : public arduino::PluggableUSBModule {
public:
  UsbIoSamdModule() : arduino::PluggableUSBModule(1, 1, _ep_type) {
    _ep_type[0] = USB_ENDPOINT_TYPE_BULK | USB_ENDPOINT_IN(0);
    PluggableUSB().plug(this);
  }

  /* Write of one stream record; see UsbIoTransport.h. Called from poll() only
   * - see the class comment above for why this particular call can block.
   * send() has no "busy, try later" state: it either gets the packet out or
   * has already burned its 70 ms timeout waiting for a host that is not
   * draining, so a short write is reported as FAILED (a strike towards the
   * core's auto-stop) rather than BUSY, which would retry forever. */
  uint8_t stream_write(const uint8_t *data, uint16_t len) {
    if (len > USBIO_STREAM_EP_SIZE) {
      return USBIO_STREAM_WRITE_FAILED;
    }
    const int sent = USBDevice.send(USB_ENDPOINT_IN(pluggedEndpoint), data, len);
    return sent == (int)len ? USBIO_STREAM_WRITE_SENT : USBIO_STREAM_WRITE_FAILED;
  }

protected:
  /* Called twice per configuration descriptor request: a dry run to size it
   * and a packing pass (USBCore.cpp:688-700); sendControl() handles both. */
  int getInterface(uint8_t *interfaceCount) override {
    *interfaceCount += 1;
    const InterfaceDescriptor itf =
        D_INTERFACE(pluggedInterface, 1, USBIO_ITF_CLASS, USBIO_ITF_SUBCLASS,
                    USBIO_ITF_PROTOCOL);
    const EndpointDescriptor ep = D_ENDPOINT(USB_ENDPOINT_IN(pluggedEndpoint),
                                             USB_ENDPOINT_TYPE_BULK,
                                             USBIO_STREAM_EP_SIZE, 0);
    const int itf_sent = USBDevice.sendControl(&itf, sizeof(itf));
    if (itf_sent < 0) {
      return itf_sent;
    }
    const int ep_sent = USBDevice.sendControl(&ep, sizeof(ep));
    if (ep_sent < 0) {
      return ep_sent;
    }
    return itf_sent + ep_sent;
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

  /* Non-const because PluggableUSBModule's ctor wants unsigned int*
   * (api/PluggableUSB.h:31); filled in before plug() below ever reads it,
   * exactly like CDC.cpp:170-176 fills its own epType[]. */
  unsigned int _ep_type[1];
};

/* Static instance: constructed (and plugged) during static initialisation. */
UsbIoSamdModule module;

} // namespace

uint16_t usbio_transport_begin() {
  return USBIO_FLAG_VENDOR_INTERFACE | USBIO_FLAG_STREAMING;
}

uint8_t usbio_transport_stream_write(const uint8_t *data, uint16_t len) {
  return module.stream_write(data, len);
}

#endif /* ARDUINO_ARCH_SAMD */
