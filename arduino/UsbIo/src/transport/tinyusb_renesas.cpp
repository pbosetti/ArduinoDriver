/*
 * tinyusb_renesas.cpp - UsbIo transport for the Renesas RA4M1 core (UNO R4
 * Minima, Nano R4), TinyUSB device stack.
 *
 * TinyUSB dispatches every vendor-type SETUP packet to the weak
 * tud_vendor_control_xfer_cb() (device/usbd.h:152; usbd.c:624-629, outside
 * any CFG_TUD_VENDOR guard) and the core never defines it, so the definition
 * below takes over without touching the core. The descriptors are fixed by
 * the core (usb/USB.cpp): no vendor interface can be added, so the transport
 * reports no USBIO_FLAG_VENDOR_INTERFACE and the core STALLs the
 * interface-recipient request form with UNSUPPORTED.
 *
 * Binding note: usbd.h declares the callback TU_ATTR_WEAK and GCC carries a
 * weak attribute from a prior declaration onto the definition, so nm shows
 * this symbol as W (weak defined) rather than T. The link is nevertheless
 * deterministic: the only other occurrence is the weak *undefined* reference
 * in core.a(usbd.c.o), which a weak definition satisfies (verified on the
 * built ELF: resolved to this function's address).
 *
 * tud_task() runs inside the USB interrupt (usb/USB.cpp:299-303), so this
 * callback is ISR context; it only forwards to UsbIo.handle_setup().
 */
#if defined(ARDUINO_ARCH_RENESAS)

#if defined(NO_USB)
#error "UsbIo: this board has no native USB device (the UNO R4 WiFi is not supported)"
#endif

#include <Arduino.h>

#include "tusb.h"

#include "../UsbIo.h"
#include "UsbIoTransport.h"

extern "C" bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                           tusb_control_request_t const *request) {
  if (stage != CONTROL_STAGE_SETUP) {
    return true; /* DATA / ACK stages: nothing to do, let the transfer complete */
  }
  if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR) {
    return false; /* cannot happen: usbd.c only routes vendor requests here */
  }
  /* Raw packet straight through; the core applies the recipient policy. */
  const bool in = request->bmRequestType_bit.direction == TUSB_DIR_IN;
  const uint8_t *reply = NULL;
  uint16_t len = 0;
  if (!UsbIo.handle_setup(request->bmRequestType, request->bRequest,
                          request->wValue, request->wIndex, request->wLength,
                          &reply, &len)) {
    return false; /* usbd.c stalls EP0 */
  }
  if (in) {
    /* usbd_control.c:106-131 clamps to wLength, splits the reply into
     * CFG_TUD_ENDPOINT0_SIZE (64) byte transactions and sends a ZLP when
     * len == 0. The reply buffer is a static member of UsbIo, so it outlives
     * the data stage. */
    return tud_control_xfer(rhport, request, const_cast<uint8_t *>(reply), len);
  }
  return tud_control_status(rhport, request); /* ACK: status stage only */
}

uint16_t usbio_transport_begin() {
  return 0; /* no vendor interface on this stack */
}

#endif /* ARDUINO_ARCH_RENESAS */
