/*
 * tinyusb_rp2040.cpp - UsbIo transport for arduino-pico (Earle Philhower's
 * RP2040 / RP2350 core), TinyUSB device stack.
 *
 * UNVERIFIED: that core cannot be installed in the development environment,
 * so this file has never been compiled or run. It mirrors
 * tinyusb_renesas.cpp: TinyUSB routes every vendor-type SETUP packet to the
 * weak tud_vendor_control_xfer_cb() (device/usbd.c, outside any
 * CFG_TUD_VENDOR guard) and the definition below is expected to be the only
 * one. No vendor interface is added, so the host identifies the board by
 * VID 0x2E8A + GET_INFO probe (flag 0).
 *
 * To verify once the core is available:
 *   - the core must not define tud_vendor_control_xfer_cb itself: grep
 *     cores/rp2040/RP2040USB.cpp (and libraries/Adafruit_TinyUSB_Arduino when
 *     the "Adafruit TinyUSB" USB-stack menu entry is selected);
 *   - the "tusb.h" include path (pico-sdk's TinyUSB with the default stack,
 *     the Adafruit library's copy otherwise) and that tusb_config.h reaches
 *     us through the same flags the core's own USB sources use;
 *   - CFG_TUD_ENDPOINT0_SIZE (64) and the tud_control_xfer() multi-packet /
 *     ZLP behaviour in device/usbd_control.c, as checked for the Renesas core;
 *   - whether RP2040USB.cpp offers a hook to add an interface descriptor
 *     (its __USBInstallSerial-style installers): if so, a FF/49/4F vendor
 *     interface can be added as a follow-up and the flag reported;
 *   - the Nano RP2040 Connect on the *mbed* core does not use this file: it
 *     defines ARDUINO_ARCH_MBED and is served by pluggable_mbed.cpp.
 *
 * Binding: the callback is declared TU_ATTR_WEAK by usbd.h, so this
 * definition is emitted as a weak symbol (W); it still resolves because the
 * only other occurrence is the weak undefined reference in usbd.c.
 */
#if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)

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
  /* Raw packet straight through: the core applies the recipient policy and
   * STALLs the interface-recipient form (UNSUPPORTED) since this transport
   * reports no vendor interface. */
  const bool in = request->bmRequestType_bit.direction == TUSB_DIR_IN;
  const uint8_t *reply = NULL;
  uint16_t len = 0;
  if (!UsbIo.handle_setup(request->bmRequestType, request->bRequest,
                          request->wValue, request->wIndex, request->wLength,
                          &reply, &len)) {
    return false; /* usbd.c stalls EP0 */
  }
  if (in) {
    return tud_control_xfer(rhport, request, const_cast<uint8_t *>(reply), len);
  }
  return tud_control_status(rhport, request); /* ACK: status stage only */
}

uint16_t usbio_transport_begin() {
  return 0; /* no vendor interface on this stack (yet) */
}

#endif /* ARDUINO_ARCH_RP2040 && !ARDUINO_ARCH_MBED */
