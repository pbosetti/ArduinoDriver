/*
 * esp32_vendor.cpp - UsbIo transport for arduino-esp32 3.x native USB
 * (ESP32-S2 / ESP32-S3 with "USB Mode: USB-OTG (TinyUSB)").
 *
 * UNVERIFIED: that core cannot be installed in the development environment;
 * this file is written from memory of the core's USBVendor API and has never
 * been compiled or run. Best-effort scaffolding.
 *
 * In OTG mode the core owns tud_vendor_control_xfer_cb() and forwards vendor
 * control requests to the handler registered with USBVendor::onRequest().
 * USBVendor adds TinyUSB's generic vendor interface (class FF, subclass 00,
 * protocol 00, two bulk endpoints) - not our FF/49/4F triple - so this
 * transport reports flag 0 and the host identifies the board by VID 0x303A +
 * GET_INFO probe. The "Hardware CDC and JTAG" mode (ARDUINO_USB_MODE == 1)
 * has no vendor request path at all, hence the #error.
 *
 * To verify once the core is available (libraries/USB/src/USBVendor.h/.cpp):
 *   - the names and layout of arduino_usb_control_request_t (bmRequestType,
 *     bRequest, wValue, wIndex, wLength) and of the handler type
 *     arduino_usb_vendor_control_request_handler_t
 *     (bool (*)(uint8_t rhport, uint8_t stage, arduino_usb_control_request_t const *));
 *   - the stage constants (REQUEST_STAGE_SETUP / DATA / ACK) the core passes;
 *   - USBVendor::sendResponse(rhport, request, data, len): that it wraps
 *     tud_control_xfer() (data) / tud_control_status() (len == 0) and copes
 *     with multi-packet replies and ZLPs;
 *   - the ordering constraint: USBVendor::begin() must run before USB.begin()
 *     for the interface to be enumerated. With ARDUINO_USB_CDC_ON_BOOT the
 *     core may already have called USB.begin() before setup(); if so the
 *     vendor interface must be registered from a static initializer instead;
 *   - that the handler runs in the TinyUSB task/ISR context the core uses
 *     (UsbIo.handle_setup() is safe in either).
 */
#if defined(ARDUINO_ARCH_ESP32)

#if !defined(ARDUINO_USB_MODE) || ARDUINO_USB_MODE != 0
#error "UsbIo: select USB Mode: USB-OTG (TinyUSB) in the board menu"
#endif

#include <Arduino.h>

#include "USB.h"
#include "USBVendor.h"

#include "../UsbIo.h"
#include "UsbIoTransport.h"

namespace {

USBVendor vendor; /* TinyUSB generic vendor class FF/00/00 + two bulk EPs */

bool on_request(uint8_t rhport, uint8_t stage,
                arduino_usb_control_request_t const *request) {
  if (stage != REQUEST_STAGE_SETUP) {
    return true; /* DATA / ACK stages: nothing to do */
  }
  if ((request->bmRequestType & 0x60u) != 0x40u) {
    return false; /* not a vendor request */
  }
  const bool in = (request->bmRequestType & 0x80u) != 0;
  const uint8_t *reply = NULL;
  uint16_t len = 0;
  /* Raw packet straight through; the interface-recipient form STALLs with
   * UNSUPPORTED because this transport reports no (FF/49/4F) interface. */
  if (!UsbIo.handle_setup(request->bmRequestType, request->bRequest,
                          request->wValue, request->wIndex, request->wLength,
                          &reply, &len)) {
    return false; /* STALL */
  }
  return vendor.sendResponse(rhport, request,
                             in ? const_cast<uint8_t *>(reply) : NULL,
                             in ? len : 0);
}

} // namespace

uint16_t usbio_transport_begin() {
  vendor.onRequest(on_request);
  vendor.begin(); /* must precede USB.begin(), see the header comment */
  USB.begin();
  return 0; /* generic vendor interface only: host probes by VID 0x303A */
}

#endif /* ARDUINO_ARCH_ESP32 */
