/*
 * UsbIoTransport.h - internal seam between the board-agnostic core
 * (UsbIo.cpp) and the USB-stack specific glue in the transport/ sources.
 *
 * Exactly one transport file compiles on a given architecture (selected by
 * the ARDUINO_ARCH_* macros); the others reduce to empty translation units.
 * Every transport funnels the SETUP packets it recognises as vendor requests
 * into UsbIo.handle_setup() and maps the answer onto its stack's ACK / STALL /
 * send-data primitives.
 */
#ifndef USBIO_TRANSPORT_H
#define USBIO_TRANSPORT_H

#include <stdint.h>

/*
 * Called once from UsbIoDevice::begin(). Does whatever runtime setup the stack
 * needs (most stacks hook in at static-initialisation time and need nothing
 * here) and returns the usbio_info_flags bits the transport contributes, e.g.
 * USBIO_FLAG_VENDOR_INTERFACE. Referencing this symbol from the core also
 * guarantees that the transport object file - and the static USB module it
 * defines - is linked into the sketch.
 */
uint16_t usbio_transport_begin();

#endif /* USBIO_TRANSPORT_H */
