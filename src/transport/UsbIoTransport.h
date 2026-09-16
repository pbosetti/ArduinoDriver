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
 * USBIO_FLAG_VENDOR_INTERFACE and, on a transport with a bulk IN endpoint,
 * USBIO_FLAG_STREAMING. This return value doubles as the streaming capability
 * query: STREAM_START (the only way to make the core call
 * usbio_transport_stream_write() below) itself STALLs USBIO_UNSUPPORTED
 * unless the flag it got back from here is set (see UsbIo.cpp,
 * handle_stream_start()). A transport with no endpoint (Renesas, RP2040,
 * ESP32 today) therefore never needs to define that symbol at all - and on
 * its architecture UsbIo.h's USBIO_HAS_STREAM_TRANSPORT is 0, so the core is
 * compiled without any reference to it. Referencing this symbol from the
 * core also guarantees that the transport object file - and the static USB
 * module it defines - is linked into the sketch.
 */
uint16_t usbio_transport_begin();

/* Outcome of usbio_transport_stream_write() below. The distinction between
 * BUSY and FAILED matters: BUSY is the ordinary state of a healthy stream
 * (poll() produced a record faster than the bus drained the last one) and
 * must never be treated as an error, while FAILED means the host has stopped
 * reading the endpoint altogether. */
enum usbio_stream_write_result {
  USBIO_STREAM_WRITE_SENT = 0,   /* handed to the USB stack                  */
  USBIO_STREAM_WRITE_BUSY = 1,   /* previous packet in flight; retry later    */
  USBIO_STREAM_WRITE_FAILED = 2  /* endpoint is not being drained by the host */
};

/*
 * Largest payload usbio_transport_stream_write() accepts in one call: the
 * bulk IN endpoint's wMaxPacketSize minus one byte. The core packs as many
 * whole records as fit into each packet (UsbIo.cpp, stream_poll()), and
 * keeping every packet one byte short of wMaxPacketSize makes it a short
 * packet, which completes the host's pending bulk read immediately; a
 * completely full packet followed by a pause in the stream would instead sit
 * in the host's transfer until the next packet arrived. Never larger than
 * USBIO_STREAM_PACKET_MAX_LEN (UsbIo.h), the size of the core's packet buffer.
 */
uint16_t usbio_transport_stream_packet_max();

/*
 * Non-blocking write of one bulk packet carrying one or more whole stream
 * records (usbio_stream_header_t + samples [+ digital bitmap], see
 * usbio_protocol.h "Record format") to the bulk IN endpoint. `len` never
 * exceeds usbio_transport_stream_packet_max() - the protocol's channel and
 * pin-count limits guarantee a single record always fits, so a transport
 * never has to split one across calls.
 *
 * Called ONLY from UsbIoDevice::poll() (loop() context), never from the setup
 * callback. On USBIO_STREAM_WRITE_SENT the caller may reuse `data` immediately
 * (the transport owns its own copy); on BUSY it keeps the record queued and
 * retries on a later poll(); on FAILED it counts a strike and stops the stream
 * after UsbIoDevice::StreamTxFailureLimit consecutive ones, so a transport
 * whose send blocks until a timeout (SAMD) cannot stall loop() indefinitely.
 * Only a transport that sets USBIO_FLAG_STREAMING from usbio_transport_begin()
 * above needs to define this - see UsbIo.h, USBIO_HAS_STREAM_TRANSPORT.
 */
uint8_t usbio_transport_stream_write(const uint8_t *data, uint16_t len);

#endif /* USBIO_TRANSPORT_H */
