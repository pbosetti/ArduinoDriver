/*
 * UsbIo.h - USB vendor-class I/O device for Arduino boards.
 *
 *   #include <UsbIo.h>
 *   void setup() { UsbIo.begin(); }
 *   void loop()  { UsbIo.poll(); }
 *
 * The host drives the board with USB control requests on endpoint 0
 * (usbio_protocol.h is the wire contract). Requests arrive in the USB
 * interrupt: handle_setup() validates them, queues writes and answers reads
 * from a shadow copy of the pin state. poll(), called from loop(), executes
 * the queued Arduino calls and refreshes the shadow. No Arduino I/O call ever
 * runs in interrupt context, which is what keeps the mbed core (mutex-guarded
 * AnalogIn/PwmOut) and the others happy.
 *
 * The library never touches the CDC port: Serial stays available to sketches.
 */
#ifndef USBIO_H
#define USBIO_H

#include <stdint.h>

#include "usbio_protocol.h"

/* Transports that can add a bulk IN endpoint to the vendor interface
 * (usbio_protocol.h, "Streaming"): mbed (transport/pluggable_mbed.cpp) and
 * SAMD (transport/pluggable_samd.cpp). Renesas has fixed TinyUSB descriptors
 * with no vendor interface to hang an endpoint on, and the RP2040/ESP32
 * transports are still unverified for plain control I/O, so on those
 * architectures the whole streaming implementation below is compiled out: no
 * ring, no extra state, GET_INFO never sets USBIO_FLAG_STREAMING and the
 * STREAM_* requests fall straight through to STALL USBIO_UNSUPPORTED. */
#if defined(ARDUINO_ARCH_MBED) || defined(ARDUINO_ARCH_SAMD)
#define USBIO_HAS_STREAM_TRANSPORT 1
#else
#define USBIO_HAS_STREAM_TRANSPORT 0
#endif

class UsbIoDevice {
public:
  UsbIoDevice();

  /* Discover pin capabilities from the core, apply the ADC/PWM resolutions
   * advertised in GET_INFO and start the transport. Call once from setup(). */
  void begin();

  /* Execute queued commands, then refresh the shadow state: every pin in an
   * INPUT* mode is read, plus ONE analog pin per call (round-robin over the
   * pins in ANALOG_IN mode) so loop() stays fast. Call from loop(). */
  void poll();

  /*
   * Transport entry point, INTERRUPT CONTEXT. Decodes one SETUP packet, passed
   * RAW: bmRequestType as received (direction, type and recipient bits) and
   * wIndex untouched. The recipient policy is applied here: device recipient
   * -> pin = wIndex; interface recipient -> pin = wIndex >> 8, accepted only
   * when the transport reported USBIO_FLAG_VENDOR_INTERFACE (else STALL,
   * UNSUPPORTED); any other recipient STALLs with BAD_CMD. A transport that
   * owns an interface must still drop interface-recipient packets addressed
   * to a different interface before calling this.
   * Returns false to STALL. Returns true to ACK (OUT requests, *reply_len ==
   * 0) or to send *reply_len bytes from *reply (IN requests). The reply buffer
   * is a member of this object: it stays valid until the next SETUP packet,
   * which on EP0 cannot arrive before the current transfer has completed or
   * been aborted. *reply_len is already clamped to wLength.
   */
  bool handle_setup(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
                    uint16_t wIndex, uint16_t wLength, const uint8_t **reply,
                    uint16_t *reply_len);

  /*
   * Transport hook, INTERRUPT CONTEXT: called by a transport that can detect
   * losing the USB connection (suspend, disconnect, a bus reset before
   * re-enumeration) so a running stream is stopped and its selection cleared,
   * matching RESET's streaming semantics (usbio_protocol.h, "Streaming": "the
   * selection and counters do not survive a re-enumeration"). A transport
   * with no such callback (SAMD today) simply never calls this. No-op when
   * the build has no streaming transport.
   */
  void handle_usb_disconnected();

  uint8_t pin_count() const { return _n_pins; }
  uint8_t pin_caps(uint8_t pin) const { return pin < _n_pins ? _caps[pin] : 0; }

private:
  /* One queued OUT request. `epoch` is the value of _epoch at enqueue time:
   * RESET bumps _epoch, so stale entries are skipped by the consumer instead
   * of the producer rewinding _tail (only poll() ever writes _tail). */
  struct Command {
    uint8_t cmd;
    uint8_t pin;
    uint8_t epoch;
    uint16_t value;
  };

  static const uint8_t ModeNone = 0xFF; /* pin never configured by the host */

#if USBIO_HAS_STREAM_TRANSPORT
  /* Every record fits in one bulk packet: USBIO_MAX_STREAM_CHANNELS (8) and
   * USBIO_MAX_PINS (128, i.e. a 16-byte digital bitmap) bound header +
   * samples + bitmap at 44 bytes, well under USBIO_STREAM_EP_SIZE (64) - see
   * the USBIO_STATIC_ASSERT below. The device therefore never needs to
   * straddle a record across two packets even though the wire format (and
   * the host's reassembly) allows it. */
  static const uint16_t StreamRecordMaxLen =
      (uint16_t)(sizeof(usbio_stream_header_t) + 2u * USBIO_MAX_STREAM_CHANNELS +
                 ((((USBIO_MAX_PINS + 7u) / 8u) + 1u) & ~1u));
  USBIO_STATIC_ASSERT(StreamRecordMaxLen <= USBIO_STREAM_EP_SIZE,
                      "a stream record must always fit in one bulk packet");

  /* One buffered record, ready to hand to the transport verbatim. */
  struct StreamRecord {
    uint8_t len;
    uint8_t data[StreamRecordMaxLen];
  };

  /* SPSC-style ring, same discipline as _queue below (power-of-two depth,
   * free-running 8-bit indices) - but here both ends run from poll(): the
   * sampler (producer) and the transport drain (consumer) never race an ISR,
   * only each other across poll() calls, so head/tail need no volatile.
   * STREAM_START still has to reset them; it does that through the
   * request/done pair near the bottom of this class instead of touching the
   * indices from interrupt context (mirrors _reset_requests/_reset_done).
   * Depth 32 keeps the ring under 1.5 kB even at the worst-case record size,
   * comfortable on the SAMD21's 32 kB of RAM. */
  static const uint8_t StreamRingDepth = 32;
  USBIO_STATIC_ASSERT((StreamRingDepth & (StreamRingDepth - 1u)) == 0,
                      "stream ring depth must be a power of two");

  /* Consecutive USBIO_STREAM_WRITE_FAILED results that stop the stream and
   * drop its backlog. Only a transport whose send blocks until a timeout
   * (SAMD, ~70 ms) ever reports FAILED, so this bounds what an abandoned
   * stream - a host that exited without STREAM_STOP - can cost loop(): three
   * timeouts once, instead of one on every poll() call for ever. A healthy
   * stream reports BUSY, never FAILED, so this can never stop it. */
  static const uint8_t StreamTxFailureLimit = 3;
#endif

  bool handle_out(uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
                  uint16_t wLength);
  bool handle_in(uint8_t bRequest, uint16_t wIndex, uint16_t wLength,
                 uint16_t *reply_len);
  bool stall(uint8_t reason);
  bool enqueue(uint8_t cmd, uint8_t pin, uint16_t value);
  void request_reset();
  uint8_t queue_pending() const;
  bool busy() const;

  void execute(const Command &c);
  void apply_mode(uint8_t pin, uint8_t mode);
  void apply_reset();
  void refresh_shadow();

  static bool mode_allowed(uint8_t caps, uint8_t mode);
  static bool mode_is_input(uint8_t mode);
  static bool mode_is_digital(uint8_t mode);
  static uint8_t read_pin(uint8_t pin);
  static uint16_t read_analog(uint8_t pin);

#if USBIO_HAS_STREAM_TRANSPORT
  /* ISR side: validate and apply one STREAM_* OUT request. See
   * usbio_protocol.h, "OUT request validation", the "Streaming requests"
   * paragraph, for the exact rule each one enforces. */
  bool handle_stream_select(uint16_t wValue, uint16_t wIndex);
  bool handle_stream_start(uint16_t wValue, uint16_t wIndex);
  bool handle_stream_stop();
  /* Index of `pin` in _stream_channels[0.._stream_n_channels), or -1. */
  int8_t stream_channel_index(uint8_t pin) const;

  /* poll() side: apply a pending START's reset, sample on schedule, drain the
   * ring into the transport. Called once from poll(). */
  void stream_poll();
  /* Format one record for `now` (a micros() timestamp) and push it onto the
   * ring, or drop it and count an overrun if the ring is full. */
  void stream_sample(uint32_t now);
#endif

  uint8_t _n_pins;
  uint8_t _n_ain;
  uint16_t _flags;
  uint8_t _caps[USBIO_MAX_PINS];
  /* Intended mode (usbio_pin_mode or ModeNone), recorded when a PIN_MODE is
   * accepted. Written by the ISR, read by both sides; single bytes, so every
   * access is atomic. */
  volatile uint8_t _mode[USBIO_MAX_PINS];
  /* Shadow state indexed by Arduino pin number (not by AIN ordinal: the ISR
   * must not search). Written by poll(), read by the ISR; byte / aligned
   * halfword stores are atomic on every supported MCU. */
  uint8_t _dio[USBIO_MAX_PINS];
  uint16_t _ain[USBIO_MAX_PINS];
  /* SPSC ring: ISR produces (_head), poll() consumes (_tail). Free-running
   * 8-bit indices, slot = index & (depth - 1). */
  Command _queue[USBIO_QUEUE_DEPTH];
  volatile uint8_t _head;
  volatile uint8_t _tail;
  volatile uint8_t _epoch;
  volatile uint8_t _reset_requests; /* bumped by the ISR on RESET */
  uint8_t _reset_done;              /* last value honoured by poll() */
  volatile uint8_t _last_error;
  uint8_t _ain_cursor;

#if USBIO_HAS_STREAM_TRANSPORT
  /* Selection set, in STREAM_SELECT order. Only mutated while the stream is
   * stopped (SELECT STALLs BUSY while running), so plain uint8_t is enough
   * for the array itself. */
  uint8_t _stream_channels[USBIO_MAX_STREAM_CHANNELS];
  /* Written by the ISR (SELECT/START/STOP, and PIN_MODE/RESET stopping a
   * running stream), read by poll()'s sampler and by GET_STREAM_STATUS;
   * poll() also clears _running itself on an overrun when
   * USBIO_STREAM_FLAG_STOP_ON_OVERRUN was requested, hence volatile. */
  volatile uint8_t _stream_n_channels;
  volatile uint8_t _stream_running;
  volatile uint8_t _stream_flags; /* enum usbio_stream_flags in force */
  volatile uint16_t _stream_period_us;
  /* poll() writes both, GET_STREAM_STATUS (ISR) reads them. */
  volatile uint32_t _stream_seq;
  volatile uint32_t _stream_overruns;
  /* START request/done pair, same discipline as _reset_requests/_reset_done
   * above: the ISR only bumps the counter; poll() does the actual ring/seq/
   * overrun reset, so a SETUP packet can never land mid-reset. */
  volatile uint8_t _stream_start_requests;
  uint8_t _stream_start_done;
  uint32_t _stream_deadline_us; /* poll()-only: next sample's micros() due time */
  StreamRecord _stream_ring[StreamRingDepth];
  uint8_t _stream_head; /* poll()-only producer index */
  uint8_t _stream_tail; /* poll()-only consumer index */
  uint8_t _stream_tx_failures; /* poll()-only: consecutive FAILED writes */
#endif

  uint8_t _reply[USBIO_MAX_REPLY_LEN];
};

extern UsbIoDevice UsbIo;

#endif /* USBIO_H */
