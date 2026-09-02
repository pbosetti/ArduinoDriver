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
  uint8_t _reply[USBIO_MAX_REPLY_LEN];
};

extern UsbIoDevice UsbIo;

#endif /* USBIO_H */
