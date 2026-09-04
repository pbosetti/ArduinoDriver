/*
 * UsbIo.cpp - board-agnostic core: request validation (ISR side), command
 * queue, shadow state and the loop()-side executor. See UsbIo.h.
 */
#include "UsbIo.h"

#include <Arduino.h>
#include <string.h>

#include "UsbIoBoard.h"
#include "transport/UsbIoTransport.h"

USBIO_STATIC_ASSERT((USBIO_QUEUE_DEPTH & (USBIO_QUEUE_DEPTH - 1u)) == 0,
                    "queue depth must be a power of two");
USBIO_STATIC_ASSERT(USBIO_QUEUE_DEPTH <= 128u,
                    "8-bit free-running queue indices need depth <= 128");
USBIO_STATIC_ASSERT(USBIO_MAX_REPLY_LEN >= sizeof(usbio_info_t),
                    "reply buffer too small for GET_INFO");
USBIO_STATIC_ASSERT(USBIO_MAX_REPLY_LEN >= USBIO_MAX_PINS,
                    "reply buffer too small for GET_PIN_CAPS");
USBIO_STATIC_ASSERT(USBIO_MAX_REPLY_LEN >=
                        sizeof(usbio_all_header_t) + (USBIO_MAX_PINS + 7u) / 8u,
                    "reply buffer too small for DIO_READ_ALL");
USBIO_STATIC_ASSERT(USBIO_MAX_REPLY_LEN >=
                        sizeof(usbio_all_header_t) + 2u * USBIO_MAX_AIN,
                    "reply buffer too small for AI_READ_ALL");
USBIO_STATIC_ASSERT(USBIO_MAX_REPLY_LEN >= sizeof(usbio_event_header_t) +
                                                USBIO_MAX_EVENTS_PER_POP *
                                                    sizeof(usbio_event_t),
                    "reply buffer too small for EVENT_POP");
USBIO_STATIC_ASSERT(USBIO_MAX_REPLY_LEN >=
                        sizeof(usbio_event_header_t) +
                            USBIO_MAX_EVENT_PINS * sizeof(usbio_event_count_t),
                    "reply buffer too small for EVENT_COUNTS");
USBIO_STATIC_ASSERT((USBIO_EVENT_QUEUE_DEPTH & (USBIO_EVENT_QUEUE_DEPTH - 1u)) == 0,
                    "event queue depth must be a power of two");
/* The cores have a single analogWriteResolution() shared by PWM and DAC, so
 * DAC values are rescaled from dac_bits to pwm_bits before analogWrite(). */
USBIO_STATIC_ASSERT(UsbIoBoard::DacBits <= UsbIoBoard::PwmBits,
                    "dac_bits must not exceed pwm_bits");

UsbIoDevice UsbIo;

namespace {

const uint8_t QueueMask = (uint8_t)(USBIO_QUEUE_DEPTH - 1u);
const uint8_t EventQueueMask = (uint8_t)(USBIO_EVENT_QUEUE_DEPTH - 1u);

/* Producer fills a slot, then publishes it by bumping _head; the consumer
 * copies a slot, then releases it by bumping _tail. Both run on one core, so
 * no hardware fence is needed, but the compiler must not move the slot
 * accesses across the index write. */
inline void compiler_barrier() { __asm__ __volatile__("" ::: "memory"); }

inline uint16_t full_scale(uint8_t bits) {
  return bits >= 16u ? 0xFFFFu : (uint16_t)((1u << bits) - 1u);
}

inline void put_u16(uint8_t *dst, uint16_t v) {
  dst[0] = (uint8_t)(v & 0xFFu);
  dst[1] = (uint8_t)(v >> 8);
}

} // namespace

UsbIoDevice::UsbIoDevice()
    : _n_pins(0), _n_ain(0), _flags(0), _head(0), _tail(0), _epoch(0),
      _reset_requests(0), _reset_done(0), _last_error(USBIO_OK),
      _ain_cursor(0), _event_n_watched(0), _event_arm_seq(0), _event_head(0),
      _event_tail(0), _event_dropped(0)
#if USBIO_HAS_STREAM_TRANSPORT
      ,
      _stream_n_channels(0), _stream_running(0), _stream_flags(0),
      _stream_period_us(0), _stream_seq(0), _stream_overruns(0),
      _stream_start_requests(0), _stream_start_done(0), _stream_deadline_us(0),
      _stream_head(0), _stream_tail(0), _stream_tx_failures(0)
#endif
{
  for (unsigned p = 0; p < USBIO_MAX_PINS; ++p) {
    _caps[p] = 0;
    _mode[p] = ModeNone;
    _dio[p] = 0;
    _ain[p] = 0;
  }
  memset(_queue, 0, sizeof(_queue));
  memset(_reply, 0, sizeof(_reply));
  memset(_event_watch, 0, sizeof(_event_watch));
  memset(_event_ring, 0, sizeof(_event_ring));
#if USBIO_HAS_STREAM_TRANSPORT
  memset(_stream_channels, 0, sizeof(_stream_channels));
  memset(_stream_ring, 0, sizeof(_stream_ring));
#endif
}

/* ---- setup ------------------------------------------------------------- */

void UsbIoDevice::begin() {
  _n_pins = UsbIoBoard::pin_count();
  _n_ain = 0;
  for (uint8_t p = 0; p < _n_pins; ++p) {
    uint8_t caps = 0;
    if (UsbIoBoard::pin_has_dio(p)) {
      caps |= USBIO_CAP_DIO;
    }
    if (UsbIoBoard::pin_has_ain(p) && _n_ain < USBIO_MAX_AIN) {
      caps |= USBIO_CAP_AIN;
      ++_n_ain;
    }
    if (UsbIoBoard::pin_has_pwm(p)) {
      caps |= USBIO_CAP_PWM;
    }
    if (UsbIoBoard::DacBits != 0 && UsbIoBoard::pin_has_dac(p)) {
      caps |= USBIO_CAP_DAC;
    }
    _caps[p] = caps;
    _mode[p] = ModeNone;
    _dio[p] = 0;
    _ain[p] = 0;
  }
  /* Pins are deliberately left untouched here: a pin is only reconfigured
   * once the host asks for it (PIN_MODE or RESET), so a sketch's own use of
   * Serial1 / SPI / LEDs keeps working until the host takes over. */
  analogReadResolution(UsbIoBoard::AdcBits);
  analogWriteResolution(UsbIoBoard::PwmBits);
  _flags = UsbIoBoard::HasPulldown ? USBIO_FLAG_PULLDOWN : 0;
  /* usbio_protocol.h, "Pin events": events are plain control transfers plus
   * software scanning, no endpoint needed, so every board gets them -
   * unlike USBIO_FLAG_STREAMING below, this does not depend on the transport. */
  _flags |= USBIO_FLAG_EVENTS;
  _flags |= usbio_transport_begin();
}

/* ---- interrupt side ---------------------------------------------------- */

bool UsbIoDevice::handle_setup(uint8_t bmRequestType, uint8_t bRequest,
                               uint16_t wValue, uint16_t wIndex,
                               uint16_t wLength, const uint8_t **reply,
                               uint16_t *reply_len) {
  *reply = _reply;
  *reply_len = 0;
  if ((bmRequestType & 0x60u) != 0x40u) {
    return false; /* not a vendor request; transports filter these already */
  }
  /* Recipient policy lives here so that every transport can hand over the raw
   * packet: the device-recipient form carries the pin in wIndex, the
   * interface-recipient form (USBIO_REQTYPE_*_ITF) carries
   * (pin << 8) | interface and only exists on stacks that expose the vendor
   * interface. */
  uint16_t index;
  switch (bmRequestType & 0x1Fu) {
  case 0: /* device */
    index = wIndex;
    break;
  case 1: /* interface */
    if (!(_flags & USBIO_FLAG_VENDOR_INTERFACE)) {
      return stall(USBIO_UNSUPPORTED);
    }
    index = (uint16_t)(wIndex >> 8);
    break;
  default: /* endpoint / other: not part of the protocol */
    return stall(USBIO_BAD_CMD);
  }
  if (bmRequestType & 0x80u) {
    return handle_in(bRequest, index, wLength, reply_len);
  }
  return handle_out(bRequest, wValue, index, wLength);
}

bool UsbIoDevice::stall(uint8_t reason) {
  _last_error = reason;
  return false;
}

uint8_t UsbIoDevice::queue_pending() const {
  return (uint8_t)(_head - _tail);
}

bool UsbIoDevice::busy() const {
  return queue_pending() != 0 || _reset_done != _reset_requests;
}

bool UsbIoDevice::enqueue(uint8_t cmd, uint8_t pin, uint16_t value) {
  const uint8_t head = _head;
  if ((uint8_t)(head - _tail) >= USBIO_QUEUE_DEPTH) {
    return false;
  }
  Command &slot = _queue[head & QueueMask];
  slot.cmd = cmd;
  slot.pin = pin;
  slot.epoch = _epoch;
  slot.value = value;
  compiler_barrier();
  _head = (uint8_t)(head + 1u);
  return true;
}

void UsbIoDevice::request_reset() {
  /* Order matters: invalidate queued work first, then publish the new
   * intended modes, then ask poll() to reconfigure the hardware. Commands
   * accepted after this point carry the new epoch and survive. */
  _epoch = (uint8_t)(_epoch + 1u);
  for (uint8_t p = 0; p < _n_pins; ++p) {
    _mode[p] = (_caps[p] & USBIO_CAP_DIO) ? (uint8_t)USBIO_MODE_INPUT : ModeNone;
  }
#if USBIO_HAS_STREAM_TRANSPORT
  /* usbio_protocol.h, "Streaming": RESET stops a running stream and clears
   * its selection (plain field writes, no Arduino I/O call - ISR-safe). */
  _stream_running = 0;
  _stream_n_channels = 0;
#endif
  /* usbio_protocol.h, "Event requests": RESET unwatches every pin and clears
   * the queue. Both happen here, in the ISR: these are single-byte writes
   * with no Arduino I/O, so it is ISR-safe exactly like the streaming reset
   * above, and a CONFIG racing in right behind RESET is then correctly
   * ordered after the clear.
   *
   * The queue is discarded by moving _event_tail - the index this side owns
   * (EVENT_POP advances it) - up to a snapshot of poll()'s _event_head. Doing
   * it the other way round, from poll(), would be a real hazard: poll() would
   * have to write _event_head from a snapshot of _event_tail, and an
   * EVENT_POP landing in between would leave tail one *ahead* of head, so
   * (uint8_t)(head - tail) would read as 255 - a permanently "full" ring that
   * drops every later edge. Moving tail forward can never overtake head, and
   * at worst an edge pushed in the same instant as the RESET survives it. */
  for (uint8_t i = 0; i < USBIO_MAX_EVENT_PINS; ++i) {
    _event_watch[i].active = 0;
  }
  _event_n_watched = 0;
  _event_tail = _event_head;
  _event_dropped = 0;
  _reset_requests = (uint8_t)(_reset_requests + 1u);
}

bool UsbIoDevice::mode_allowed(uint8_t caps, uint8_t mode) {
  switch (mode) {
  case USBIO_MODE_INPUT:
  case USBIO_MODE_OUTPUT:
  case USBIO_MODE_INPUT_PULLUP:
    return (caps & USBIO_CAP_DIO) != 0;
  case USBIO_MODE_INPUT_PULLDOWN:
    return (caps & USBIO_CAP_DIO) != 0 && UsbIoBoard::HasPulldown;
  case USBIO_MODE_ANALOG_IN:
    return (caps & USBIO_CAP_AIN) != 0;
  case USBIO_MODE_PWM:
    return (caps & USBIO_CAP_PWM) != 0;
  case USBIO_MODE_DAC:
    return (caps & USBIO_CAP_DAC) != 0;
  default:
    return false;
  }
}

bool UsbIoDevice::mode_is_input(uint8_t mode) {
  return mode == USBIO_MODE_INPUT || mode == USBIO_MODE_INPUT_PULLUP ||
         mode == USBIO_MODE_INPUT_PULLDOWN;
}

bool UsbIoDevice::mode_is_digital(uint8_t mode) {
  return mode_is_input(mode) || mode == USBIO_MODE_OUTPUT;
}

bool UsbIoDevice::handle_out(uint8_t bRequest, uint16_t wValue,
                             uint16_t wIndex, uint16_t wLength) {
  if (wLength != 0) {
    return stall(USBIO_BAD_CMD); /* the protocol has no OUT data stage */
  }
  switch (bRequest) {
  case USBIO_REQ_RESET:
    request_reset();
    return true;
  case USBIO_REQ_STREAM_SELECT:
#if USBIO_HAS_STREAM_TRANSPORT
    return handle_stream_select(wValue, wIndex);
#else
    return stall(USBIO_UNSUPPORTED);
#endif
  case USBIO_REQ_STREAM_START:
#if USBIO_HAS_STREAM_TRANSPORT
    return handle_stream_start(wValue, wIndex);
#else
    return stall(USBIO_UNSUPPORTED);
#endif
  case USBIO_REQ_STREAM_STOP:
#if USBIO_HAS_STREAM_TRANSPORT
    return handle_stream_stop();
#else
    return stall(USBIO_UNSUPPORTED);
#endif
  case USBIO_REQ_PIN_MODE:
  case USBIO_REQ_DIO_WRITE:
  case USBIO_REQ_PWM_WRITE:
  case USBIO_REQ_DAC_WRITE:
  case USBIO_REQ_EVENT_CONFIG:
    break;
  default:
    return stall(USBIO_BAD_CMD);
  }
  if (wIndex >= _n_pins) {
    return stall(USBIO_BAD_PIN);
  }
  const uint8_t pin = (uint8_t)wIndex;
  const uint8_t mode = _mode[pin];

  switch (bRequest) {
  case USBIO_REQ_PIN_MODE:
    if (wValue >= USBIO_MODE_COUNT) {
      return stall(USBIO_BAD_VALUE);
    }
    if (!mode_allowed(_caps[pin], (uint8_t)wValue)) {
      return stall(USBIO_UNSUPPORTED);
    }
    if (!enqueue(bRequest, pin, wValue)) {
      return stall(USBIO_QUEUE_FULL);
    }
    /* Record the intent now, not at execution: a DIO_WRITE that follows
     * immediately must validate against the new mode (usbio_protocol.h,
     * "Mode bookkeeping"). */
    _mode[pin] = (uint8_t)wValue;
#if USBIO_HAS_STREAM_TRANSPORT
    /* usbio_protocol.h, "Streaming": changing the mode of a selected pin
     * stops a running stream (the selection is left alone, like STOP). */
    if (_stream_running && stream_channel_index(pin) >= 0) {
      _stream_running = 0;
    }
#endif
    /* usbio_protocol.h, "Event requests": taking a watched pin out of an
     * INPUT* mode unwatches it, but its queued events stay readable (unlike
     * EVENT_CONFIG's OFF, this does not touch the ring). */
    if (!mode_is_input((uint8_t)wValue)) {
      const int8_t idx = event_watch_index(pin);
      if (idx >= 0) {
        _event_watch[idx].active = 0;
        --_event_n_watched;
      }
    }
    return true;
  case USBIO_REQ_DIO_WRITE:
    if (mode != USBIO_MODE_OUTPUT) {
      return stall(USBIO_BAD_MODE);
    }
    return enqueue(bRequest, pin, wValue ? 1u : 0u) ? true
                                                     : stall(USBIO_QUEUE_FULL);
  case USBIO_REQ_PWM_WRITE:
    if (mode != USBIO_MODE_PWM) {
      return stall(USBIO_BAD_MODE);
    }
    if (wValue > full_scale(UsbIoBoard::PwmBits)) {
      return stall(USBIO_BAD_VALUE);
    }
    return enqueue(bRequest, pin, wValue) ? true : stall(USBIO_QUEUE_FULL);
  case USBIO_REQ_DAC_WRITE:
    if (mode != USBIO_MODE_DAC) {
      return stall(USBIO_BAD_MODE);
    }
    if (wValue > full_scale(UsbIoBoard::DacBits)) {
      return stall(USBIO_BAD_VALUE);
    }
    return enqueue(bRequest, pin, wValue) ? true : stall(USBIO_QUEUE_FULL);
  case USBIO_REQ_EVENT_CONFIG:
    return handle_event_config(pin, wValue);
  default:
    return stall(USBIO_BAD_CMD);
  }
}

bool UsbIoDevice::handle_in(uint8_t bRequest, uint16_t wIndex,
                            uint16_t wLength, uint16_t *reply_len) {
  uint16_t len = 0;
  const bool pending = busy();

  switch (bRequest) {
  case USBIO_REQ_GET_INFO: {
    usbio_info_t info;
    memset(&info, 0, sizeof(info));
    memcpy(info.magic, USBIO_MAGIC, USBIO_MAGIC_LEN);
    info.protocol_version = USBIO_PROTOCOL_VERSION;
    info.board_id = UsbIoBoard::BoardId;
    info.n_pins = _n_pins;
    info.n_ain = _n_ain;
    info.adc_bits = UsbIoBoard::AdcBits;
    info.pwm_bits = UsbIoBoard::PwmBits;
    info.dac_bits = UsbIoBoard::DacBits;
    info.queue_depth = (uint8_t)USBIO_QUEUE_DEPTH;
    info.vref_mv = UsbIoBoard::VrefMv;
    info.io_mv = UsbIoBoard::IoMv;
    info.flags = _flags;
    info.stream_max_channels =
        (_flags & USBIO_FLAG_STREAMING) ? (uint8_t)USBIO_MAX_STREAM_CHANNELS : 0;
    info.event_max_pins =
        (_flags & USBIO_FLAG_EVENTS) ? (uint8_t)USBIO_MAX_EVENT_PINS : 0;
    memcpy(_reply, &info, sizeof(info));
    len = sizeof(info);
    break;
  }
  case USBIO_REQ_GET_PIN_CAPS:
    if (wIndex < _n_pins) {
      len = (uint16_t)(_n_pins - wIndex);
      if (len > wLength) {
        len = wLength;
      }
      memcpy(_reply, &_caps[wIndex], len);
    }
    break; /* wIndex >= n_pins: zero-length reply */
  case USBIO_REQ_GET_STATUS:
    _reply[0] = USBIO_OK;
    _reply[1] = queue_pending();
    _reply[2] = _last_error;
    _reply[3] = 0;
    _last_error = USBIO_OK;
    len = sizeof(usbio_status_reply_t);
    break;
  case USBIO_REQ_GET_TIME: {
    /* usbio_protocol.h, GET_TIME: served straight from here, not from a
     * shadow - millis()/micros() are lock-free ticker reads on every
     * supported stack (verified: mbed's Timer/LowPowerTimer are documented
     * "Synchronization level: Interrupt safe"; SAMD's millis() is a plain
     * volatile word read and its micros() a lock-free retry loop against
     * SysTick; Renesas's millis() is a plain volatile word read and its
     * micros() brackets a register read with NVIC_DisableIRQ/EnableIRQ, not
     * a mutex) - so the reply is the device's clock at the moment this
     * request arrived, not at the last poll(). No `pending`/BUSY check: nothing
     * here is shadow state. */
    usbio_time_reply_t t;
    t.status = USBIO_OK;
    t.reserved = 0;
    t.reserved2 = 0;
    t.millis = millis();
    t.micros = micros();
    memcpy(_reply, &t, sizeof(t));
    len = sizeof(t);
    break;
  }
  case USBIO_REQ_DIO_READ: {
    uint8_t status = USBIO_OK;
    uint8_t value = 0;
    if (wIndex >= _n_pins) {
      status = USBIO_BAD_PIN;
    } else if (!mode_is_digital(_mode[wIndex])) {
      status = USBIO_BAD_MODE;
    } else if (pending) {
      status = USBIO_BUSY;
    } else {
      value = _dio[wIndex];
    }
    _reply[0] = status;
    _reply[1] = value;
    len = sizeof(usbio_dio_reply_t);
    break;
  }
  case USBIO_REQ_AI_READ: {
    uint8_t status = USBIO_OK;
    uint16_t raw = 0;
    if (wIndex >= _n_pins) {
      status = USBIO_BAD_PIN;
    } else if (_mode[wIndex] != USBIO_MODE_ANALOG_IN) {
      status = USBIO_BAD_MODE;
    } else if (pending) {
      status = USBIO_BUSY;
    } else {
      raw = _ain[wIndex];
    }
    _reply[0] = status;
    _reply[1] = 0;
    put_u16(&_reply[2], raw);
    len = sizeof(usbio_ai_reply_t);
    break;
  }
  case USBIO_REQ_DIO_READ_ALL: {
    const uint16_t nbytes = (uint16_t)((_n_pins + 7u) / 8u);
    _reply[0] = pending ? USBIO_BUSY : USBIO_OK;
    _reply[1] = 0;
    memset(&_reply[2], 0, nbytes);
    for (uint8_t p = 0; p < _n_pins; ++p) {
      if (mode_is_digital(_mode[p]) && _dio[p]) {
        _reply[2u + (p >> 3)] |= (uint8_t)(1u << (p & 7u));
      }
    }
    len = (uint16_t)(sizeof(usbio_all_header_t) + nbytes);
    break;
  }
  case USBIO_REQ_AI_READ_ALL: {
    _reply[0] = pending ? USBIO_BUSY : USBIO_OK;
    _reply[1] = 0;
    uint16_t off = sizeof(usbio_all_header_t);
    for (uint8_t p = 0; p < _n_pins; ++p) {
      if (_caps[p] & USBIO_CAP_AIN) {
        put_u16(&_reply[off], _mode[p] == USBIO_MODE_ANALOG_IN ? _ain[p] : 0u);
        off = (uint16_t)(off + 2u);
      }
    }
    len = off;
    break;
  }
  case USBIO_REQ_STREAM_STATUS:
#if USBIO_HAS_STREAM_TRANSPORT
    if (!(_flags & USBIO_FLAG_STREAMING)) {
      return stall(USBIO_UNSUPPORTED);
    }
    {
      usbio_stream_status_t st;
      memset(&st, 0, sizeof(st));
      st.status = USBIO_OK;
      st.running = _stream_running;
      st.n_channels = _stream_n_channels;
      st.flags = _stream_flags;
      st.period_us = _stream_period_us;
      /* _stream_seq is the seq the NEXT record will use (see stream_sample());
       * report the last one actually produced, 0 before the first. */
      st.seq = _stream_seq == 0 ? 0 : _stream_seq - 1u;
      st.overruns = _stream_overruns;
      memcpy(_reply, &st, sizeof(st));
      len = sizeof(st);
    }
    break;
#else
    return stall(USBIO_UNSUPPORTED);
#endif
  case USBIO_REQ_EVENT_POP: {
    /* usbio_protocol.h, "Event requests": never STALLs on an events build -
     * status is always OK (usbio_event_header_t). No `pending`/BUSY check:
     * EVENT_CONFIG applies synchronously in the ISR, so the watch table and
     * ring are never "queued" the way pin commands are. */
    uint16_t cap = wIndex;
    if (cap == 0 || cap > USBIO_MAX_EVENTS_PER_POP) {
      cap = USBIO_MAX_EVENTS_PER_POP;
    }
    usbio_event_header_t hdr;
    hdr.status = USBIO_OK;
    hdr.count = 0;
    uint8_t tail = _event_tail;
    uint16_t off = sizeof(hdr);
    while (tail != _event_head && hdr.count < cap) {
      const usbio_event_t &slot = _event_ring[tail & EventQueueMask];
      tail = (uint8_t)(tail + 1u);
      if (slot.pin == EventTombstone) {
        continue; /* discarded by a since-issued EVENT_CONFIG OFF; skip it,
                   * don't count it against `cap` or the reply */
      }
      memcpy(&_reply[off], &slot, sizeof(slot));
      off = (uint16_t)(off + sizeof(slot));
      ++hdr.count;
    }
    _event_tail = tail; /* ISR-only index: safe to advance from here */
    hdr.dropped = _event_dropped;
    _event_dropped = 0; /* usbio_protocol.h: "Reset dropped when it is
                         * reported by a pop" */
    hdr.pending = count_queued_events();
    memcpy(_reply, &hdr, sizeof(hdr));
    len = off;
    break;
  }
  case USBIO_REQ_EVENT_COUNTS: {
    /* usbio_protocol.h: "entries appear in the order the pins were armed" -
     * the watch table itself never moves (UsbIo.h, EventWatch), so sort the
     * currently-active slots by arm_seq instead. At most USBIO_MAX_EVENT_PINS
     * (8) entries: a plain insertion sort is cheap enough for ISR context. */
    uint8_t order[USBIO_MAX_EVENT_PINS];
    uint8_t n = 0;
    for (uint8_t i = 0; i < USBIO_MAX_EVENT_PINS; ++i) {
      if (_event_watch[i].active) {
        order[n++] = i;
      }
    }
    for (uint8_t a = 1; a < n; ++a) {
      const uint8_t key = order[a];
      int8_t b = (int8_t)(a - 1u);
      while (b >= 0 && (int8_t)(_event_watch[order[b]].arm_seq -
                                 _event_watch[key].arm_seq) > 0) {
        order[b + 1] = order[b];
        --b;
      }
      order[b + 1] = key;
    }
    usbio_event_header_t hdr;
    hdr.status = USBIO_OK;
    hdr.dropped = 0;  /* meaningful only for EVENT_POP replies */
    hdr.pending = 0;  /* meaningful only for EVENT_POP replies */
    hdr.count = n;
    uint16_t off = sizeof(hdr);
    for (uint8_t k = 0; k < n; ++k) {
      const EventWatch &w = _event_watch[order[k]];
      usbio_event_count_t ec;
      ec.pin = w.pin;
      ec.mode = w.edge_mode;
      ec.count = w.count;
      memcpy(&_reply[off], &ec, sizeof(ec));
      off = (uint16_t)(off + sizeof(ec));
    }
    memcpy(_reply, &hdr, sizeof(hdr));
    len = off;
    break;
  }
  default:
    return stall(USBIO_BAD_CMD); /* unknown bRequest: the only IN request that STALLs */
  }

  if (len > wLength) {
    len = wLength;
  }
  *reply_len = len;
  return true;
}

/* ---- loop() side ------------------------------------------------------- */

void UsbIoDevice::poll() {
  do {
    while (_reset_done != _reset_requests) {
      const uint8_t seen = _reset_requests;
      apply_reset();
      _reset_done = seen;
    }
    while (_tail != _head) {
      const uint8_t tail = _tail;
      const Command c = _queue[tail & QueueMask];
      compiler_barrier();
      _tail = (uint8_t)(tail + 1u);
      if (c.epoch == _epoch) {
        execute(c);
      }
    }
    /* A RESET that landed while draining must be applied before we return,
     * otherwise a read could observe pre-reset hardware with an empty queue. */
  } while (_reset_done != _reset_requests);
  refresh_shadow();
  /* usbio_protocol.h, "Pin events": detection compares against the shadow
   * refresh_shadow() just took, so it must run right after it. */
  event_poll();
#if USBIO_HAS_STREAM_TRANSPORT
  stream_poll();
#endif
}

void UsbIoDevice::execute(const Command &c) {
  switch (c.cmd) {
  case USBIO_REQ_PIN_MODE:
    apply_mode(c.pin, (uint8_t)c.value);
    break;
  case USBIO_REQ_DIO_WRITE:
    digitalWrite(c.pin, c.value ? HIGH : LOW);
    _dio[c.pin] = c.value ? 1u : 0u;
    break;
  case USBIO_REQ_PWM_WRITE:
    analogWrite(c.pin, (int)c.value);
    break;
  case USBIO_REQ_DAC_WRITE:
    analogWrite(c.pin,
                (int)((unsigned)c.value
                      << (UsbIoBoard::PwmBits - UsbIoBoard::DacBits)));
    break;
  default:
    break;
  }
}

void UsbIoDevice::apply_mode(uint8_t pin, uint8_t mode) {
  switch (mode) {
  case USBIO_MODE_INPUT:
    pinMode(pin, INPUT);
    _dio[pin] = read_pin(pin);
    break;
  case USBIO_MODE_OUTPUT:
    /* Cores leave whatever level the pad had; mirror it instead of guessing
     * so the first DIO_READ after PIN_MODE(OUTPUT) is truthful. */
    pinMode(pin, OUTPUT);
    _dio[pin] = read_pin(pin);
    break;
  case USBIO_MODE_INPUT_PULLUP:
    pinMode(pin, INPUT_PULLUP);
    _dio[pin] = read_pin(pin);
    break;
  case USBIO_MODE_INPUT_PULLDOWN:
    if (UsbIoBoard::HasPulldown) {
      pinMode(pin, INPUT_PULLDOWN);
    } else {
      pinMode(pin, INPUT); /* unreachable: rejected at validation */
    }
    _dio[pin] = read_pin(pin);
    break;
  case USBIO_MODE_ANALOG_IN:
    /* analogRead() configures the pad itself on every supported core; take a
     * first sample so the shadow is not stale until the round-robin arrives. */
    _ain[pin] = read_analog(pin);
    break;
  case USBIO_MODE_PWM:
  case USBIO_MODE_DAC:
    analogWrite(pin, 0); /* claims the timer / DAC channel, output at 0 */
    break;
  default:
    break;
  }
}

void UsbIoDevice::apply_reset() {
  for (uint8_t p = 0; p < _n_pins; ++p) {
    if (_caps[p] & USBIO_CAP_DIO) {
      pinMode(p, INPUT);
      _dio[p] = read_pin(p);
    } else {
      _dio[p] = 0;
    }
    _ain[p] = 0;
  }
  _ain_cursor = 0;
  /* The event ring is NOT cleared here: that happens in request_reset(), on
   * the side that owns _event_tail. See the comment there. */
}

void UsbIoDevice::refresh_shadow() {
  if (_n_pins == 0) {
    return;
  }
  for (uint8_t p = 0; p < _n_pins; ++p) {
    if (mode_is_input(_mode[p])) {
      _dio[p] = read_pin(p);
    }
  }
  /* One analogRead() per poll(): an ADC conversion costs tens of
   * microseconds and there may be many analog pins. */
  for (uint8_t i = 0; i < _n_pins; ++i) {
    const uint8_t p = _ain_cursor;
    _ain_cursor = (uint8_t)((_ain_cursor + 1u) % _n_pins);
    if (_mode[p] == USBIO_MODE_ANALOG_IN) {
      _ain[p] = read_analog(p);
      break;
    }
  }
}

uint8_t UsbIoDevice::read_pin(uint8_t pin) {
  return digitalRead(pin) == HIGH ? 1u : 0u;
}

uint16_t UsbIoDevice::read_analog(uint8_t pin) {
  const int v = analogRead(pin);
  if (v < 0) {
    return 0; /* mbed returns -1 for a pin without an ADC channel */
  }
  return v > 0xFFFF ? 0xFFFFu : (uint16_t)v;
}

/* ---- pin events -----------------------------------------------------------
 * usbio_protocol.h, "Pin events": handle_event_config() runs in the ISR
 * exactly like handle_out()'s other cases (validate, update state, never
 * touch Arduino I/O); event_poll()/push_event() run from poll() like
 * refresh_shadow(). Unlike streaming, none of this is behind a transport
 * macro - every board gets events (UsbIo::begin() always sets the flag).
 */

int8_t UsbIoDevice::event_watch_index(uint8_t pin) const {
  for (uint8_t i = 0; i < USBIO_MAX_EVENT_PINS; ++i) {
    if (_event_watch[i].active && _event_watch[i].pin == pin) {
      return (int8_t)i;
    }
  }
  return -1;
}

uint8_t UsbIoDevice::find_free_event_slot() const {
  for (uint8_t i = 0; i < USBIO_MAX_EVENT_PINS; ++i) {
    if (!_event_watch[i].active) {
      return i;
    }
  }
  return 0; /* unreachable: the caller already checked _event_n_watched */
}

void UsbIoDevice::discard_events_for_pin(uint8_t pin) {
  /* usbio_protocol.h: OFF "discards its queued events" - just this pin's,
   * unlike RESET's whole-queue clear, so other pins' queued events must
   * survive. _event_head is poll()-only (push_event()), so instead of
   * touching the indices this tombstones matching slots in place: entries
   * between _event_tail and _event_head are already published and poll()
   * never revisits a slot once past it, so mutating their contents here is
   * safe. */
  for (uint8_t i = _event_tail; i != _event_head; ++i) {
    usbio_event_t &slot = _event_ring[i & EventQueueMask];
    if (slot.pin == pin) {
      slot.pin = EventTombstone;
    }
  }
}

uint8_t UsbIoDevice::count_queued_events() const {
  uint8_t n = 0;
  for (uint8_t i = _event_tail; i != _event_head; ++i) {
    if (_event_ring[i & EventQueueMask].pin != EventTombstone) {
      ++n;
    }
  }
  return n;
}

bool UsbIoDevice::handle_event_config(uint8_t pin, uint16_t wValue) {
  /* usbio_protocol.h, "Event requests": pin range was checked by
   * handle_out()'s shared block. OFF is handled first and unconditionally -
   * unwatching skips the capability/mode check on purpose, so a host can
   * always stop watching a pin even after moving it to another mode (which
   * already unwatched it). Arming then validates capability + mode, the edge
   * value, and capacity, in that order. */
  const uint8_t edge = (uint8_t)(wValue & 0xFFu);
  int8_t idx = event_watch_index(pin);
  if (edge == USBIO_EDGE_OFF) {
    /* Unwatching an already-unwatched pin is a no-op (mirrors
     * STREAM_SELECT's remove). */
    if (idx >= 0) {
      _event_watch[idx].active = 0;
      --_event_n_watched;
      discard_events_for_pin(pin);
    }
    return true;
  }
  if (!(_caps[pin] & USBIO_CAP_DIO) || !mode_is_input(_mode[pin])) {
    return stall(USBIO_BAD_MODE);
  }
  if (edge >= USBIO_EDGE_COUNT) {
    return stall(USBIO_BAD_VALUE);
  }
  const uint8_t debounce_ms = (uint8_t)(wValue >> 8);
  if (idx < 0) {
    /* Arming a pin that is not yet watched when event_max_pins are already
     * watched -> BAD_VALUE (usbio_protocol.h, "Event requests"). */
    if (_event_n_watched >= USBIO_MAX_EVENT_PINS) {
      return stall(USBIO_BAD_VALUE);
    }
    idx = (int8_t)find_free_event_slot();
    ++_event_n_watched;
  }
  /* Re-arming a watched pin replaces its mode and debounce and resets its
   * counter (usbio_protocol.h). Fill every field the ISR owns before
   * publishing via `active`/`arm_seq`, exactly like enqueue() publishes a
   * queue slot: event_poll() reads `active` first and must never see a
   * slot with a fresh arm_seq but a stale pin/edge_mode, or vice versa. */
  EventWatch &w = _event_watch[idx];
  w.pin = pin;
  w.edge_mode = edge;
  w.debounce_ms = debounce_ms;
  compiler_barrier();
  ++_event_arm_seq;
  w.arm_seq = _event_arm_seq; /* poll() notices this changed and resets
                               * `count` itself - see event_poll(). */
  compiler_barrier();
  w.active = 1;
  return true;
}

void UsbIoDevice::push_event(uint8_t pin, uint8_t edge, uint16_t seq,
                             uint32_t now) {
  if ((uint8_t)(_event_head - _event_tail) >= USBIO_EVENT_QUEUE_DEPTH) {
    /* usbio_protocol.h, "Pin events": drop the newest edge and saturate
     * `dropped` - the per-pin counter was already bumped by the caller, so
     * the host can still recover the exact edge count even though this
     * edge's timing is lost. */
    if (_event_dropped < 0xFFu) {
      ++_event_dropped;
    }
    return;
  }
  usbio_event_t &slot = _event_ring[_event_head & EventQueueMask];
  slot.pin = pin;
  slot.edge = edge;
  slot.seq = seq;
  slot.t_ms = now;
  compiler_barrier();
  _event_head = (uint8_t)(_event_head + 1u);
}

void UsbIoDevice::event_poll() {
  if (_n_pins == 0) {
    return;
  }
  const uint32_t now = millis();
  for (uint8_t i = 0; i < USBIO_MAX_EVENT_PINS; ++i) {
    EventWatch &w = _event_watch[i];
    if (!w.active) {
      continue;
    }
    const uint8_t pin = w.pin;
    if (w.seen_seq != w.arm_seq) {
      /* Fresh (re)arm: usbio_protocol.h says re-arming "resets its counter" -
       * `count` is poll()-owned (see UsbIo.h, EventWatch) precisely so this
       * reset can never race EVENT_CONFIG bumping arm_seq from the ISR.
       * Baseline against the shadow refresh_shadow() just took instead of
       * comparing, so the first scan after an arm never reports a spurious
       * edge, and start the debounce window already elapsed so a genuine
       * first edge is not blocked. */
      w.seen_seq = w.arm_seq;
      w.count = 0;
      w.prev_level = _dio[pin];
      w.debounce_until = now;
      continue;
    }
    const uint8_t level = _dio[pin];
    if (level == w.prev_level) {
      continue;
    }
    w.prev_level = level;
    const bool rising = level != 0;
    const bool matches = w.edge_mode == USBIO_EDGE_CHANGE ||
                          (rising ? w.edge_mode == USBIO_EDGE_RISING
                                  : w.edge_mode == USBIO_EDGE_FALLING);
    if (!matches || (int32_t)(now - w.debounce_until) < 0) {
      continue; /* wrong direction for the armed mode, or still inside the
                 * debounce window - "first edge wins" (usbio_protocol.h,
                 * "Pin events"): absorbed, not counted, not queued.        */
    }
    ++w.count; /* wraps; matches usbio_event_t.seq / usbio_event_count_t.count */
    w.debounce_until = now + w.debounce_ms;
    push_event(pin, rising ? USBIO_EDGE_RISING : USBIO_EDGE_FALLING, w.count,
               now);
  }
}

/* ---- streaming ----------------------------------------------------------
 * usbio_protocol.h, "Streaming": handle_stream_* run in the ISR exactly like
 * handle_out()'s other cases (validate, update state, never touch Arduino
 * I/O); stream_poll()/stream_sample() run from poll() like refresh_shadow().
 * Both halves only exist when USBIO_HAS_STREAM_TRANSPORT (UsbIo.h).
 */

void UsbIoDevice::handle_usb_disconnected() {
#if USBIO_HAS_STREAM_TRANSPORT
  _stream_running = 0;
  _stream_n_channels = 0;
#endif
}

#if USBIO_HAS_STREAM_TRANSPORT

int8_t UsbIoDevice::stream_channel_index(uint8_t pin) const {
  for (uint8_t i = 0; i < _stream_n_channels; ++i) {
    if (_stream_channels[i] == pin) {
      return (int8_t)i;
    }
  }
  return -1;
}

bool UsbIoDevice::handle_stream_select(uint16_t wValue, uint16_t wIndex) {
  if (!(_flags & USBIO_FLAG_STREAMING)) {
    return stall(USBIO_UNSUPPORTED);
  }
  if (_stream_running) {
    return stall(USBIO_BUSY);
  }
  if (wIndex >= _n_pins) {
    return stall(USBIO_BAD_PIN);
  }
  const uint8_t pin = (uint8_t)wIndex;
  const uint8_t mode = _mode[pin];
  if (mode != USBIO_MODE_ANALOG_IN && !mode_is_input(mode)) {
    return stall(USBIO_BAD_MODE);
  }
  if (wValue > 1u) {
    return stall(USBIO_BAD_VALUE);
  }
  const int8_t idx = stream_channel_index(pin);
  if (wValue != 0) {
    /* add: selecting an already-selected pin is a no-op */
    if (idx >= 0) {
      return true;
    }
    if (_stream_n_channels >= USBIO_MAX_STREAM_CHANNELS) {
      return stall(USBIO_BAD_VALUE);
    }
    _stream_channels[_stream_n_channels] = pin;
    ++_stream_n_channels;
  } else {
    /* remove: an unselected pin is a no-op */
    if (idx < 0) {
      return true;
    }
    for (uint8_t i = (uint8_t)idx; (uint8_t)(i + 1u) < _stream_n_channels; ++i) {
      _stream_channels[i] = _stream_channels[i + 1u];
    }
    --_stream_n_channels;
  }
  return true;
}

bool UsbIoDevice::handle_stream_start(uint16_t wValue, uint16_t wIndex) {
  if (!(_flags & USBIO_FLAG_STREAMING)) {
    return stall(USBIO_UNSUPPORTED);
  }
  if (_stream_running) {
    return stall(USBIO_BUSY);
  }
  if (_stream_n_channels == 0) {
    return stall(USBIO_BAD_VALUE);
  }
  if (wValue != 0 && wValue < USBIO_STREAM_MIN_PERIOD_US) {
    return stall(USBIO_BAD_VALUE);
  }
  const uint16_t known_flags =
      (uint16_t)(USBIO_STREAM_FLAG_DIGITAL | USBIO_STREAM_FLAG_STOP_ON_OVERRUN);
  if (wIndex & ~known_flags) {
    return stall(USBIO_BAD_VALUE);
  }
  _stream_period_us = wValue;
  _stream_flags = (uint8_t)wIndex;
  _stream_running = 1;
  /* The actual seq/overruns/ring reset happens in stream_poll(): touching
   * ring indices here could race a poll() call mid-drain (same reasoning as
   * _reset_requests/_reset_done for RESET). */
  ++_stream_start_requests;
  return true;
}

bool UsbIoDevice::handle_stream_stop() {
  if (!(_flags & USBIO_FLAG_STREAMING)) {
    return stall(USBIO_UNSUPPORTED);
  }
  _stream_running = 0; /* selection and any already-queued records are kept */
  return true;
}

void UsbIoDevice::stream_poll() {
  if (_stream_start_done != _stream_start_requests) {
    _stream_start_done = _stream_start_requests;
    _stream_head = 0;
    _stream_tail = 0;
    _stream_seq = 0;
    _stream_overruns = 0;
    _stream_tx_failures = 0;
    _stream_deadline_us = micros();
  }
  if (_stream_running) {
    const uint32_t now = micros();
    const bool due =
        _stream_period_us == 0 || (int32_t)(now - _stream_deadline_us) >= 0;
    if (due) {
      stream_sample(now);
      if (_stream_period_us != 0) {
        _stream_deadline_us += _stream_period_us;
      }
    }
  }
  /* Drain whatever the ring holds - not just what was just sampled - so a
   * backlog left over from a busy transport keeps draining even on a poll()
   * call that samples nothing (period not yet due, or the stream stopped). */
  while (_stream_tail != _stream_head) {
    const StreamRecord &rec =
        _stream_ring[_stream_tail & (uint8_t)(StreamRingDepth - 1u)];
    const uint8_t result = usbio_transport_stream_write(rec.data, rec.len);
    if (result == USBIO_STREAM_WRITE_BUSY) {
      break; /* previous packet still in flight; retry next poll() */
    }
    if (result == USBIO_STREAM_WRITE_FAILED) {
      /* The host is not draining the endpoint. Give it StreamTxFailureLimit
       * tries, then stop the stream and drop the backlog: leaving records
       * queued would keep the drain loop above paying the transport's send
       * timeout on every poll() for as long as the sketch runs. The dropped
       * records are counted as overruns, and GET_STREAM_STATUS then reports
       * running == 0, so a host that comes back sees exactly what happened. */
      if (++_stream_tx_failures < StreamTxFailureLimit) {
        break;
      }
      _stream_running = 0;
      _stream_overruns += (uint8_t)(_stream_head - _stream_tail);
      _stream_tail = _stream_head;
      _stream_tx_failures = 0;
      break;
    }
    _stream_tx_failures = 0;
    _stream_tail = (uint8_t)(_stream_tail + 1u);
  }
}

void UsbIoDevice::stream_sample(uint32_t now) {
  uint8_t buf[StreamRecordMaxLen];
  usbio_stream_header_t hdr;
  hdr.magic = USBIO_STREAM_MAGIC;
  hdr.n_samples = _stream_n_channels;
  hdr.seq = _stream_seq;
  hdr.t_us = now;
  memcpy(buf, &hdr, sizeof(hdr));
  uint16_t off = sizeof(hdr);
  for (uint8_t i = 0; i < _stream_n_channels; ++i) {
    const uint8_t pin = _stream_channels[i];
    const uint16_t v = _mode[pin] == USBIO_MODE_ANALOG_IN
                            ? read_analog(pin)
                            : (uint16_t)read_pin(pin);
    put_u16(&buf[off], v);
    off = (uint16_t)(off + 2u);
  }
  if (_stream_flags & USBIO_STREAM_FLAG_DIGITAL) {
    /* Same bitmap as DIO_READ_ALL: the poll()-refreshed _dio shadow, not a
     * fresh digitalRead() per pin. */
    const uint16_t nbytes = (uint16_t)((_n_pins + 7u) / 8u);
    memset(&buf[off], 0, nbytes);
    for (uint8_t p = 0; p < _n_pins; ++p) {
      if (mode_is_digital(_mode[p]) && _dio[p]) {
        buf[off + (p >> 3)] |= (uint8_t)(1u << (p & 7u));
      }
    }
    off = (uint16_t)(off + nbytes);
    if (off & 1u) {
      buf[off] = 0; /* pad to an even record length */
      ++off;
    }
  }
  ++_stream_seq; /* bumped even when the record below is dropped: the host
                  * sees the drop as a gap between the seq values it did get */
  if ((uint8_t)(_stream_head - _stream_tail) >= StreamRingDepth) {
    ++_stream_overruns;
    if (_stream_flags & USBIO_STREAM_FLAG_STOP_ON_OVERRUN) {
      _stream_running = 0;
    }
    return;
  }
  StreamRecord &slot = _stream_ring[_stream_head & (uint8_t)(StreamRingDepth - 1u)];
  slot.len = (uint8_t)off;
  memcpy(slot.data, buf, off);
  _stream_head = (uint8_t)(_stream_head + 1u);
}

#endif /* USBIO_HAS_STREAM_TRANSPORT */
