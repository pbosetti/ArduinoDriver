/*
 * usbio_protocol.h - wire protocol shared by the firmware (arduino/UsbIo) and
 * the host driver (driver/). This file is the single source of truth for the
 * protocol: it is C-compatible, freestanding (only <stdint.h>) and compiles as
 * C17 and C++20 on every target (Arduino cores, clang, gcc, MSVC).
 *
 * Transport
 * ---------
 * USB control transfers on endpoint 0 with type = vendor and recipient =
 * device. All multi-byte fields are little-endian (USB byte order; every
 * supported MCU is little-endian, so the structs below travel verbatim).
 *
 *   OUT (host -> device): bmRequestType = USBIO_REQTYPE_OUT, bRequest = command,
 *     wIndex = pin, wValue = argument, wLength = 0 (no data stage).
 *     The device validates the request in interrupt context and either ACKs it
 *     (the command is queued and executed from loop()) or STALLs it (invalid
 *     pin / mode / value; the reason is readable through GET_STATUS.last_error).
 *
 *   IN (device -> host): bmRequestType = USBIO_REQTYPE_IN, bRequest = command,
 *     wIndex = pin (or first pin), wLength = reply size. Replies are served from
 *     a shadow copy of the pin state that loop() keeps fresh. If queued commands
 *     are still pending, data-carrying replies report USBIO_BUSY and the host
 *     retries. IN requests never STALL for protocol reasons: errors travel in
 *     the status byte. Only unknown bRequest values STALL.
 *
 *   Interface-recipient form (optional): on boards that expose the dedicated
 *     vendor interface the same requests are also accepted with
 *     bmRequestType = USBIO_REQTYPE_OUT_ITF / USBIO_REQTYPE_IN_ITF and
 *     wIndex = (pin << 8) | interface_number, the USB convention for
 *     interface-recipient requests. Boards without the interface (Renesas)
 *     STALL that form. Hosts use the device-recipient form unless an OS
 *     requires routing through the interface (e.g. WinUSB on Windows).
 *
 *   Streaming endpoint (optional): boards whose core lets the vendor interface
 *     own endpoints (mbed, SAMD) add one bulk IN endpoint to it and advertise
 *     USBIO_FLAG_STREAMING in GET_INFO. Control transfers stay the command
 *     channel; the bulk endpoint carries sample records only (see "Streaming"
 *     below). Boards without it (Renesas) leave the flag clear and STALL the
 *     STREAM_* requests with USBIO_UNSUPPORTED.
 *
 * Pin numbering is the Arduino pin number of the board (A0 is e.g. 14 on the
 * UNO R4 and 15 on the Portenta H7); GET_PIN_CAPS tells what each pin can do.
 *
 * Pin state model
 * ---------------
 * After boot every pin is "unconfigured": the firmware does not touch a pin
 * until the host sends PIN_MODE for it (or RESET), so a sketch's own use of
 * Serial1 / SPI / LEDs keeps working until the host takes over. DIO_READ and
 * AI_READ on an unconfigured pin report USBIO_BAD_MODE; it reads 0 in the
 * *_READ_ALL replies. RESET puts every DIO-capable pin in INPUT and returns
 * pins without USBIO_CAP_DIO (analog-only pads) to "unconfigured".
 *
 * Mode bookkeeping: the device records the *intended* mode of a pin at the
 * moment PIN_MODE is accepted (not when it is executed), so a PIN_MODE followed
 * immediately by DIO_WRITE on the same pin validates correctly.
 *
 * Not ready: the USB stack may enumerate before the sketch reaches begin().
 * Until then GET_INFO reports n_pins == 0 and every pin request fails; a host
 * that sees n_pins == 0 should wait briefly and read GET_INFO again.
 *
 * Streaming (USBIO_FLAG_STREAMING)
 * --------------------------------
 * The host builds a channel set one pin at a time with STREAM_SELECT (so no
 * request needs an OUT data stage), then STREAM_START begins periodic sampling
 * of that set. Each period the device samples every selected pin, formats one
 * record (usbio_stream_header_t + samples) and queues it for the bulk IN
 * endpoint; STREAM_STOP ends the stream and keeps the selection.
 *
 * Sampling runs from loop(), not from an interrupt, so the achieved period is
 * best-effort: t_us in each record - the device's own micros() at sample time -
 * is the timing reference, not the host's arrival time. When the device's ring
 * fills, the newest record is dropped and overruns is incremented, which the
 * host also sees as a gap in seq (unless STOP_ON_OVERRUN was requested).
 *
 * The endpoint carries a continuous byte stream of records: the device writes
 * whole packets, a record may straddle packet boundaries, and the host
 * reassembles. Every record starts with USBIO_STREAM_MAGIC so a host that lost
 * sync can resynchronise, and record lengths are always even.
 */
#ifndef USBIO_PROTOCOL_H
#define USBIO_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__cplusplus)
#define USBIO_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#define USBIO_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#else
#define USBIO_STATIC_ASSERT(cond, msg)
#endif

/* ---- Identity ---------------------------------------------------------- */

#define USBIO_PROTOCOL_VERSION 0x0001u /* bumped on incompatible changes */
#define USBIO_MAGIC "UIO1"             /* 4 bytes, not NUL terminated */
#define USBIO_MAGIC_LEN 4

/* Vendor IDs the host probes with GET_INFO when no vendor interface is found
 * in the configuration descriptor (Renesas boards cannot add one). */
#define USBIO_VID_ARDUINO 0x2341u
#define USBIO_VID_RASPBERRY_PI 0x2E8Au
#define USBIO_VID_ESPRESSIF 0x303Au

/* Dedicated vendor interface, added where the core allows it (SAMD, mbed,
 * RP2040, ESP32). Zero endpoints; it exists so the host can identify the
 * device from the descriptor and so Windows has a function to bind WinUSB to. */
#define USBIO_ITF_CLASS 0xFFu
#define USBIO_ITF_SUBCLASS 0x49u /* 'I' */
#define USBIO_ITF_PROTOCOL 0x4Fu /* 'O' */
#define USBIO_ITF_STRING "UsbIo"

/* ---- Request encoding -------------------------------------------------- */

#define USBIO_REQTYPE_OUT 0x40u     /* host->device, vendor, device recipient    */
#define USBIO_REQTYPE_IN 0xC0u      /* device->host, vendor, device recipient    */
#define USBIO_REQTYPE_OUT_ITF 0x41u /* host->device, vendor, interface recipient */
#define USBIO_REQTYPE_IN_ITF 0xC1u  /* device->host, vendor, interface recipient */

enum usbio_request {
  USBIO_REQ_GET_INFO = 0x00,     /* IN  -> usbio_info_t                         */
  USBIO_REQ_GET_PIN_CAPS = 0x01, /* IN  wIndex=first pin -> uint8_t caps[]      */
  USBIO_REQ_PIN_MODE = 0x02,     /* OUT wIndex=pin wValue=usbio_pin_mode        */
  USBIO_REQ_DIO_READ = 0x03,     /* IN  wIndex=pin -> usbio_dio_reply_t         */
  USBIO_REQ_DIO_WRITE = 0x04,    /* OUT wIndex=pin wValue=0 (LOW) / !=0 (HIGH)  */
  USBIO_REQ_AI_READ = 0x05,      /* IN  wIndex=pin -> usbio_ai_reply_t          */
  USBIO_REQ_PWM_WRITE = 0x06,    /* OUT wIndex=pin wValue=duty 0..2^pwm_bits-1  */
  USBIO_REQ_DAC_WRITE = 0x07,    /* OUT wIndex=pin wValue=value 0..2^dac_bits-1 */
  USBIO_REQ_DIO_READ_ALL = 0x10, /* IN  -> usbio_all_header_t + bitmap          */
  USBIO_REQ_AI_READ_ALL = 0x11,  /* IN  -> usbio_all_header_t + uint16_t[n_ain] */
  USBIO_REQ_GET_STATUS = 0x20,   /* IN  -> usbio_status_reply_t                 */
  /* Streaming (only when USBIO_FLAG_STREAMING is set; else STALL UNSUPPORTED) */
  USBIO_REQ_STREAM_SELECT = 0x30, /* OUT wIndex=pin wValue=0 remove / 1 add     */
  USBIO_REQ_STREAM_START = 0x31,  /* OUT wIndex=usbio_stream_flags
                                   *     wValue=period_us (0 = free running)    */
  USBIO_REQ_STREAM_STOP = 0x32,   /* OUT stop sampling, keep the selection      */
  USBIO_REQ_STREAM_STATUS = 0x33, /* IN  -> usbio_stream_status_t               */
  USBIO_REQ_RESET = 0x7F          /* OUT all pins -> INPUT, queue cleared       */
};

/* Status byte of IN replies and GET_STATUS.last_error (reason of last STALL). */
enum usbio_status {
  USBIO_OK = 0,
  USBIO_BUSY = 1,        /* queued commands pending; retry the read          */
  USBIO_BAD_PIN = 2,     /* pin >= n_pins                                     */
  USBIO_BAD_MODE = 3,    /* request not valid for the pin's intended mode     */
  USBIO_BAD_CMD = 4,     /* unknown bRequest, or OUT request with wLength != 0 */
  USBIO_UNSUPPORTED = 5, /* pin lacks the capability, or mode not on board    */
  USBIO_QUEUE_FULL = 6,  /* command queue full; GET_STATUS/sync then retry    */
  USBIO_BAD_VALUE = 7    /* wValue out of range for the request               */
};

/* wValue of PIN_MODE. Validation: INPUT/OUTPUT/INPUT_PULLUP need CAP_DIO,
 * INPUT_PULLDOWN needs CAP_DIO + USBIO_FLAG_PULLDOWN, ANALOG_IN needs CAP_AIN,
 * PWM needs CAP_PWM, DAC needs CAP_DAC. Otherwise STALL (UNSUPPORTED). A mode
 * value >= USBIO_MODE_COUNT STALLs with BAD_VALUE. */
enum usbio_pin_mode {
  USBIO_MODE_INPUT = 0,
  USBIO_MODE_OUTPUT = 1,
  USBIO_MODE_INPUT_PULLUP = 2,
  USBIO_MODE_INPUT_PULLDOWN = 3,
  USBIO_MODE_ANALOG_IN = 4, /* enables background sampling of the pin */
  USBIO_MODE_PWM = 5,
  USBIO_MODE_DAC = 6,
  USBIO_MODE_COUNT = 7
};

/* Bits of the per-pin capability byte returned by GET_PIN_CAPS. */
enum usbio_pin_caps {
  USBIO_CAP_DIO = 1u << 0,
  USBIO_CAP_AIN = 1u << 1,
  USBIO_CAP_PWM = 1u << 2,
  USBIO_CAP_DAC = 1u << 3
};

enum usbio_info_flags {
  USBIO_FLAG_VENDOR_INTERFACE = 1u << 0, /* device exposes the vendor interface */
  USBIO_FLAG_PULLDOWN = 1u << 1,         /* INPUT_PULLDOWN is supported         */
  USBIO_FLAG_STREAMING = 1u << 2         /* bulk IN endpoint + STREAM_* requests;
                                          * stream_max_channels is then valid   */
};

/* wIndex of STREAM_START. */
enum usbio_stream_flags {
  USBIO_STREAM_FLAG_DIGITAL = 1u << 0,        /* append the digital bitmap to
                                               * every record                   */
  USBIO_STREAM_FLAG_STOP_ON_OVERRUN = 1u << 1 /* stop the stream instead of
                                               * dropping records               */
};

/* High byte = architecture family, low byte = board. */
enum usbio_board_id {
  USBIO_BOARD_UNKNOWN = 0x0000,
  /* Renesas RA (TinyUSB) */
  USBIO_BOARD_UNO_R4_MINIMA = 0x0101,
  USBIO_BOARD_NANO_R4 = 0x0102,
  USBIO_BOARD_RENESAS_GENERIC = 0x01FF,
  /* mbed */
  USBIO_BOARD_PORTENTA_H7 = 0x0201,
  USBIO_BOARD_GIGA_R1 = 0x0202,
  USBIO_BOARD_NANO_33_BLE = 0x0203,
  USBIO_BOARD_MBED_GENERIC = 0x02FF,
  /* SAMD21 */
  USBIO_BOARD_ZERO = 0x0301,
  USBIO_BOARD_MKR = 0x0302,
  USBIO_BOARD_NANO_33_IOT = 0x0303,
  USBIO_BOARD_SAMD_GENERIC = 0x03FF,
  /* RP2040 / RP2350 (unverified transport) */
  USBIO_BOARD_NANO_RP2040_CONNECT = 0x0401,
  USBIO_BOARD_RP2040_GENERIC = 0x04FF,
  /* ESP32-S2 / S3 (unverified transport) */
  USBIO_BOARD_NANO_ESP32 = 0x0501,
  USBIO_BOARD_ESP32_GENERIC = 0x05FF
};

/* ---- Limits ------------------------------------------------------------ */

#define USBIO_MAX_PINS 128u      /* bitmap = 16 bytes, caps reply = 128 bytes */
#define USBIO_MAX_AIN 32u        /* AI_READ_ALL payload = 64 bytes            */
#define USBIO_QUEUE_DEPTH 32u    /* firmware command queue capacity           */
#define USBIO_MAX_REPLY_LEN 128u /* largest IN reply (GET_PIN_CAPS)           */

/* Streaming */
#define USBIO_STREAM_MAGIC 0x5355u   /* 'U','S' little-endian; starts a record */
#define USBIO_STREAM_EP_SIZE 64u     /* bulk IN max packet size (full speed);
                                      * hosts use the descriptor's value       */
#define USBIO_MAX_STREAM_CHANNELS 8u /* upper bound on stream_max_channels     */
#define USBIO_STREAM_MIN_PERIOD_US 100u /* fastest period a device must accept;
                                         * 0 means free running                */

/* ---- Reply layouts (naturally aligned, no packing pragmas needed) ------ */

typedef struct usbio_info {
  uint8_t magic[USBIO_MAGIC_LEN]; /* USBIO_MAGIC                              */
  uint16_t protocol_version;      /* USBIO_PROTOCOL_VERSION                   */
  uint16_t board_id;              /* enum usbio_board_id                      */
  uint8_t n_pins;                 /* pins are addressed 0 .. n_pins-1; 0 means
                                   * the sketch has not called begin() yet    */
  uint8_t n_ain;                  /* exact count of pins carrying CAP_AIN in
                                   * GET_PIN_CAPS (defines the AI_READ_ALL
                                   * layout)                                  */
  uint8_t adc_bits;               /* analogRead() resolution in use           */
  uint8_t pwm_bits;               /* PWM duty resolution in use               */
  uint8_t dac_bits;               /* 0 when the board has no DAC              */
  uint8_t queue_depth;            /* USBIO_QUEUE_DEPTH                        */
  uint16_t vref_mv;               /* ADC full-scale voltage, millivolts; also
                                   * the DAC full-scale on every board        */
  uint16_t io_mv;                 /* digital logic level, millivolts          */
  uint16_t flags;                 /* enum usbio_info_flags                    */
  uint8_t stream_max_channels;    /* pins STREAM_SELECT accepts at once; 0 when
                                   * USBIO_FLAG_STREAMING is clear            */
  uint8_t reserved[3];            /* zero                                     */
} usbio_info_t;
USBIO_STATIC_ASSERT(sizeof(usbio_info_t) == 24, "usbio_info_t must be 24 bytes");

/* GET_PIN_CAPS: wIndex = first pin; reply = min(wLength, n_pins - first) bytes,
 * one capability byte per pin. wIndex >= n_pins -> zero-length reply. */

typedef struct usbio_dio_reply {
  uint8_t status; /* OK: value valid. BAD_PIN, BAD_MODE (unconfigured, ANALOG_IN,
                   * PWM or DAC pin), BUSY                                   */
  uint8_t value;  /* 0 / 1. OUTPUT pins report the last value written        */
} usbio_dio_reply_t;
USBIO_STATIC_ASSERT(sizeof(usbio_dio_reply_t) == 2, "usbio_dio_reply_t must be 2 bytes");

typedef struct usbio_ai_reply {
  uint8_t status;   /* OK, BAD_PIN, BAD_MODE (pin not in ANALOG_IN), BUSY     */
  uint8_t reserved; /* zero                                                   */
  uint16_t raw;     /* 0 .. 2^adc_bits-1                                      */
} usbio_ai_reply_t;
USBIO_STATIC_ASSERT(sizeof(usbio_ai_reply_t) == 4, "usbio_ai_reply_t must be 4 bytes");

/* Header of DIO_READ_ALL and AI_READ_ALL replies. Payload follows:
 *   DIO_READ_ALL: uint8_t bitmap[(n_pins + 7) / 8]; bit (i % 8) of byte (i / 8)
 *                 is pin i. Pins that are not in an INPUT* or OUTPUT mode read 0.
 *   AI_READ_ALL:  uint16_t raw[n_ain] in ascending pin order over the pins that
 *                 carry CAP_AIN. Pins not in ANALOG_IN mode read 0. Samples may
 *                 have been taken at different times (round-robin sampling). */
typedef struct usbio_all_header {
  uint8_t status;   /* OK or BUSY */
  uint8_t reserved; /* zero */
} usbio_all_header_t;
USBIO_STATIC_ASSERT(sizeof(usbio_all_header_t) == 2, "usbio_all_header_t must be 2 bytes");

typedef struct usbio_status_reply {
  uint8_t status;        /* always OK                                        */
  uint8_t queue_pending; /* commands accepted but not yet executed           */
  uint8_t last_error;    /* enum usbio_status: reason of the last STALLed
                          * request (OUT or IN); cleared to OK by this read  */
  uint8_t reserved;      /* zero                                             */
} usbio_status_reply_t;
USBIO_STATIC_ASSERT(sizeof(usbio_status_reply_t) == 4, "usbio_status_reply_t must be 4 bytes");

typedef struct usbio_stream_status {
  uint8_t status;      /* always OK on a streaming build                      */
  uint8_t running;     /* 0 stopped, 1 sampling                               */
  uint8_t n_channels;  /* pins currently selected                             */
  uint8_t flags;       /* enum usbio_stream_flags in force since START        */
  uint16_t period_us;  /* period requested by START (0 = free running)        */
  uint16_t reserved;   /* zero                                                */
  uint32_t seq;        /* seq of the most recent record produced              */
  uint32_t overruns;   /* records dropped since START (ring full)             */
} usbio_stream_status_t;
USBIO_STATIC_ASSERT(sizeof(usbio_stream_status_t) == 16, "usbio_stream_status_t must be 16 bytes");

/* Header of every record on the bulk IN endpoint. The payload follows:
 *   uint16_t samples[n_samples]  raw readings, in STREAM_SELECT order; a DIO
 *                                pin reads 0 or 1, an ANALOG_IN pin 0..2^adc_bits-1
 *   uint8_t  bitmap[]            only with USBIO_STREAM_FLAG_DIGITAL: the
 *                                DIO_READ_ALL bitmap, (n_pins + 7) / 8 bytes
 *                                padded with a zero byte to an even length
 * so record length = 12 + 2 * n_samples [+ padded bitmap], always even. */
typedef struct usbio_stream_header {
  uint16_t magic;     /* USBIO_STREAM_MAGIC                                   */
  uint16_t n_samples; /* channels in this record (== n_channels at START)     */
  uint32_t seq;       /* record counter from START, wraps; gaps == drops      */
  uint32_t t_us;      /* device micros() when the record was sampled          */
} usbio_stream_header_t;
USBIO_STATIC_ASSERT(sizeof(usbio_stream_header_t) == 12, "usbio_stream_header_t must be 12 bytes");

/* ---- OUT request validation (device side, interrupt context) ----------- *
 *
 * Checks run in this order and the first failure is the recorded reason:
 * wLength, bRequest, pin, then the request-specific mode/value checks, and
 * queue capacity last.
 *
 *  any        wLength != 0 -> STALL, BAD_CMD (the protocol has no OUT data).
 *             unknown bRequest -> STALL, BAD_CMD.
 *  PIN_MODE   pin < n_pins (else BAD_PIN), mode < USBIO_MODE_COUNT (else
 *             BAD_VALUE), capability as above (else UNSUPPORTED).
 *  DIO_WRITE  intended mode == OUTPUT.                (else STALL, BAD_MODE)
 *  PWM_WRITE  intended mode == PWM, wValue <= 2^pwm_bits-1 (else BAD_VALUE).
 *  DAC_WRITE  intended mode == DAC, wValue <= 2^dac_bits-1 (else BAD_VALUE).
 *  RESET      always accepted; clears the queue, sets every DIO-capable pin's
 *             intended mode to INPUT (others to unconfigured) and executes
 *             pinMode(INPUT) on the DIO-capable pins from loop(). Also stops a
 *             running stream and clears its selection.
 *  any        queue full -> STALL, QUEUE_FULL.
 *
 * Streaming requests (all STALL UNSUPPORTED when USBIO_FLAG_STREAMING is clear):
 *
 *  SELECT     stream stopped (else BUSY), pin < n_pins (else BAD_PIN), intended
 *             mode is ANALOG_IN or INPUT* (else BAD_MODE), wValue <= 1 (else
 *             BAD_VALUE); adding beyond stream_max_channels -> BAD_VALUE.
 *             Adding a selected pin or removing an unselected one is a no-op.
 *  START      stream stopped (else BUSY), n_channels > 0 (else BAD_VALUE),
 *             wValue == 0 or >= USBIO_STREAM_MIN_PERIOD_US (else BAD_VALUE),
 *             wIndex has no unknown flag bit (else BAD_VALUE). Resets seq,
 *             overruns and the ring, then starts sampling from loop().
 *  STOP       always accepted; stops sampling and keeps the selection. Records
 *             already queued still reach the host.
 *  STATUS     never STALLs on a streaming build; it is the way to read the
 *             device-side drop count and confirm the achieved state.
 *
 * A stream keeps running while control requests are served; changing the mode
 * of a selected pin (PIN_MODE) or RESET stops it. Unplugging or a USB suspend
 * stops it too: the selection and counters do not survive a re-enumeration.
 */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* USBIO_PROTOCOL_H */
