/* arduino_driver_c.h - C ABI wrapper around the ArduinoDriver C++ host
 * library (extras/driver), for language bindings (Python, R, ...) that
 * cannot link the C++ API directly.
 *
 * Conventions:
 *   - Every entry point that can fail returns adrv_status_t; ADRV_OK (0) is
 *     success. On any other value, adrv_last_error_message() (thread-local)
 *     describes what went wrong until the next failing call on this thread.
 *   - Opaque handles (adrv_context_t, adrv_device_t, adrv_stream_t) are
 *     created by one function and released by exactly one matching
 *     "destroy"/"close" function; passing an already-released handle is
 *     undefined behaviour, same as any other manual-lifetime C API.
 *   - None of these functions are safe to call on the same handle from two
 *     threads at once, except where documented otherwise (mirrors the C++
 *     Device threading contract in arduino_driver/Device.h).
 *   - v1 exposes only the polling I/O and streaming surface; pin events and
 *     callback-based streaming (Stream::on_records, EventWatcher) are not
 *     part of this API yet.
 */
#ifndef ARDUINO_DRIVER_C_H
#define ARDUINO_DRIVER_C_H

#include "adrv_export.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Opaque handles -------------------------------------------------- */

typedef struct adrv_context adrv_context_t;
typedef struct adrv_device adrv_device_t;
typedef struct adrv_stream adrv_stream_t;

/* ---- Errors ------------------------------------------------------------ */

typedef enum {
  ADRV_OK = 0,
  ADRV_ERR_USB,           /* libusb transfer failure */
  ADRV_ERR_STALL,         /* device rejected the request (STALL) */
  ADRV_ERR_TIMEOUT,       /* control transfer did not complete in time */
  ADRV_ERR_PROTOCOL,      /* device answered something the protocol forbids */
  ADRV_ERR_DEVICE_BUSY,   /* a Stream is running, or BUSY retries ran out */
  ADRV_ERR_INVALID_PIN,   /* pin index out of range */
  ADRV_ERR_INVALID_MODE,  /* pin not in the mode the call needs */
  ADRV_ERR_INVALID_VALUE, /* value outside the accepted range */
  ADRV_ERR_NOT_SUPPORTED, /* pin or board lacks the needed capability */
  ADRV_ERR_QUEUE_FULL,    /* firmware command queue is full */
  ADRV_ERR_NOT_READY,     /* device enumerated but the sketch has not begun */
  ADRV_ERR_DEVICE_NOT_FOUND, /* no matching USB device */
  ADRV_ERR_INVALID_ARGUMENT, /* bad argument to this C API (e.g. null out-param)
                              */
  ADRV_ERR_OTHER,            /* any other failure */
} adrv_status_t;

/* Message for the most recent non-ADRV_OK return on the calling thread.
 * Never null; empty string when nothing has failed yet on this thread. The
 * pointer is valid until the next ArduinoDriver call on the same thread. */
ADRV_API const char *adrv_last_error_message(void);

/* ---- Context (one per process, or one per independent enumeration) ---- */

/* Creates a libusb context. Returns NULL on failure (see
 * adrv_last_error_message()). */
ADRV_API adrv_context_t *adrv_context_create(void);
ADRV_API void adrv_context_destroy(adrv_context_t *ctx);

/* ---- Device: open / close ---------------------------------------------- */

/* Opens the first identified UsbIo device. *out is set only on ADRV_OK. */
ADRV_API adrv_status_t adrv_device_open_first(adrv_context_t *ctx,
                                              adrv_device_t **out);
/* Opens the identified device with this USB serial number. */
ADRV_API adrv_status_t adrv_device_open_by_serial(adrv_context_t *ctx,
                                                  const char *serial,
                                                  adrv_device_t **out);
/* Closes a device opened by either function above. No-op on NULL. */
ADRV_API void adrv_device_close(adrv_device_t *dev);

/* ---- Static information (no USB traffic) ------------------------------- */

typedef struct {
  uint16_t protocol_version;
  uint16_t board_id;
  uint8_t n_pins;
  uint8_t n_ain;
  uint8_t adc_bits;
  uint8_t pwm_bits;
  uint8_t dac_bits;
  uint8_t queue_depth;
  uint16_t vref_mv;
  uint16_t io_mv;
  uint16_t flags;
  uint8_t stream_max_channels;
  uint8_t event_max_pins;
  uint16_t stream_min_period_us;
} adrv_info_t;

/* USBIO_FLAG_* bits, for adrv_info_t.flags. */
#define ADRV_FLAG_VENDOR_INTERFACE (1u << 0)
#define ADRV_FLAG_PULLDOWN (1u << 1)
#define ADRV_FLAG_STREAMING (1u << 2)
#define ADRV_FLAG_EVENTS (1u << 3)

ADRV_API adrv_status_t adrv_device_get_info(adrv_device_t *dev,
                                            adrv_info_t *out);
/* Number of addressable pins; 0 on a null device. Never fails. */
ADRV_API size_t adrv_pin_count(adrv_device_t *dev);
/* Human-readable board name, e.g. "Portenta H7"; never null. */
ADRV_API const char *adrv_board_name(uint16_t board_id);

/* ---- Configuration ------------------------------------------------------ */

typedef enum {
  ADRV_PIN_INPUT = 0,
  ADRV_PIN_OUTPUT,
  ADRV_PIN_INPUT_PULLUP,
  ADRV_PIN_INPUT_PULLDOWN,
  ADRV_PIN_ANALOG_IN,
  ADRV_PIN_PWM,
  ADRV_PIN_DAC,
} adrv_pin_mode_t;

ADRV_API adrv_status_t adrv_pin_mode(adrv_device_t *dev, uint8_t pin,
                                     adrv_pin_mode_t mode);

/* ---- Digital I/O --------------------------------------------------------- */

ADRV_API adrv_status_t adrv_digital_write(adrv_device_t *dev, uint8_t pin,
                                          bool high);
ADRV_API adrv_status_t adrv_digital_read(adrv_device_t *dev, uint8_t pin,
                                         bool *out);
/* Writes at most out_len entries (one per pin, pin i at index i) and sets
 * *n_written to the device's pin count, regardless of out_len (so callers
 * can size a first, too-small buffer correctly on the next call). */
ADRV_API adrv_status_t adrv_read_all_digital(adrv_device_t *dev, bool *out,
                                             size_t out_len, size_t *n_written);

/* ---- Analog input ---------------------------------------------------------
 */

ADRV_API adrv_status_t adrv_analog_read(adrv_device_t *dev, uint8_t pin,
                                        uint16_t *out);
ADRV_API adrv_status_t adrv_analog_read_volts(adrv_device_t *dev, uint8_t pin,
                                              double *out);
/* One raw sample per analog pin, in ascending pin order; see
 * adrv_read_all_digital() for the out/out_len/n_written convention. */
ADRV_API adrv_status_t adrv_read_all_analog(adrv_device_t *dev, uint16_t *out,
                                            size_t out_len, size_t *n_written);
ADRV_API adrv_status_t adrv_to_volts(adrv_device_t *dev, uint16_t raw,
                                     double *out);

/* ---- PWM / DAC output ------------------------------------------------------
 */

ADRV_API adrv_status_t adrv_pwm_write(adrv_device_t *dev, uint8_t pin,
                                      uint16_t duty);
ADRV_API adrv_status_t adrv_pwm_write_fraction(adrv_device_t *dev, uint8_t pin,
                                               double fraction);
ADRV_API adrv_status_t adrv_dac_write(adrv_device_t *dev, uint8_t pin,
                                      uint16_t value);
ADRV_API adrv_status_t adrv_dac_write_volts(adrv_device_t *dev, uint8_t pin,
                                            double volts);

/* ---- Control ----------------------------------------------------------------
 */

ADRV_API adrv_status_t adrv_device_status(adrv_device_t *dev,
                                          uint8_t *out_last_error,
                                          uint8_t *out_queue_pending);
ADRV_API adrv_status_t adrv_sync(adrv_device_t *dev);
ADRV_API adrv_status_t adrv_reset(adrv_device_t *dev);

/* ---- Streaming (polling only; see Stream.h's threading contract) -----------
 */

typedef struct {
  const uint8_t *pins; /* sampling order; copied, not retained */
  size_t n_pins;
  uint32_t period_us;    /* 0 = free running */
  uint8_t flags;         /* ADRV_STREAM_FLAG_* bits */
  size_t queue_capacity; /* 0 -> library default */
} adrv_stream_config_t;

#define ADRV_STREAM_FLAG_DIGITAL (1u << 0)
#define ADRV_STREAM_FLAG_STOP_ON_OVERRUN (1u << 1)

typedef struct {
  uint8_t pin;
  uint16_t raw;
  double volts;
  uint32_t t_us;
} adrv_sample_t;

typedef struct {
  uint32_t device_overruns;
  uint64_t records_received;
  uint64_t seq_gaps;
  uint64_t host_drops;
  uint64_t resyncs;
  uint64_t stale_records;
} adrv_stream_stats_t;

/* Starts continuous sampling; *out is set only on ADRV_OK. While the
 * returned stream is running, every adrv_* call above that touches the
 * transport (other than adrv_device_status()'s status-only path is NOT
 * exempt either) returns ADRV_ERR_DEVICE_BUSY, mirroring Device's C++
 * threading contract. */
ADRV_API adrv_status_t adrv_start_stream(adrv_device_t *dev,
                                         const adrv_stream_config_t *config,
                                         adrv_stream_t **out);
/* Blocks until at least one sample is available or timeout_ms elapses;
 * *n_read is the number of samples copied (0 on timeout). Samples from one
 * record are always copied contiguously and in full. */
ADRV_API adrv_status_t adrv_stream_read(adrv_stream_t *stream,
                                        adrv_sample_t *out, size_t out_len,
                                        uint32_t timeout_ms, size_t *n_read);
ADRV_API adrv_status_t adrv_stream_stats(adrv_stream_t *stream,
                                         adrv_stream_stats_t *out);
/* True until stop() completes or the worker stops on its own. */
ADRV_API bool adrv_stream_running(adrv_stream_t *stream);
/* Why the worker stopped on its own; empty string while running normally or
 * after a regular stop(). Never null. */
ADRV_API const char *adrv_stream_error(adrv_stream_t *stream);
/* Stops the device stream and joins the worker thread. Idempotent. */
ADRV_API void adrv_stream_stop(adrv_stream_t *stream);
/* Stops (if still running) and releases the stream. No-op on NULL. */
ADRV_API void adrv_stream_destroy(adrv_stream_t *stream);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ARDUINO_DRIVER_C_H */
