# device.R - Device R6 class and its constructors (see arduino_driver_c.h's
# adrv_device_t entry points).

#' Pin modes (mirrors adrv_pin_mode_t)
#' @export
PinMode <- list(
  INPUT = 0L,
  OUTPUT = 1L,
  INPUT_PULLUP = 2L,
  INPUT_PULLDOWN = 3L,
  ANALOG_IN = 4L,
  PWM = 5L,
  DAC = 6L
)

.FLAG_STREAMING <- 4L # ADRV_FLAG_STREAMING (1 << 2)
.FLAG_EVENTS <- 8L # ADRV_FLAG_EVENTS (1 << 3)
.STREAM_FLAG_DIGITAL <- 1L # ADRV_STREAM_FLAG_DIGITAL (1 << 0)

#' One open UsbIo device
#'
#' Build one with \code{\link{device_open_first}} or
#' \code{\link{device_open_by_serial}}; call \code{close()} when done (a
#' finalizer also closes it when the object is garbage-collected).
#' @export
Device <- R6::R6Class("Device",
  public = list(
    #' @description Not a public constructor: use device_open_first() /
    #' device_open_by_serial().
    initialize = function(context, handle) {
      private$context_ <- context
      private$handle_ <- handle
      reg.finalizer(self, function(e) e$close(), onexit = TRUE)
    },

    #' @description Closes the device. Idempotent.
    close = function() {
      if (!is.null(private$handle_)) {
        .Call("adrv_r_device_close", private$handle_)
        private$handle_ <- NULL
      }
      if (!is.null(private$context_)) {
        .Call("adrv_r_context_destroy", private$context_)
        private$context_ <- NULL
      }
      invisible(self)
    },

    #' @description Static information: a named list with board_name,
    #' streaming, events, vref_volts and io_volts added to the raw
    #' adrv_info_t fields.
    info = function() {
      result <- .Call("adrv_r_device_get_info", private$handle_)
      .check_status(result$status)
      info <- result$value
      info$board_name <- .Call("adrv_r_board_name", info$board_id)
      info$streaming <- bitwAnd(info$flags, .FLAG_STREAMING) != 0
      info$events <- bitwAnd(info$flags, .FLAG_EVENTS) != 0
      info$vref_volts <- info$vref_mv / 1000
      info$io_volts <- info$io_mv / 1000
      info
    },

    #' @description Number of addressable pins.
    pin_count = function() {
      .Call("adrv_r_pin_count", private$handle_)
    },

    #' @description Sets a pin's mode; see PinMode.
    pin_mode = function(pin, mode) {
      .check_status(.Call("adrv_r_pin_mode", private$handle_, as.integer(pin), as.integer(mode)))
      invisible(self)
    },

    digital_write = function(pin, high) {
      .check_status(.Call("adrv_r_digital_write", private$handle_, as.integer(pin), isTRUE(high)))
      invisible(self)
    },

    digital_read = function(pin) {
      result <- .Call("adrv_r_digital_read", private$handle_, as.integer(pin))
      .check_status(result$status)
      result$value
    },

    #' @description One entry per pin, pin i at index i + 1.
    read_all_digital = function() {
      result <- .Call("adrv_r_read_all_digital", private$handle_)
      .check_status(result$status)
      result$value
    },

    analog_read = function(pin) {
      result <- .Call("adrv_r_analog_read", private$handle_, as.integer(pin))
      .check_status(result$status)
      result$value
    },

    analog_read_volts = function(pin) {
      result <- .Call("adrv_r_analog_read_volts", private$handle_, as.integer(pin))
      .check_status(result$status)
      result$value
    },

    #' @description One raw sample per analog pin, in ascending pin order.
    read_all_analog = function() {
      result <- .Call("adrv_r_read_all_analog", private$handle_)
      .check_status(result$status)
      result$value
    },

    pwm_write = function(pin, duty) {
      .check_status(.Call("adrv_r_pwm_write", private$handle_, as.integer(pin), as.integer(duty)))
      invisible(self)
    },

    pwm_write_fraction = function(pin, fraction) {
      .check_status(.Call("adrv_r_pwm_write_fraction", private$handle_, as.integer(pin), as.double(fraction)))
      invisible(self)
    },

    dac_write = function(pin, value) {
      .check_status(.Call("adrv_r_dac_write", private$handle_, as.integer(pin), as.integer(value)))
      invisible(self)
    },

    dac_write_volts = function(pin, volts) {
      .check_status(.Call("adrv_r_dac_write_volts", private$handle_, as.integer(pin), as.double(volts)))
      invisible(self)
    },

    #' @description A list(last_error, queue_pending); see adrv_device_status().
    status = function() {
      result <- .Call("adrv_r_device_status", private$handle_)
      .check_status(result$status)
      result$value
    },

    sync = function() {
      .check_status(.Call("adrv_r_sync", private$handle_))
      invisible(self)
    },

    reset = function() {
      .check_status(.Call("adrv_r_reset", private$handle_))
      invisible(self)
    },

    #' @description Starts continuous sampling of `pins`; returns a Stream.
    start_stream = function(pins, period_us = 0L, digital = FALSE, queue_capacity = 0L) {
      flags <- if (isTRUE(digital)) .STREAM_FLAG_DIGITAL else 0L
      result <- .Call(
        "adrv_r_start_stream", private$handle_, as.integer(pins),
        as.integer(period_us), as.integer(flags), as.integer(queue_capacity)
      )
      .check_status(result$status)
      Stream$new(result$value)
    }
  ),
  private = list(
    context_ = NULL,
    handle_ = NULL
  )
)

#' Opens the first identified UsbIo device
#' @export
device_open_first <- function() {
  ctx <- .Call("adrv_r_context_create")
  if (is.null(ctx)) {
    .check_status(14L) # ADRV_ERR_OTHER
  }
  result <- .Call("adrv_r_device_open_first", ctx)
  if (!identical(result$status, 0L)) {
    .Call("adrv_r_context_destroy", ctx)
    .check_status(result$status)
  }
  Device$new(ctx, result$value)
}

#' Opens the identified device with this USB serial number
#' @export
device_open_by_serial <- function(serial) {
  ctx <- .Call("adrv_r_context_create")
  if (is.null(ctx)) {
    .check_status(14L) # ADRV_ERR_OTHER
  }
  result <- .Call("adrv_r_device_open_by_serial", ctx, as.character(serial))
  if (!identical(result$status, 0L)) {
    .Call("adrv_r_context_destroy", ctx)
    .check_status(result$status)
  }
  Device$new(ctx, result$value)
}
