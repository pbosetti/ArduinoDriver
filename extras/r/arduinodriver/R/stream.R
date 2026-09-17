# stream.R - Stream R6 class (Device$start_stream()); polling only,
# mirroring arduino_driver_c.h's adrv_stream_* entry points.

#' A continuous-sampling session (Device$start_stream())
#' @export
Stream <- R6::R6Class("Stream",
  public = list(
    #' @description Not a public constructor: use Device$start_stream().
    initialize = function(handle) {
      private$handle_ <- handle
      reg.finalizer(self, function(e) e$close(), onexit = TRUE)
    },

    #' @description Blocks until at least one sample is available or
    #' timeout_ms elapses; returns a data.frame with columns pin, raw,
    #' volts, t_us (0 rows on timeout).
    read = function(max_samples = 64L, timeout_ms = 1000L) {
      result <- .Call("adrv_r_stream_read", private$handle_, as.integer(max_samples), as.integer(timeout_ms))
      .check_status(result$status)
      as.data.frame(result$value)
    },

    #' @description Counters accumulated since the stream started.
    stats = function() {
      result <- .Call("adrv_r_stream_stats", private$handle_)
      .check_status(result$status)
      result$value
    },

    running = function() {
      .Call("adrv_r_stream_running", private$handle_)
    },

    #' @description Why the worker stopped on its own; "" while running
    #' normally or after a regular stop().
    error = function() {
      .Call("adrv_r_stream_error", private$handle_)
    },

    #' @description Stops the device stream and joins the worker thread.
    #' Idempotent.
    stop = function() {
      if (!is.null(private$handle_)) {
        .Call("adrv_r_stream_stop", private$handle_)
      }
      invisible(self)
    },

    #' @description Stops (if still running) and releases the stream.
    #' Idempotent.
    close = function() {
      if (!is.null(private$handle_)) {
        .Call("adrv_r_stream_destroy", private$handle_)
        private$handle_ <- NULL
      }
      invisible(self)
    }
  ),
  private = list(
    handle_ = NULL
  )
)
