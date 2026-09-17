# errors.R - turns a non-OK adrv_status_t into a classed R condition, so
# callers can tryCatch() on the class matching ArduinoDriver's C++ exception
# hierarchy (Errors.h) / arduino_driver_c.h's adrv_status_t.

.status_classes <- list(
  `1` = "arduinodriver_usb_error",
  `2` = c("arduinodriver_stall_error", "arduinodriver_usb_error"),
  `3` = c("arduinodriver_timeout_error", "arduinodriver_usb_error"),
  `4` = "arduinodriver_protocol_error",
  `5` = "arduinodriver_device_busy_error",
  `6` = "arduinodriver_invalid_pin_error",
  `7` = "arduinodriver_invalid_mode_error",
  `8` = "arduinodriver_invalid_value_error",
  `9` = "arduinodriver_not_supported_error",
  `10` = "arduinodriver_queue_full_error",
  `11` = "arduinodriver_not_ready_error",
  `12` = "arduinodriver_device_not_found_error",
  `13` = "arduinodriver_invalid_argument_error",
  `14` = character(0)
)

# Raises an error classed to match `status` (an adrv_status_t; a no-op for
# ADRV_OK == 0). Every raised condition carries "arduinodriver_error" so
# `tryCatch(..., arduinodriver_error = function(e) ...)` catches any of them.
.check_status <- function(status) {
  if (identical(status, 0L)) {
    return(invisible(NULL))
  }
  message <- .Call("adrv_r_last_error_message")
  extra_classes <- .status_classes[[as.character(status)]]
  classes <- c(extra_classes, "arduinodriver_error", "error", "condition")
  stop(structure(
    class = classes,
    list(message = message, call = sys.call(-1))
  ))
}
