# Stream (continuous sampling): needs a board with the streaming capability
# and an analog-capable pin, so it skips itself when neither is available.

open_streaming_device_or_skip <- function() {
  dev <- tryCatch(
    device_open_first(),
    arduinodriver_device_not_found_error = function(e) testthat::skip("no UsbIo board attached")
  )
  if (!dev$info()$streaming) {
    dev$close()
    testthat::skip("attached board does not support streaming")
  }
  dev
}

find_analog_pin <- function(dev) {
  for (pin in seq_len(dev$info()$n_pins) - 1L) {
    ok <- tryCatch(
      {
        dev$pin_mode(pin, PinMode$ANALOG_IN)
        TRUE
      },
      arduinodriver_not_supported_error = function(e) FALSE
    )
    if (ok) {
      return(pin)
    }
  }
  testthat::skip("attached board has no analog-capable pin")
}

test_that("start_stream / read / stop round-trips", {
  dev <- open_streaming_device_or_skip()
  on.exit(dev$close())
  pin <- find_analog_pin(dev)

  stream <- dev$start_stream(pin, period_us = 1000L)
  on.exit(stream$close(), add = TRUE)

  expect_true(stream$running())
  samples <- stream$read(max_samples = 16L, timeout_ms = 1000L)
  expect_gt(nrow(samples), 0)
  expect_true(all(samples$pin == pin))

  stats <- stream$stats()
  expect_gte(stats$records_received, nrow(samples))

  stream$stop()
  expect_false(stream$running())
})
