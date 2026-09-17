# Exercises the real native library (.Call -> arduino_driver_c ->
# ArduinoDriver -> libusb). Tests that need a board skip themselves when
# none is attached, so this file runs meaningfully both in CI (skips) and
# on a dev machine with a board plugged in (real hardware round-trip).

open_device_or_skip <- function() {
  tryCatch(
    device_open_first(),
    arduinodriver_device_not_found_error = function(e) {
      testthat::skip("no UsbIo board attached")
    }
  )
}

test_that("opening a nonexistent serial raises device_not_found", {
  expect_error(
    device_open_by_serial("nonexistent-serial-should-never-match"),
    class = "arduinodriver_device_not_found_error"
  )
})

test_that("info reports a sane board", {
  dev <- open_device_or_skip()
  on.exit(dev$close())
  info <- dev$info()
  expect_gt(info$n_pins, 0)
  expect_true(nzchar(info$board_name))
  expect_equal(dev$pin_count(), info$n_pins)
})

test_that("digital write/read round-trips", {
  dev <- open_device_or_skip()
  on.exit(dev$close())
  dev$pin_mode(0, PinMode$OUTPUT)
  dev$digital_write(0, TRUE)
  expect_true(dev$digital_read(0))
  dev$digital_write(0, FALSE)
  expect_false(dev$digital_read(0))
})

test_that("reset returns pins to input", {
  dev <- open_device_or_skip()
  on.exit(dev$close())
  dev$pin_mode(0, PinMode$OUTPUT)
  dev$reset()
  expect_error(dev$digital_write(0, TRUE), class = "arduinodriver_invalid_mode_error")
})
