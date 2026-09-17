# .check_status(): every adrv_status_t maps to the right condition class,
# and ADRV_OK is a no-op. Pure logic -- no native calls, no hardware needed.

test_that("ADRV_OK is a no-op", {
  expect_null(.check_status(0L))
})

test_that("each status code raises the matching condition class", {
  expectations <- list(
    list(status = 1L, class = "arduinodriver_usb_error"),
    list(status = 2L, class = "arduinodriver_stall_error"),
    list(status = 3L, class = "arduinodriver_timeout_error"),
    list(status = 4L, class = "arduinodriver_protocol_error"),
    list(status = 5L, class = "arduinodriver_device_busy_error"),
    list(status = 6L, class = "arduinodriver_invalid_pin_error"),
    list(status = 7L, class = "arduinodriver_invalid_mode_error"),
    list(status = 8L, class = "arduinodriver_invalid_value_error"),
    list(status = 9L, class = "arduinodriver_not_supported_error"),
    list(status = 10L, class = "arduinodriver_queue_full_error"),
    list(status = 11L, class = "arduinodriver_not_ready_error"),
    list(status = 12L, class = "arduinodriver_device_not_found_error"),
    list(status = 13L, class = "arduinodriver_invalid_argument_error")
  )
  for (e in expectations) {
    err <- tryCatch(
      .check_status(e$status),
      error = function(cond) cond
    )
    expect_s3_class(err, e$class)
    expect_s3_class(err, "arduinodriver_error")
  }
})

test_that("stall is also a usb error", {
  err <- tryCatch(.check_status(2L), error = function(cond) cond)
  expect_s3_class(err, "arduinodriver_usb_error")
})

test_that("status 14 (ADRV_ERR_OTHER) still raises an arduinodriver_error", {
  err <- tryCatch(.check_status(14L), error = function(cond) cond)
  expect_s3_class(err, "arduinodriver_error")
})
