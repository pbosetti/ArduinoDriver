// error_mapping.h - shared exception-to-status_t translation, used by every
// adrv_* entry point.
#pragma once

#include "arduino_driver/Errors.h"
#include "arduino_driver_c.h"

#include <exception>
#include <string>
#include <utility>

namespace adrv_detail {

void set_last_error(const std::string &message);
void clear_last_error();

// Maps a caught ArduinoDriver::Error to its adrv_status_t and records its
// message as the thread's last error.
adrv_status_t handle_exception(const ArduinoDriver::Error &e);

// Records `message` and returns ADRV_ERR_INVALID_ARGUMENT, for the null /
// out-of-range checks entry points make before touching the C++ API (no
// exception is thrown for those, so handle_exception() does not apply).
adrv_status_t invalid_argument(const std::string &message);

} // namespace adrv_detail

// Clears the thread's last-error message, runs `body`, and maps whatever it
// throws to an adrv_status_t; `body` reports its own success by returning
// normally (ADRV_OK) or by assigning to captured out-parameters before
// returning.
template <typename Body> adrv_status_t adrv_guard(Body &&body) {
  adrv_detail::clear_last_error();
  try {
    body();
    return ADRV_OK;
  } catch (const ArduinoDriver::Error &e) {
    return adrv_detail::handle_exception(e);
  } catch (const std::exception &e) {
    adrv_detail::set_last_error(e.what());
    return ADRV_ERR_OTHER;
  } catch (...) {
    adrv_detail::set_last_error("unknown error");
    return ADRV_ERR_OTHER;
  }
}
