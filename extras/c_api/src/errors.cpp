#include "error_mapping.h"

namespace adrv_detail {
namespace {
thread_local std::string g_last_error;
} // namespace

void set_last_error(const std::string &message) { g_last_error = message; }
void clear_last_error() { g_last_error.clear(); }

adrv_status_t invalid_argument(const std::string &message) {
  set_last_error(message);
  return ADRV_ERR_INVALID_ARGUMENT;
}

adrv_status_t handle_exception(const ArduinoDriver::Error &e) {
  using namespace ArduinoDriver;
  set_last_error(e.what());
  // Most-derived first: StallError/TimeoutError derive from UsbError.
  if (dynamic_cast<const StallError *>(&e) != nullptr) {
    return ADRV_ERR_STALL;
  }
  if (dynamic_cast<const TimeoutError *>(&e) != nullptr) {
    return ADRV_ERR_TIMEOUT;
  }
  if (dynamic_cast<const UsbError *>(&e) != nullptr) {
    return ADRV_ERR_USB;
  }
  if (dynamic_cast<const ProtocolError *>(&e) != nullptr) {
    return ADRV_ERR_PROTOCOL;
  }
  if (dynamic_cast<const DeviceBusy *>(&e) != nullptr) {
    return ADRV_ERR_DEVICE_BUSY;
  }
  if (dynamic_cast<const InvalidPin *>(&e) != nullptr) {
    return ADRV_ERR_INVALID_PIN;
  }
  if (dynamic_cast<const InvalidMode *>(&e) != nullptr) {
    return ADRV_ERR_INVALID_MODE;
  }
  if (dynamic_cast<const InvalidValue *>(&e) != nullptr) {
    return ADRV_ERR_INVALID_VALUE;
  }
  if (dynamic_cast<const NotSupported *>(&e) != nullptr) {
    return ADRV_ERR_NOT_SUPPORTED;
  }
  if (dynamic_cast<const QueueFull *>(&e) != nullptr) {
    return ADRV_ERR_QUEUE_FULL;
  }
  if (dynamic_cast<const NotReady *>(&e) != nullptr) {
    return ADRV_ERR_NOT_READY;
  }
  if (dynamic_cast<const DeviceNotFound *>(&e) != nullptr) {
    return ADRV_ERR_DEVICE_NOT_FOUND;
  }
  return ADRV_ERR_OTHER;
}

} // namespace adrv_detail

const char *adrv_last_error_message(void) {
  return adrv_detail::g_last_error.c_str();
}
