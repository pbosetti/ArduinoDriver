#include "arduino_driver_c.h"
#include "error_mapping.h"
#include "internal_types.h"

#include <memory>

adrv_context_t *adrv_context_create(void) {
  adrv_detail::clear_last_error();
  try {
    auto handle = std::make_unique<adrv_context>();
    handle->ctx = std::make_shared<ArduinoDriver::Context>();
    return handle.release();
  } catch (const ArduinoDriver::Error &e) {
    adrv_detail::handle_exception(e);
    return nullptr;
  } catch (const std::exception &e) {
    adrv_detail::set_last_error(e.what());
    return nullptr;
  }
}

void adrv_context_destroy(adrv_context_t *ctx) { delete ctx; }
