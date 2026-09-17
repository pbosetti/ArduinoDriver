// TestBackdoor.h - wraps a Device already built on a test transport (e.g.
// ArduinoDriver::Testing::Rig, on top of FakeTransport) into an adrv_device_t,
// so the C API can be exercised without real libusb hardware. Not part of
// the public API or the installed headers.
#pragma once

#include "../src/internal_types.h"
#include "arduino_driver_c.h"

#include <memory>
#include <utility>

inline adrv_device_t *adrv_test_wrap_device(ArduinoDriver::Device &&device) {
  auto *handle = new adrv_device();
  handle->device = std::make_unique<ArduinoDriver::Device>(std::move(device));
  return handle;
}
