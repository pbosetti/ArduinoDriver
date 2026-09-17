// internal_types.h - definitions completing the opaque handles declared in
// arduino_driver_c.h. Included by every src/*.cpp and by the C API's own
// tests, never by binding code outside this library.
#pragma once

#include "arduino_driver/Device.h"
#include "arduino_driver/LibusbTransport.h"
#include "arduino_driver/Stream.h"

#include <memory>

struct adrv_context {
  std::shared_ptr<ArduinoDriver::Context> ctx;
};

struct adrv_device {
  std::unique_ptr<ArduinoDriver::Device> device;
  // Keeps the context (and therefore libusb) alive for as long as the
  // device is open; empty for a device wrapped directly around a test
  // transport (see tests/TestBackdoor.h).
  std::shared_ptr<ArduinoDriver::Context> context;
};

struct adrv_stream {
  std::unique_ptr<ArduinoDriver::Stream> stream;
  std::string last_error; // adrv_stream_error()'s backing storage
};
