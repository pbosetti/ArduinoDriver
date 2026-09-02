// TestRig.h - a Device on top of a FakeTransport, with the transport still
// reachable by the test.
#pragma once

#include "FakeTransport.h"
#include "arduino_driver/Device.h"

#include <chrono>
#include <cstddef>
#include <memory>
#include <span>
#include <utility>

namespace ArduinoDriver::Testing {

/// Default options minus the back-offs, so retries do not slow the tests.
inline Device::Options fast_options() {
  Device::Options options;
  options.busy_delay = std::chrono::microseconds{0};
  options.ready_delay = std::chrono::milliseconds{0};
  return options;
}

struct Rig {
  explicit Rig(FakeBoard board = FakeBoard::uno_r4_minima(),
               Device::Options options = fast_options())
      : Rig(std::make_unique<FakeTransport>(std::move(board)), options) {}

  FakeTransport &fake;
  Device device;

private:
  Rig(std::unique_ptr<FakeTransport> transport, Device::Options options)
      : fake(*transport), device(std::move(transport), options) {}
};

} // namespace ArduinoDriver::Testing

namespace ArduinoDriver::Testing {

/// Transport that forwards to one it does not own: lets a test keep its
/// FakeTransport (and the request log) when a Device fails to construct,
/// since a Device destroys the transport it was given.
class BorrowedTransport final : public Transport {
public:
  explicit BorrowedTransport(Transport &target) : _target(target) {}

  std::size_t control_in(std::uint8_t request, std::uint16_t value,
                         std::uint16_t index, std::span<std::byte> data,
                         std::chrono::milliseconds timeout) override {
    return _target.control_in(request, value, index, data, timeout);
  }
  void control_out(std::uint8_t request, std::uint16_t value,
                   std::uint16_t index,
                   std::chrono::milliseconds timeout) override {
    _target.control_out(request, value, index, timeout);
  }
  std::size_t bulk_in(std::span<std::byte> data,
                      std::chrono::milliseconds timeout) override {
    return _target.bulk_in(data, timeout);
  }

private:
  Transport &_target;
};

} // namespace ArduinoDriver::Testing
