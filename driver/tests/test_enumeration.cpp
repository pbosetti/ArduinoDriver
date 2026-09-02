// test_enumeration.cpp - descriptor-based identification and probe policy,
// without libusb: the pure functions of Enumerator.h on hand-built inputs.
#include "arduino_driver/Enumerator.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdint>
#include <vector>

using namespace ArduinoDriver;

namespace {

constexpr InterfaceTriple CdcControl{0, 0x02, 0x02, 0x01};
constexpr InterfaceTriple CdcData{1, 0x0A, 0x00, 0x00};
constexpr InterfaceTriple UsbIo{2, USBIO_ITF_CLASS, USBIO_ITF_SUBCLASS,
                                USBIO_ITF_PROTOCOL};

} // namespace

TEST_CASE("find_vendor_interface recognises class FF / subclass 49 / "
          "protocol 4F",
          "[enumeration]") {
  SECTION("no interfaces at all") {
    CHECK_FALSE(find_vendor_interface({}).has_value());
  }
  SECTION("CDC only (Renesas boards, or a plain sketch)") {
    const std::array<InterfaceTriple, 2> cdc{CdcControl, CdcData};
    CHECK_FALSE(find_vendor_interface(cdc).has_value());
  }
  SECTION("CDC + UsbIo (mbed / SAMD boards)") {
    const std::array<InterfaceTriple, 3> composite{CdcControl, CdcData, UsbIo};
    REQUIRE(find_vendor_interface(composite).has_value());
    CHECK(*find_vendor_interface(composite) == 2);
  }
  SECTION("the interface number is reported, not the position") {
    const std::array<InterfaceTriple, 1> only{
        InterfaceTriple{5, 0xFF, 0x49, 0x4F}};
    CHECK(find_vendor_interface(only) == 5);
  }
  SECTION("other vendor-class interfaces do not match") {
    const std::array<InterfaceTriple, 3> others{
        InterfaceTriple{0, 0xFF, 0x00, 0x00}, // vendor, no subclass
        InterfaceTriple{1, 0xFF, 0x49, 0x00}, // right subclass only
        InterfaceTriple{2, 0xFE, 0x49, 0x4F}, // right sub/proto, wrong class
    };
    CHECK_FALSE(find_vendor_interface(others).has_value());
  }
  SECTION("first match wins") {
    const std::array<InterfaceTriple, 2> two{
        InterfaceTriple{3, 0xFF, 0x49, 0x4F},
        InterfaceTriple{4, 0xFF, 0x49, 0x4F}};
    CHECK(find_vendor_interface(two) == 3);
  }
}

TEST_CASE("matches_filter applies the explicit vid/pid selection",
          "[enumeration]") {
  EnumerateOptions none;
  CHECK(matches_filter(0x2341, 0x0069, none));
  CHECK(matches_filter(0x1234, 0x5678, none));

  EnumerateOptions vid_only;
  vid_only.vid = 0x2341;
  CHECK(matches_filter(0x2341, 0x0069, vid_only));
  CHECK(matches_filter(0x2341, 0xFFFF, vid_only));
  CHECK_FALSE(matches_filter(0x2342, 0x0069, vid_only));

  EnumerateOptions both;
  both.vid = 0x2341;
  both.pid = 0x0069;
  CHECK(matches_filter(0x2341, 0x0069, both));
  CHECK_FALSE(matches_filter(0x2341, 0x0070, both));
  CHECK_FALSE(matches_filter(0x2340, 0x0069, both));

  EnumerateOptions pid_only;
  pid_only.pid = 0x0069;
  CHECK(matches_filter(0x9999, 0x0069, pid_only));
  CHECK_FALSE(matches_filter(0x9999, 0x0068, pid_only));
}

TEST_CASE("is_probe_candidate: allow-listed vendors, or an explicit filter",
          "[enumeration]") {
  SECTION("without a filter only the known vendor ids are probed") {
    const EnumerateOptions none;
    CHECK(is_probe_candidate(USBIO_VID_ARDUINO, 0x0069, none));
    CHECK(is_probe_candidate(USBIO_VID_RASPBERRY_PI, 0x000A, none));
    CHECK(is_probe_candidate(USBIO_VID_ESPRESSIF, 0x1001, none));
    CHECK_FALSE(is_probe_candidate(0x1234, 0x5678, none));
    CHECK_FALSE(is_probe_candidate(0x0403, 0x6001, none)); // FTDI
    CHECK_FALSE(is_probe_candidate(0x2340, 0x0069, none)); // off by one
  }
  SECTION("an explicit vid vouches for any matching device") {
    EnumerateOptions filter;
    filter.vid = 0x1234;
    CHECK(is_probe_candidate(0x1234, 0x5678, filter));
    CHECK_FALSE(is_probe_candidate(0x1235, 0x5678, filter));
    CHECK_FALSE(is_probe_candidate(USBIO_VID_ARDUINO, 0x0069, filter));
  }
  SECTION("an explicit pid alone also vouches") {
    EnumerateOptions filter;
    filter.pid = 0x5678;
    CHECK(is_probe_candidate(0x1234, 0x5678, filter));
    CHECK_FALSE(is_probe_candidate(0x1234, 0x5679, filter));
  }
  SECTION("a filter narrows the allow-listed vendors too") {
    EnumerateOptions filter;
    filter.vid = USBIO_VID_ARDUINO;
    filter.pid = 0x0369;
    CHECK(is_probe_candidate(USBIO_VID_ARDUINO, 0x0369, filter));
    CHECK_FALSE(is_probe_candidate(USBIO_VID_ARDUINO, 0x0069, filter));
  }
  SECTION("the probe flag does not change candidacy") {
    EnumerateOptions no_probe;
    no_probe.probe = false;
    CHECK(is_probe_candidate(USBIO_VID_ARDUINO, 0x0069, no_probe));
  }
}

TEST_CASE("EnumerateOptions and DeviceInfo defaults", "[enumeration]") {
  const EnumerateOptions options;
  CHECK_FALSE(options.vid.has_value());
  CHECK_FALSE(options.pid.has_value());
  CHECK(options.probe);
  CHECK(options.probe_timeout == std::chrono::milliseconds{100});

  const DeviceInfo info;
  CHECK_FALSE(info.identified);
  CHECK_FALSE(info.has_vendor_interface);
  CHECK_FALSE(info.interface_number.has_value());
  CHECK_FALSE(info.info.has_value());
  CHECK(info.probe_error.empty());
  CHECK_FALSE(info.context);
  CHECK_FALSE(info.device);
}

TEST_CASE("open_device rejects an entry that did not come from list_devices",
          "[enumeration]") {
  const DeviceInfo hand_built;
  CHECK_THROWS_AS(open_device(hand_built), std::invalid_argument);
  CHECK_THROWS_AS(open_transport(nullptr, hand_built), std::invalid_argument);
}

TEST_CASE("keep_unprobed_candidate keeps only devices that could not be opened",
          "[enumeration]") {
  // Could not open: maybe ours, keep it visible.
  CHECK(keep_unprobed_candidate(LibusbError::Access));
  CHECK(keep_unprobed_candidate(LibusbError::NotSupported));
  CHECK(keep_unprobed_candidate(LibusbError::Busy));
  // The device answered (or vanished): not one of ours, drop it.
  CHECK_FALSE(keep_unprobed_candidate(LibusbError::Pipe)); // STALL
  CHECK_FALSE(keep_unprobed_candidate(LibusbError::Timeout));
  CHECK_FALSE(keep_unprobed_candidate(LibusbError::Io));
  CHECK_FALSE(keep_unprobed_candidate(LibusbError::NoDevice));
  CHECK_FALSE(keep_unprobed_candidate(LibusbError::NotFound));
  CHECK_FALSE(keep_unprobed_candidate(LibusbError::Overflow));
  CHECK_FALSE(keep_unprobed_candidate(LibusbError::Other));
  CHECK_FALSE(keep_unprobed_candidate(LibusbError::Success));
  CHECK_FALSE(keep_unprobed_candidate(-1234));
}
