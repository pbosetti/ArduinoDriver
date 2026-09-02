# Unit tests: Catch2 v3 against an in-process model of the firmware, plus the
# hidden hardware tests.

# FakeTransport is a small library of its own so that other tests and tools
# wanting a dry-run mode can reuse it.
add_library(arduino_driver_fake STATIC
  FakeTransport.h
  FakeTransport.cpp)
add_library(ArduinoDriver::fake ALIAS arduino_driver_fake)
arduinodriver_configure_target(arduino_driver_fake)
target_include_directories(arduino_driver_fake PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(arduino_driver_fake PUBLIC ArduinoDriver::arduino_driver)

add_executable(arduino_driver_tests
  TestRig.h
  test_protocol_encoding.cpp
  test_device_api.cpp
  test_enumeration.cpp
  test_hardware.cpp # [.hardware]: hidden unless selected explicitly
)
arduinodriver_configure_target(arduino_driver_tests)
target_link_libraries(arduino_driver_tests PRIVATE
  ArduinoDriver::fake
  Catch2::Catch2WithMain)

list(APPEND CMAKE_MODULE_PATH "${catch2_SOURCE_DIR}/extras")
include(Catch)
catch_discover_tests(arduino_driver_tests) # hidden tests are not discovered

# The hardware tests need a board: one CTest entry, disabled unless asked for
# (-DARDUINODRIVER_HARDWARE_TESTS=ON, then `ctest -R hardware`).
add_test(NAME arduino_driver_hardware
  COMMAND arduino_driver_tests "[hardware]" --warn NoAssertions)
if(NOT ARDUINODRIVER_HARDWARE_TESTS)
  set_tests_properties(arduino_driver_hardware PROPERTIES DISABLED TRUE)
endif()
