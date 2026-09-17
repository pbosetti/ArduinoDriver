# Unit tests for the C API against the fake firmware (extras/driver/tests'
# FakeTransport, reused via ArduinoDriver::fake). TestRig.h/FakeTransport.h
# come straight from extras/driver/tests/: no point duplicating them.
add_executable(arduino_driver_c_tests
  TestBackdoor.h
  test_c_api.cpp
)
target_compile_features(arduino_driver_c_tests PRIVATE cxx_std_20)
target_include_directories(arduino_driver_c_tests PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/.."
  "${CMAKE_CURRENT_SOURCE_DIR}/../../driver/tests")
target_link_libraries(arduino_driver_c_tests PRIVATE
  ArduinoDriver::arduino_driver_c
  ArduinoDriver::fake
  ArduinoDriver::arduino_driver
  Catch2::Catch2WithMain)

add_test(NAME arduino_driver_c_tests COMMAND arduino_driver_c_tests)
