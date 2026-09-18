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

# Windows has no rpath: the loader looks next to the .exe, not in the
# sibling directory where arduino_driver_c.dll is built. Without this the
# test binary cannot resolve its import and never reaches main() -- which
# shows up in CI as a test that produces no output and hangs until the job
# times out, rather than as a clean failure.
if(WIN32)
  add_custom_command(TARGET arduino_driver_c_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:arduino_driver_c_tests>
            $<TARGET_FILE_DIR:arduino_driver_c_tests>
    COMMAND_EXPAND_LISTS)
endif()

add_test(NAME arduino_driver_c_tests COMMAND arduino_driver_c_tests)
# A wedged test must never burn a whole CI job again (see the comment above).
set_tests_properties(arduino_driver_c_tests PROPERTIES TIMEOUT 120)
