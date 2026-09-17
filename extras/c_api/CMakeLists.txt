# ArduinoDriver C API - extern "C" shared library wrapping extras/driver,
# for language bindings (Python, R, ...) that cannot link the C++ API
# directly. See include/arduino_driver_c.h for the public surface.
#
# Works both as a subdirectory of the workspace root (which already brought
# in ArduinoDriver::arduino_driver) and standalone:
#   cmake -S extras/c_api -B build -G Ninja && cmake --build build && ctest --test-dir build
cmake_minimum_required(VERSION 3.24)

project(ArduinoDriverCApi
  VERSION 0.4.0
  DESCRIPTION "C ABI wrapper around the ArduinoDriver host library"
  LANGUAGES CXX)

option(ARDUINODRIVER_C_API_BUILD_TESTS "Build the C API's own tests" ON)
option(ARDUINODRIVER_C_API_INSTALL "Add extras/c_api's own install() rules (off for consumers, like extras/python, that install the target their own way)" ON)
option(ARDUINODRIVER_C_API_SHARED "Build arduino_driver_c as a shared library (OFF for a static archive, e.g. for extras/r, which links it into one final .so of its own)" ON)

# Standalone configure: bring in extras/driver the same way the workspace
# root normally does. When fetched as a subdirectory of that root (or of
# anything that already added extras/driver), the target already exists.
if(NOT TARGET ArduinoDriver::arduino_driver)
  set(ARDUINODRIVER_BUILD_TESTS ${ARDUINODRIVER_C_API_BUILD_TESTS} CACHE BOOL "" FORCE)
  set(ARDUINODRIVER_BUILD_CLI OFF CACHE BOOL "" FORCE)
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../driver" "${CMAKE_CURRENT_BINARY_DIR}/_driver")
endif()

include(GenerateExportHeader)

if(ARDUINODRIVER_C_API_SHARED)
  set(_arduinodriver_c_api_lib_type SHARED)
else()
  set(_arduinodriver_c_api_lib_type STATIC)
endif()

add_library(arduino_driver_c ${_arduinodriver_c_api_lib_type}
  include/arduino_driver_c.h
  src/internal_types.h
  src/error_mapping.h
  src/errors.cpp
  src/context.cpp
  src/device.cpp
  src/stream.cpp
)
add_library(ArduinoDriver::arduino_driver_c ALIAS arduino_driver_c)
set_target_properties(arduino_driver_c PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_compile_features(arduino_driver_c PUBLIC cxx_std_20)
if(MSVC)
  target_compile_options(arduino_driver_c PRIVATE /W4 /permissive- /utf-8 /Zc:__cplusplus)
  target_compile_definitions(arduino_driver_c PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)
else()
  target_compile_options(arduino_driver_c PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Deliberately NOT setting a hidden C_VISIBILITY_PRESET / CXX_VISIBILITY_
# PRESET here: this target statically links arduino_driver, whose exception
# hierarchy (Errors.h) is thrown from inside that archive and caught here
# via dynamic_cast (error_mapping.h). If the driver's RTTI became hidden in
# this shared library while the same static archive (or another copy of it)
# is also linked directly into another image in the same process -- which
# happens in this repo's own C API tests, which build Device fixtures on a
# FakeTransport without going through adrv_context_create() -- the two
# images end up with unmerged typeinfo for the same class and dynamic_cast
# silently fails to match, misclassifying every thrown error as
# ADRV_ERR_OTHER. ADRV_API (below) already keeps the *intended* public
# surface to just the adrv_* functions; the cost of skipping hidden
# visibility is a larger exported symbol table, not a correctness risk.

generate_export_header(arduino_driver_c
  EXPORT_MACRO_NAME ADRV_API
  EXPORT_FILE_NAME "${CMAKE_CURRENT_BINARY_DIR}/include/adrv_export.h")

target_include_directories(arduino_driver_c PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(arduino_driver_c PRIVATE ArduinoDriver::arduino_driver)

if(ARDUINODRIVER_C_API_INSTALL)
  include(GNUInstallDirs)
  install(TARGETS arduino_driver_c
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
  install(FILES
    include/arduino_driver_c.h
    "${CMAKE_CURRENT_BINARY_DIR}/include/adrv_export.h"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()

# The C API's own tests need ArduinoDriver::fake (extras/driver/tests'
# FakeTransport, built as a standalone static lib for exactly this reuse).
# It only exists when the driver's own tests were enabled above.
if(ARDUINODRIVER_C_API_BUILD_TESTS)
  if(TARGET ArduinoDriver::fake)
    enable_testing()
    add_subdirectory(tests)
  else()
    message(STATUS "ArduinoDriver C API: ArduinoDriver::fake not built (ARDUINODRIVER_BUILD_TESTS=OFF upstream), skipping C API tests")
  endif()
endif()
