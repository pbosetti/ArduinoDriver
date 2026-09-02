# ArduinoDriver - libusb host driver for the UsbIo Arduino firmware.
#
# Works both as a subdirectory of the workspace root and standalone:
#   cmake -S extras/driver -B build -G Ninja && cmake --build build && ctest --test-dir build
cmake_minimum_required(VERSION 3.24)

project(ArduinoDriver
  VERSION 0.1.0
  DESCRIPTION "libusb host driver for the UsbIo Arduino firmware"
  LANGUAGES C CXX) # C: libusb is a C library

# ---- Options ---------------------------------------------------------------
option(ARDUINODRIVER_SYSTEM_LIBUSB
  "Use the system libusb-1.0 (pkg-config) instead of building it with FetchContent" OFF)
option(ARDUINODRIVER_BUILD_TESTS "Build the Catch2 unit tests" ON)
option(ARDUINODRIVER_BUILD_CLI "Build the arduino-io command line tool" ON)
option(ARDUINODRIVER_WARNINGS_AS_ERRORS "Treat warnings in the ArduinoDriver targets as errors" OFF)
option(ARDUINODRIVER_HARDWARE_TESTS "Enable the CTest entry that talks to a real board" OFF)

# Directory holding usbio_protocol.h, the wire protocol shared with the
# firmware (single source of truth; included verbatim by the driver). The
# driver sits at extras/driver/, so the library's src/ is two levels up.
set(ARDUINODRIVER_PROTOCOL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../src"
  CACHE PATH "Directory containing usbio_protocol.h")
cmake_path(ABSOLUTE_PATH ARDUINODRIVER_PROTOCOL_DIR NORMALIZE)
if(NOT EXISTS "${ARDUINODRIVER_PROTOCOL_DIR}/usbio_protocol.h")
  message(FATAL_ERROR
    "usbio_protocol.h not found in '${ARDUINODRIVER_PROTOCOL_DIR}'. "
    "Set ARDUINODRIVER_PROTOCOL_DIR to the directory that contains it.")
endif()

# ---- Global settings (this directory and below, fetched dependencies included)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

# Warning flags are applied per target through this function, so that the
# fetched dependencies keep their own settings.
function(arduinodriver_configure_target target)
  target_compile_features(${target} PUBLIC cxx_std_20)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8 /Zc:__cplusplus)
    target_compile_definitions(${target} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)
    if(ARDUINODRIVER_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    if(ARDUINODRIVER_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
endfunction()

# ---- Dependencies (FetchContent, pinned to release tags) -------------------
# Every dependency declares cmake_minimum_required >= 3.13, so policy CMP0077
# is NEW for them and plain set() of their option names below takes effect.
include(FetchContent)

# fmt: text formatting (library error messages, CLI output).
FetchContent_Declare(fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG 11.2.0
  GIT_SHALLOW TRUE)
set(FMT_INSTALL OFF)
set(FMT_DOC OFF)
set(FMT_TEST OFF)
FetchContent_MakeAvailable(fmt)

# libusb: either the official CMake wrapper (target usb-1.0) or the system
# package. Both are exposed to our targets as ArduinoDriver::libusb.
add_library(arduinodriver_libusb INTERFACE)
add_library(ArduinoDriver::libusb ALIAS arduinodriver_libusb)
if(ARDUINODRIVER_SYSTEM_LIBUSB)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)
  target_link_libraries(arduinodriver_libusb INTERFACE PkgConfig::LIBUSB)
  message(STATUS "ArduinoDriver: using system libusb-1.0 ${LIBUSB_VERSION}")
else()
  FetchContent_Declare(libusb
    GIT_REPOSITORY https://github.com/libusb/libusb-cmake.git
    GIT_TAG v1.0.30-0
    GIT_SHALLOW TRUE) # the libusb sources are a git submodule of the wrapper
  set(LIBUSB_BUILD_SHARED_LIBS OFF)
  set(LIBUSB_BUILD_TESTING OFF)
  set(LIBUSB_BUILD_EXAMPLES OFF)
  set(LIBUSB_INSTALL_TARGETS OFF)
  FetchContent_MakeAvailable(libusb)
  target_link_libraries(arduinodriver_libusb INTERFACE usb-1.0)
  message(STATUS "ArduinoDriver: building libusb-1.0 from libusb-cmake v1.0.30-0")
endif()

# cxxopts: CLI option parsing (header-only).
if(ARDUINODRIVER_BUILD_CLI)
  FetchContent_Declare(cxxopts
    GIT_REPOSITORY https://github.com/jarro2783/cxxopts.git
    GIT_TAG v3.3.1
    GIT_SHALLOW TRUE)
  set(CXXOPTS_BUILD_EXAMPLES OFF)
  set(CXXOPTS_BUILD_TESTS OFF)
  set(CXXOPTS_ENABLE_INSTALL OFF)
  set(CXXOPTS_ENABLE_WARNINGS OFF)
  FetchContent_MakeAvailable(cxxopts)
endif()

# Catch2 v3: unit tests.
if(ARDUINODRIVER_BUILD_TESTS)
  FetchContent_Declare(Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.16.0
    GIT_SHALLOW TRUE)
  set(CATCH_INSTALL_DOCS OFF)
  set(CATCH_INSTALL_EXTRAS OFF)
  FetchContent_MakeAvailable(Catch2)
endif()

# ---- Library ---------------------------------------------------------------
find_package(Threads REQUIRED)

add_library(arduino_driver STATIC
  include/arduino_driver/Errors.h
  include/arduino_driver/Protocol.h
  include/arduino_driver/Transport.h
  include/arduino_driver/Device.h
  include/arduino_driver/Stream.h
  include/arduino_driver/LibusbTransport.h
  include/arduino_driver/Enumerator.h
  src/LibusbDetail.h
  src/Device.cpp
  src/Stream.cpp
  src/LibusbTransport.cpp
  src/Enumerator.cpp
)
add_library(ArduinoDriver::arduino_driver ALIAS arduino_driver)
arduinodriver_configure_target(arduino_driver)
target_include_directories(arduino_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${ARDUINODRIVER_PROTOCOL_DIR}>)
target_link_libraries(arduino_driver
  PUBLIC ArduinoDriver::libusb
  PRIVATE fmt::fmt Threads::Threads)

# ---- CLI -------------------------------------------------------------------
if(ARDUINODRIVER_BUILD_CLI)
  add_executable(arduino-io tools/arduino-io.cpp)
  arduinodriver_configure_target(arduino-io)
  target_link_libraries(arduino-io PRIVATE
    ArduinoDriver::arduino_driver
    fmt::fmt
    cxxopts::cxxopts)
endif()

# ---- Tests -----------------------------------------------------------------
if(ARDUINODRIVER_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
