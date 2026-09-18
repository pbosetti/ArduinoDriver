# Builds arduino_driver_c and installs it into the wheel under
# arduino_driver/_native/, next to the pure-Python package (see
# pyproject.toml's tool.scikit-build.wheel.packages). scikit-build-core sets
# CMAKE_INSTALL_PREFIX so that these DESTINATION paths land at the wheel
# root, i.e. next to the arduino_driver/ package directory.
cmake_minimum_required(VERSION 3.24)
project(ArduinoDriverPython LANGUAGES CXX)

set(ARDUINODRIVER_C_API_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ARDUINODRIVER_C_API_INSTALL OFF CACHE BOOL "" FORCE)
# Build libusb's Linux backend without udev: it would need libudev headers
# (absent from the manylinux containers the wheels are built in) and would
# add a runtime dependency on libudev.so.1, which is not on the manylinux
# whitelist. libusb then uses its netlink hotplug path instead; enumeration
# (linux_usbfs) is unaffected, and this driver does not use hotplug at all.
set(LIBUSB_ENABLE_UDEV OFF CACHE BOOL "" FORCE)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../c_api" "${CMAKE_CURRENT_BINARY_DIR}/c_api")

install(TARGETS arduino_driver_c
  RUNTIME DESTINATION arduino_driver/_native  # Windows: the .dll
  LIBRARY DESTINATION arduino_driver/_native) # macOS/Linux: .dylib / .so
