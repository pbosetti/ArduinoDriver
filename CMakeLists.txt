cmake_minimum_required(VERSION 3.24)
project(ArduinoDriverWorkspace LANGUAGES NONE)

# ---- Host driver (C++20) --------------------------------------------------
# driver/ is a self-contained CMake project; it can also be configured alone
# with `cmake -S driver -B build`.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/driver/CMakeLists.txt")
  enable_testing() # so `ctest --test-dir build` finds the driver tests
  add_subdirectory(driver)
endif()

# ---- Firmware targets (optional, need arduino-cli on PATH) ----------------
# `cmake --build build --target firmware-<board>` compiles the example sketch
# for one board; `firmware-all` compiles every board whose core is installed.
# Upload with `-DUSBIO_UPLOAD_PORT=/dev/cu.usbmodemXXXX` and
# `cmake --build build --target upload-<board>`.
find_program(ARDUINO_CLI arduino-cli)
set(USBIO_UPLOAD_PORT "" CACHE STRING "Serial port used by the upload-<board> targets")

if(ARDUINO_CLI)
  set(_usbio_sketch "${CMAKE_CURRENT_SOURCE_DIR}/arduino/UsbIo/examples/UsbIoDevice")
  set(_usbio_library "${CMAKE_CURRENT_SOURCE_DIR}/arduino/UsbIo")
  set(_usbio_all_targets "")

  function(usbio_firmware_target name fqbn)
    set(build_dir "${CMAKE_BINARY_DIR}/firmware/${name}")
    add_custom_target(firmware-${name}
      COMMAND "${ARDUINO_CLI}" compile --fqbn "${fqbn}" --warnings all
              --library "${_usbio_library}" --build-path "${build_dir}"
              "${_usbio_sketch}"
      COMMENT "UsbIo firmware: ${fqbn}"
      VERBATIM)
    add_custom_target(upload-${name}
      COMMAND "${ARDUINO_CLI}" upload --fqbn "${fqbn}" -p "${USBIO_UPLOAD_PORT}"
              --input-dir "${build_dir}"
      DEPENDS firmware-${name}
      COMMENT "UsbIo upload: ${fqbn} on ${USBIO_UPLOAD_PORT}"
      VERBATIM)
    set(_usbio_all_targets ${_usbio_all_targets} firmware-${name} PARENT_SCOPE)
  endfunction()

  # Verified mechanism, cores installable from the official Arduino index.
  usbio_firmware_target(portenta  arduino:mbed_portenta:envie_m7)
  usbio_firmware_target(giga      arduino:mbed_giga:giga)
  usbio_firmware_target(nano33ble arduino:mbed_nano:nano33ble)
  usbio_firmware_target(minima    arduino:renesas_uno:minima)
  usbio_firmware_target(nanor4    arduino:renesas_uno:nanor4)
  usbio_firmware_target(mkrzero   arduino:samd:mkrzero)
  usbio_firmware_target(nano33iot arduino:samd:nano_33_iot)
  usbio_firmware_target(zero      arduino:samd:arduino_zero_native)

  add_custom_target(firmware-all DEPENDS ${_usbio_all_targets})
else()
  message(STATUS "arduino-cli not found: firmware-* targets disabled")
endif()
