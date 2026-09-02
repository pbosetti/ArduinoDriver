/*
 * UsbIoDevice - expose the board's pins to a host computer over USB.
 *
 * Flash this sketch, then drive the board with the ArduinoDriver host library
 * or the `arduino-io` CLI (digital I/O, analog input, PWM, DAC). The board
 * keeps its normal USB serial port, so Serial.print() still works for
 * debugging. All the work happens inside UsbIo.begin() / UsbIo.poll(); keep
 * loop() short so queued commands execute promptly.
 */
#include <UsbIo.h>

void setup() {
  UsbIo.begin();
}

void loop() {
  UsbIo.poll();
}
