/*
 * UsbIoDevice - expose the board's pins to a host computer over USB.
 * 
 * Portenta H7 version ****
 * This version also uses the second core to blink a blue LED on the board, 
 * so you can see that the main core is still running while the USB I/O is 
 * active.
 * See: https://github.com/arduino-libraries/Arduino_Pro_Tutorials/tree/main/examples/Dual%20Core%20Processing
 *
 * Flash this sketch, then drive the board with the ArduinoDriver host library
 * or the `arduino-io` CLI (digital I/O, analog input, PWM, DAC). The board
 * keeps its normal USB serial port, so Serial.print() still works for
 * debugging. All the work happens inside UsbIo.begin() / UsbIo.poll(); keep
 * loop() short so queued commands execute promptly.
 */
#ifdef CORE_CM7
#include <UsbIo.h>
#else
#include "blink_pattern.h"
// Use a specific LED pattern to help recognize the running firmware
BlinkPattern BP{2000, "#---##--###-"};
#endif

// Raise error if board is not a Portenta H7, since this sketch uses the second 
// core to blink a blue LED.
// #if !defined(CORE_CM4) || !defined(CORE_CM7)
//   #error This sketch is for the Portenta H7 only.
// #endif

void setup() {
#ifdef CORE_CM4
  pinMode(LEDB, OUTPUT);
#else
  bootM4();
  UsbIo.begin();
#endif
}

void loop() {
#ifdef CORE_CM7
  UsbIo.poll();
#else
  bool state = BP.state(millis());
  digitalWrite(LEDB, state);
  // static unsigned long last = 0;
  // if (millis() - last >= 250) {
  //   last = millis();
  //   digitalWrite(LEDB, !digitalRead(LEDB));
  // }
#endif
}
