/*
 * UsbIoBoard.h - per-architecture capability discovery.
 *
 * Everything below is derived at begin() time from the core's own pin tables
 * and macros; there are no hand-written per-board tables. The few numbers the
 * cores do not expose (resolution honoured by analogRead()/analogWrite(),
 * reference voltage, pull-down support) are per-architecture constants, each
 * annotated with the core source line that justifies it.
 *
 * Pin numbers are Arduino pin numbers. Rules shared by every architecture:
 *   AIN  pins PIN_A0 .. PIN_A0 + NUM_ANALOG_INPUTS - 1 (contiguous on every
 *        supported variant).
 *   DAC  the pin(s) named by the core's DAC macro(s).
 *   The addressable range (pin_count()) is the last "named" pin + 1: digital
 *   header, analog header and on-board LEDs. It is clamped to the core's own
 *   PINS_COUNT and to USBIO_MAX_PINS.
 */
#ifndef USBIO_BOARD_H
#define USBIO_BOARD_H

#include <Arduino.h>
#include <stdint.h>

#include "usbio_protocol.h"

namespace UsbIoBoard {

/* Shared helper: highest named pin + 1, see the header comment. Every macro is
 * optional because variants differ in what they define. */
inline uint8_t named_pin_count() {
  unsigned n = NUM_DIGITAL_PINS;
#if defined(PIN_A0)
  if ((unsigned)PIN_A0 + (unsigned)NUM_ANALOG_INPUTS > n) {
    n = (unsigned)PIN_A0 + (unsigned)NUM_ANALOG_INPUTS;
  }
#endif
#if defined(LED_BUILTIN)
  if ((unsigned)LED_BUILTIN + 1u > n) {
    n = (unsigned)LED_BUILTIN + 1u;
  }
#endif
#if defined(LEDR)
  if ((unsigned)LEDR + 1u > n) {
    n = (unsigned)LEDR + 1u;
  }
#endif
#if defined(LEDG)
  if ((unsigned)LEDG + 1u > n) {
    n = (unsigned)LEDG + 1u;
  }
#endif
#if defined(LEDB)
  if ((unsigned)LEDB + 1u > n) {
    n = (unsigned)LEDB + 1u;
  }
#endif
#if defined(LED_PWR)
  if ((unsigned)LED_PWR + 1u > n) {
    n = (unsigned)LED_PWR + 1u;
  }
#endif
#if defined(PINS_COUNT)
  if (n > (unsigned)PINS_COUNT) {
    n = (unsigned)PINS_COUNT;
  }
#endif
  if (n > USBIO_MAX_PINS) {
    n = USBIO_MAX_PINS;
  }
  return (uint8_t)n;
}

inline bool analog_header_pin(uint8_t p) {
#if defined(PIN_A0)
  return (unsigned)p >= (unsigned)PIN_A0 &&
         (unsigned)p < (unsigned)PIN_A0 + (unsigned)NUM_ANALOG_INPUTS;
#else
  (void)p;
  return false;
#endif
}

/* ======================================================================== */
#if defined(ARDUINO_ARCH_MBED)
/* Arduino mbed cores: Portenta H7, GIGA R1, Nano 33 BLE, Nano RP2040 Connect.
 * Board macros: ARDUINO_{build.board}, boards.txt build.board = variant name. */

/* STM targets publish the timer pinmap as `const PinMap PinMap_PWM[]`
 * (targets/TARGET_STM/PeripheralPins.h:88). The nRF52 and RP2040 targets use
 * other names/types (pinmap_ex.h `PinMapPWM PinMap_PWM[]`, `PinMap_PWM_OUT[]`)
 * but there every GPIO can be routed to a PWM unit, so no lookup is needed. */
#if defined(TARGET_STM) && defined(DEVICE_PWMOUT)
#include "PeripheralPins.h"
#define USBIO_MBED_PWM_PINMAP 1
#else
#define USBIO_MBED_PWM_PINMAP 0
#endif

#if defined(ARDUINO_PORTENTA_H7_M7)
static const uint16_t BoardId = USBIO_BOARD_PORTENTA_H7;
#elif defined(ARDUINO_GIGA)
static const uint16_t BoardId = USBIO_BOARD_GIGA_R1;
#elif defined(ARDUINO_ARDUINO_NANO33BLE)
static const uint16_t BoardId = USBIO_BOARD_NANO_33_BLE;
#elif defined(ARDUINO_NANO_RP2040_CONNECT)
/* mbed_nano core: the board is an RP2040 but the USB stack is mbed's. */
static const uint16_t BoardId = USBIO_BOARD_NANO_RP2040_CONNECT;
#else
static const uint16_t BoardId = USBIO_BOARD_MBED_GENERIC;
#endif

/* wiring_analog.cpp:120 returns read_u16() >> (16 - bits): 16 is the widest
 * setting that adds information (the STM32H7 ADC is natively 16-bit; the
 * nRF52840 delivers 12 significant bits, left-aligned).
 * wiring_analog.cpp:37/71 scale analogWrite() by (1 << bits) - 1 for the DAC
 * and the PWM alike; 12 bits matches the STM32 DAC. */
static const uint8_t AdcBits = 16;
static const uint8_t PwmBits = 12;
#if defined(DAC)
static const uint8_t DacBits = 12;
#else
static const uint8_t DacBits = 0;
#endif
static const uint16_t VrefMv = 3300;
static const uint16_t IoMv = 3300;
static const bool HasPulldown = true; /* wiring_digital.cpp:76 */

/* mbed variants list every MCU pad in g_APinDescription, so PINS_COUNT also
 * covers pads wired to the QSPI flash, SDRAM, radio... Reconfiguring those
 * from RESET would brick the board, hence the named-pin rule: Portenta H7 26
 * pins, GIGA R1 103 (NUM_DIGITAL_PINS; D92..D100 are the USB-host, WiFi/BLE
 * enable and BOOT0 lines, which a RESET does put in INPUT), Nano 33 BLE 26,
 * Nano RP2040 Connect 30. */
inline uint8_t pin_count() { return named_pin_count(); }

inline bool pin_has_dio(uint8_t p) {
  const PinName name = digitalPinToPinName(p);
  if (name == NC) {
    return false;
  }
#if defined(DUAL_PAD)
  /* STM32H7 "_C" pads (Portenta A0..A3) are ADC-only and have no GPIO behind
   * them: PORTENTA_H7_M7/pins_arduino.h:72 refuses them as digital pins. */
  if ((static_cast<uint32_t>(name) & DUAL_PAD) != 0) {
    return false;
  }
#endif
  return true;
}

inline bool pin_has_ain(uint8_t p) { return analog_header_pin(p); }

inline bool pin_has_pwm(uint8_t p) {
  if (!pin_has_dio(p)) {
    return false;
  }
#if USBIO_MBED_PWM_PINMAP
  /* pinmap_find_peripheral() returns NC for a pin absent from the map;
   * pinmap_peripheral() would raise MBED_ERROR instead (mbed-os
   * hal/source/mbed_pinmap_common.c). */
  return pinmap_find_peripheral(digitalPinToPinName(p), PinMap_PWM) !=
         static_cast<uint32_t>(NC);
#else
  return true; /* nRF52840 / RP2040: any GPIO can be routed to a PWM unit */
#endif
}

/* Portenta: DAC = A6 (pins_arduino.h:81). GIGA: DAC_0 = A12, DAC_1 = A13
 * (pins_arduino.h:64-66); wiring_analog.cpp:63-70 serves every entry of
 * g_AAnalogOutPinDescription, so both channels are reachable. */
inline bool pin_has_dac(uint8_t p) {
#if defined(DAC)
  if (p == DAC) {
    return true;
  }
#endif
#if defined(DAC_0)
  if (p == DAC_0) {
    return true;
  }
#endif
#if defined(DAC_1)
  if (p == DAC_1) {
    return true;
  }
#endif
  (void)p;
  return false;
}

/* ======================================================================== */
#elif defined(ARDUINO_ARCH_RENESAS)
/* Renesas RA4M1 (UNO R4 Minima, Nano R4). boards.txt:172 defines both
 * ARDUINO_NANO_R4 and ARDUINO_UNOR4_MINIMA for the Nano, so test it first. */

#if defined(ARDUINO_NANO_R4)
static const uint16_t BoardId = USBIO_BOARD_NANO_R4;
#elif defined(ARDUINO_UNOR4_MINIMA)
static const uint16_t BoardId = USBIO_BOARD_UNO_R4_MINIMA;
#else
static const uint16_t BoardId = USBIO_BOARD_RENESAS_GENERIC;
#endif

/* analog.cpp:604 accepts 8/10/12/14/16 and rescales from the fixed hardware
 * resolution, which is 14 bits on the RA4M1. analog.cpp:717 stores any PWM
 * resolution; 12 bits matches the 12-bit DAC (MINIMA/pins_arduino.h:23). */
static const uint8_t AdcBits = 14;
static const uint8_t PwmBits = 12;
#if defined(DAC)
static const uint8_t DacBits = 12;
#else
static const uint8_t DacBits = 0;
#endif
static const uint16_t VrefMv = 5000;
static const uint16_t IoMv = 5000;
static const bool HasPulldown = false; /* digital.cpp:6 maps it to INPUT */

inline uint8_t pin_count() { return named_pin_count(); }
/* digital.cpp:3-16 drives g_pin_cfg[pin] for any index without checking
 * NUM_DIGITAL_PINS, so every entry of the variant table is a GPIO (the Nano
 * R4 LEDs sit at 22..25, above NUM_DIGITAL_PINS = 20). */
inline bool pin_has_dio(uint8_t p) { return (unsigned)p < (unsigned)PINS_COUNT; }
inline bool pin_has_ain(uint8_t p) { return analog_header_pin(p); }
inline bool pin_has_pwm(uint8_t p) {
  return pin_has_dio(p) && digitalPinHasPWM(p); /* pins_arduino.h:93 */
}
inline bool pin_has_dac(uint8_t p) {
#if defined(IS_DAC)
  return IS_DAC(p); /* MINIMA/pins_arduino.h:25 */
#elif defined(DAC)
  return p == DAC;
#else
  (void)p;
  return false;
#endif
}

/* ======================================================================== */
#elif defined(ARDUINO_ARCH_SAMD)
/* SAMD21 (Zero, MKR family, Nano 33 IoT). All MKR boards get
 * -DUSE_ARDUINO_MKR_PIN_LAYOUT (boards.txt) except the Vidor. */

#if defined(ARDUINO_SAMD_ZERO)
static const uint16_t BoardId = USBIO_BOARD_ZERO;
#elif defined(ARDUINO_SAMD_NANO_33_IOT)
static const uint16_t BoardId = USBIO_BOARD_NANO_33_IOT;
#elif defined(USE_ARDUINO_MKR_PIN_LAYOUT) || defined(ARDUINO_SAMD_MKRVIDOR4000)
static const uint16_t BoardId = USBIO_BOARD_MKR;
#else
static const uint16_t BoardId = USBIO_BOARD_SAMD_GENERIC;
#endif

/* wiring_analog.c:59: any request above 10 selects the 12-bit hardware mode.
 * wiring_analog.c:207/219: one write resolution feeds both the 10-bit DAC and
 * the 16-bit PWM timers, so 10 bits keeps both paths lossless. */
static const uint8_t AdcBits = 12;
static const uint8_t PwmBits = 10;
static const uint8_t DacBits = 10; /* every SAMD21 has the one 10-bit DAC */
static const uint16_t VrefMv = 3300;
static const uint16_t IoMv = 3300;
static const bool HasPulldown = true; /* wiring_digital.c:51 */

inline uint8_t pin_count() { return named_pin_count(); }
/* wiring_digital.c:pinMode() only refuses PIO_NOT_A_PIN entries; PIN_ATTR_*
 * is not consulted (the MKR LED is PIN_ATTR_NONE). PIO_COM marks the USB pads,
 * which a RESET must never reconfigure. */
inline bool pin_has_dio(uint8_t p) {
  if ((unsigned)p >= (unsigned)PINS_COUNT) {
    return false;
  }
  const EPioType type = g_APinDescription[p].ulPinType;
  return type != PIO_NOT_A_PIN && type != PIO_COM;
}
inline bool pin_has_ain(uint8_t p) { return analog_header_pin(p); }
inline bool pin_has_pwm(uint8_t p) {
  return pin_has_dio(p) && digitalPinHasPWM(p); /* variant.h:71 */
}
/* wiring_analog.c:199-203 routes analogWrite() to the DAC for any
 * PIN_ATTR_ANALOG entry on ADC/DAC channel 0. Several MKR variants never
 * define PIN_DAC0, so derive it from the table exactly like the core does. */
inline bool pin_has_dac(uint8_t p) {
  if ((unsigned)p >= (unsigned)PINS_COUNT) {
    return false;
  }
  const PinDescription &d = g_APinDescription[p];
  return (d.ulPinAttribute & PIN_ATTR_ANALOG) != 0 &&
         (d.ulADCChannelNumber == ADC_Channel0 || d.ulADCChannelNumber == DAC_Channel0);
}

/* ======================================================================== */
#elif defined(ARDUINO_ARCH_RP2040)
/* UNVERIFIED: arduino-pico (Earle Philhower) core. Not compile-checked. */

#if defined(ARDUINO_NANO_RP2040_CONNECT)
static const uint16_t BoardId = USBIO_BOARD_NANO_RP2040_CONNECT;
#else
static const uint16_t BoardId = USBIO_BOARD_RP2040_GENERIC;
#endif
static const uint8_t AdcBits = 12;
static const uint8_t PwmBits = 12;
static const uint8_t DacBits = 0;
static const uint16_t VrefMv = 3300;
static const uint16_t IoMv = 3300;
static const bool HasPulldown = true;

inline uint8_t pin_count() { return named_pin_count(); }
inline bool pin_has_dio(uint8_t p) { return (unsigned)p < (unsigned)NUM_DIGITAL_PINS; }
inline bool pin_has_ain(uint8_t p) { return analog_header_pin(p); }
inline bool pin_has_pwm(uint8_t p) { return pin_has_dio(p); } /* every GPIO has a PWM slice */
inline bool pin_has_dac(uint8_t p) {
  (void)p;
  return false;
}

/* ======================================================================== */
#elif defined(ARDUINO_ARCH_ESP32)
/* UNVERIFIED: arduino-esp32 3.x. Not compile-checked. ADC pins are scattered,
 * so the contiguous PIN_A0 rule is replaced by the core's channel lookup. */

#if defined(ARDUINO_NANO_ESP32)
static const uint16_t BoardId = USBIO_BOARD_NANO_ESP32;
#else
static const uint16_t BoardId = USBIO_BOARD_ESP32_GENERIC;
#endif
static const uint8_t AdcBits = 12;
static const uint8_t PwmBits = 8; /* LEDC default resolution */
#if defined(DAC1)
static const uint8_t DacBits = 8;
#else
static const uint8_t DacBits = 0;
#endif
static const uint16_t VrefMv = 3300;
static const uint16_t IoMv = 3300;
static const bool HasPulldown = true;

inline uint8_t pin_count() { return named_pin_count(); }
inline bool pin_has_dio(uint8_t p) { return (unsigned)p < (unsigned)NUM_DIGITAL_PINS; }
inline bool pin_has_ain(uint8_t p) { return digitalPinToAnalogChannel(p) >= 0; }
inline bool pin_has_pwm(uint8_t p) { return pin_has_dio(p); }
inline bool pin_has_dac(uint8_t p) {
#if defined(DAC1) && defined(DAC2)
  return p == DAC1 || p == DAC2;
#else
  (void)p;
  return false;
#endif
}

#else
#error "UsbIo: unsupported architecture (mbed, renesas_uno, samd, rp2040, esp32)"
#endif

} // namespace UsbIoBoard

#endif /* USBIO_BOARD_H */
