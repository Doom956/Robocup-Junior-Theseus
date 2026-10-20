// Waveshare 2inch LCD Module (ST7789V, 240x320 IPS, SPI) used as a big 16x2 text display, in place of the 16x2
// character LCD. Same calls as LiquidCrystal (begin, setCursor, print), so display.cpp and the victim code's lcdPrint()
// work unchanged.
//
// The ST7789 is driven by software SPI on the old LCD pins (the hardware SPI pins D10-D13 are the stepper's), from its
// own thread below normal priority: print() only writes into a text buffer (microseconds) and the thread draws what
// changed. Navigation, the camera thread and the pause thread never wait for the screen.
//
// Wiring (Waveshare cable colour -> GIGA pin):
//   VCC red -> 3.3V or 5V   GND black -> GND
//   DIN green -> 23   CLK orange -> 53   CS yellow -> 29   DC blue -> 31   RST brown -> 27   BL grey -> 25
#pragma once
#include <Arduino.h>

// Display: 1 = this screen, 0 = the 16x2 character LCD (LiquidCrystal on the same pins). Change it here only.
// The simulator always builds the LCD version (it reads the end-of-run text from it).
#define DISPLAY_TFT 1
#if DISPLAY_TFT && defined(ARDUINO_ARCH_MBED)
#define USE_TFT 1
#else
#define USE_TFT 0
#endif

#define TFT_PIN_DIN 23
#define TFT_PIN_CLK 53
#define TFT_PIN_CS  29
#define TFT_PIN_DC  31
#define TFT_PIN_RST 27
#define TFT_PIN_BL  25
// Landscape, one way or the other: 1 or 3 (text upside down -> use the other one).
#define TFT_ROTATION 1

class TftText {
 public:
  void begin(uint8_t cols, uint8_t rows);
  void setCursor(uint8_t col, uint8_t row);
  size_t print(const char *text);
  size_t print(const String &t) { return print(t.c_str()); }
};
