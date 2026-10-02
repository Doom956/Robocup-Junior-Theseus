// Simulated Arduino core for the physics simulator. Time is simulated: millis(),
// micros() and delay() run on the simulator's clock, and every call that would talk
// to hardware advances that clock by roughly what it costs on the robot.
#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <string>
#include <algorithm>
#include "mbed.h" // the GIGA's Arduino core makes rtos:: visible everywhere, so the robot code relies on it
typedef bool boolean;
typedef uint8_t byte;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 1
#define FALLING 2
#define RISING 3
#define HEX 16
#define DEC 10
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define digitalPinToInterrupt(p) (p)
using std::abs;
using std::max;
using std::min;

unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
void pinMode(int pin, int mode);
int digitalRead(int pin);
void digitalWrite(int pin, int value);
void attachInterrupt(int irq, void (*isr)(), int mode);
void detachInterrupt(int irq);

class String {
 public:
  std::string s;
  String() {}
  String(const char *c) : s(c) {}
  String(const std::string &c) : s(c) {}
  String(int v) : s(std::to_string(v)) {}
  String(unsigned v) : s(std::to_string(v)) {}
  String(long v) : s(std::to_string(v)) {}
  String(double v) : s(std::to_string(v)) {}
  const char *c_str() const { return s.c_str(); }
  String operator+(const String &o) const { return String(s + o.s); }
  String operator+(const char *o) const { return String(s + o); }
};
inline String operator+(const char *a, const String &b) { return String(std::string(a) + b.s); }

// Serial output goes to stdout only with --verbose. Serial3/Serial4 (victim cameras) never receive anything.
extern bool simSerialEcho;
class SimSerial {
 public:
  void begin(unsigned long) {}
  int available() { return 0; }
  int read() { return -1; }
  template <class T> void print(T v) { if (simSerialEcho) out(v); }
  template <class T> void print(T v, int) { if (simSerialEcho) out(v); }
  template <class T> void println(T v) { if (simSerialEcho) { out(v); std::putchar('\n'); } }
  template <class T> void println(T v, int) { println(v); }
  void println() { if (simSerialEcho) std::putchar('\n'); }
 private:
  void out(const char *v) { std::fputs(v, stdout); }
  void out(char *v) { std::fputs(v, stdout); }
  void out(const String &v) { std::fputs(v.c_str(), stdout); }
  void out(bool v) { std::printf("%d", v ? 1 : 0); }
  void out(char v) { std::putchar(v); }
  void out(int v) { std::printf("%d", v); }
  void out(unsigned v) { std::printf("%u", v); }
  void out(long v) { std::printf("%ld", v); }
  void out(unsigned long v) { std::printf("%lu", v); }
  void out(float v) { std::printf("%.2f", v); }
  void out(double v) { std::printf("%.2f", v); }
};
extern SimSerial Serial, Serial3, Serial4;
