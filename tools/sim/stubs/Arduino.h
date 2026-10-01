// Host stand-in for the Arduino core: just enough for Main/navigation.cpp to
// compile on a PC. Nothing here touches hardware.
#ifndef SIM_ARDUINO_H
#define SIM_ARDUINO_H
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <string>
#include <algorithm>

// Serial output is swallowed unless the simulator sets serialEcho (--verbose).
extern bool serialEcho;
struct SimSerial {
  template <class T> void print(const T &v) { if (serialEcho) out(v); }
  template <class T> void print(const T &v, int) { if (serialEcho) out(v); }
  template <class T> void println(const T &v) { if (serialEcho) { out(v); std::putchar('\n'); } }
  template <class T> void println(const T &v, int) { println(v); }
  void println() { if (serialEcho) std::putchar('\n'); }
private:
  void out(const char *s) { std::fputs(s, stdout); }
  void out(char *s) { std::fputs(s, stdout); }
  void out(const std::string &s) { std::fputs(s.c_str(), stdout); }
  void out(bool v) { std::printf("%d", v ? 1 : 0); }
  void out(char v) { std::putchar(v); }
  void out(int v) { std::printf("%d", v); }
  void out(long v) { std::printf("%ld", v); }
  void out(unsigned v) { std::printf("%u", v); }
  void out(unsigned long v) { std::printf("%lu", v); }
  void out(double v) { std::printf("%.2f", v); }
};
extern SimSerial Serial;
#endif
