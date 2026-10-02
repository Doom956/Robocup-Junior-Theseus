#pragma once
#include <chrono>
// Threads are not run in the simulator: the victim-camera thread has nothing to read, and
// the pause-switch thread is emulated by the simulator itself (it sets Pausemaze).
namespace rtos {
struct Mutex { void lock() {} void unlock() {} };
namespace ThisThread { void sleep_for(std::chrono::milliseconds ms); }
struct Thread {
  template <class F> int start(F) { return 0; }
  int set_priority(int) { return 0; }
};
}
enum { osPriorityNormal = 24, osPriorityAboveNormal = 32 };
