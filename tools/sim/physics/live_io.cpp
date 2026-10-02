// Live mode helpers for physics.cpp (--live): wall clock, sleeping, and reading commands from stdin
// without waiting. Kept in their own file because <windows.h> clashes with the Arduino stand-in
// headers (INPUT, boolean, ...).
#include <chrono>
#include <string>
#ifdef _WIN32
#include <windows.h>
#else
#include <poll.h>
#include <unistd.h>
#endif

double liveWallMs() {
  using namespace std::chrono;
  static const steady_clock::time_point t0 = steady_clock::now();
  return duration<double, std::milli>(steady_clock::now() - t0).count();
}

void liveSleepMs(double ms) {
  if (ms <= 0) return;
#ifdef _WIN32
  Sleep((DWORD)(ms + 0.5));
#else
  usleep((useconds_t)(ms * 1000));
#endif
}

// whatever has arrived on stdin so far (the server writes one command per line)
std::string liveReadInput() {
  std::string out;
#ifdef _WIN32
  HANDLE h = GetStdHandle(STD_INPUT_HANDLE);
  DWORD avail = 0;
  if (!PeekNamedPipe(h, NULL, 0, NULL, &avail, NULL) || !avail) return out;
  out.resize(avail);
  DWORD got = 0;
  if (!ReadFile(h, &out[0], avail, &got, NULL)) got = 0;
  out.resize(got);
#else
  pollfd p = {0, POLLIN, 0};
  while (poll(&p, 1, 0) > 0 && (p.revents & POLLIN)) {
    char b[256];
    ssize_t n = read(0, b, sizeof b);
    if (n <= 0) break;
    out.append(b, (size_t)n);
  }
#endif
  return out;
}
