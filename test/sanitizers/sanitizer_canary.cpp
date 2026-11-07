// Sanitizer canaries: each mode contains one deliberate bug. CTest registers a
// mode only for the matching sanitizer preset and passes only if the
// sanitizer's report appears in the output. This proves the preset really
// instruments the code: a misconfigured preset would silently run an
// uninstrumented build and every real test would stay green.
//
// Never linked into any library. Built only when WMM_SANITIZER is set.
#include <climits>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>

namespace {

// Values derive from argc so the compiler cannot prove the bug at compile time
// (which would turn it into a warning or fold it away).

int heapOverflow(int argc) {
  auto buffer = std::make_unique<int[]>(4);  // NOLINT(*-avoid-c-arrays)
  const int index = 3 + argc;                // argc >= 1, so index >= 4: one past the end
  return buffer[index];                      // NOLINT: deliberate out-of-bounds read
}

int leak(int argc) {
  auto* lost = new int[16];  // NOLINT: deliberately never freed
  lost[0] = argc;
  std::printf("leaked %p\n", static_cast<void*>(lost));
  return 0;
}

int signedOverflow(int argc) {
  volatile int value = INT_MAX - 1;  // NOLINT
  return value + argc + 1;           // INT_MAX - 1 + 2 overflows
}

int dataRace(int argc) {
  int shared = 0;  // written by two threads without synchronisation
  std::thread writer{[&shared, argc] { shared += argc; }};
  shared += argc;
  writer.join();
  return shared == 0 ? 1 : 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: sanitizer_canary heap-overflow|leak|signed-overflow|data-race\n");
    return 2;
  }
  const char* mode = argv[1];  // NOLINT(*-pointer-arithmetic)
  if (std::strcmp(mode, "heap-overflow") == 0) {
    return heapOverflow(argc - 1);
  }
  if (std::strcmp(mode, "leak") == 0) {
    return leak(argc - 1);
  }
  if (std::strcmp(mode, "signed-overflow") == 0) {
    return signedOverflow(argc - 1);
  }
  if (std::strcmp(mode, "data-race") == 0) {
    return dataRace(argc - 1);
  }
  std::fprintf(stderr, "unknown mode %s\n", mode);
  return 2;
}
