#include "gpu.h"
#include "core/gpu.h"
#include <string.h>

static struct {
  bool initialized;
} state;

bool lovrGpuInit2() {
  if (state.initialized) return false;
  if (!gpu_init()) {
    return false;
  }
  return state.initialized = true;
}

void lovrGpuDestroy2() {
  if (!state.initialized) return;
  memset(&state, 0, sizeof(state));
}
