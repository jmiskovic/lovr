#include "gpu.h"
#include "core/gpu.h"
#include "core/util.h"
#include <string.h>
#include <stdlib.h>

static struct {
  bool initialized;
} state;

bool lovrGpuInit2() {
  if (state.initialized) return false;

  lovrAssert(gpu_init(), "Could not initialize gpu");

  return state.initialized = true;
}

void lovrGpuDestroy2() {
  if (!state.initialized) return;
  gpu_destroy();
  memset(&state, 0, sizeof(state));
}

void lovrGpuBeginFrame() {
  gpu_begin_frame();
}

void lovrGpuEndFrame() {
  gpu_end_frame();
}
