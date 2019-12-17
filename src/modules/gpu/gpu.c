#include "gpu.h"
#include "core/gpu.h"
#include "core/util.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static struct {
  bool initialized;
} state;

void callback(void* context, const char* message, bool severe) {
  if (severe) {
    lovrThrow("GPU Error: %s", message);
  } else {
    fprintf(stdout, "%s\n", message);
  }
}

bool lovrGpuInit2(bool debug) {
  if (state.initialized) return false;

  gpu_config config = {
    .debug = debug,
    .callback = callback,
    .context = NULL
  };

  lovrAssert(gpu_init(&config), "Could not initialize gpu");

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
