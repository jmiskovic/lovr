#include "api.h"
#include "modules/gpu/gpu.h"

static int l_lovrGpuBeginFrame() {
  lovrGpuBeginFrame();
  return 0;
}

static int l_lovrGpuEndFrame() {
  lovrGpuEndFrame();
  return 0;
}

static const luaL_Reg lovrGpu[] = {
  { "beginFrame", l_lovrGpuBeginFrame },
  { "endFrame", l_lovrGpuEndFrame },
  { NULL, NULL }
};

int luaopen_lovr_gpu(lua_State* L) {
  lua_newtable(L);
  luaL_register(L, NULL, lovrGpu);
  if (lovrGpuInit2()) {
    luax_atexit(L, lovrGpuDestroy2);
  }
  return 1;
}
