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

  luax_pushconf(L);
  lua_getfield(L, -1, "gpu");

  bool debug = false;
  if (lua_istable(L, -1)) {
    lua_getfield(L, -1, "debug");
    debug = lua_toboolean(L, -1);
    lua_pop(L, 1);
  }

  lua_pop(L, 2);

  if (lovrGpuInit2(debug)) {
    luax_atexit(L, lovrGpuDestroy2);
  }

  return 1;
}
