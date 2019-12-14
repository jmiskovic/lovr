#include "api.h"
#include "modules/gpu/gpu.h"

int luaopen_lovr_gpu(lua_State* L) {
  lua_newtable(L);
  if (lovrGpuInit2()) {
    luax_atexit(L, lovrGpuDestroy2);
  }
  return 1;
}
