#include "gpu.h"
#define VK_NO_PROTOTYPES
#include <vulkan/vulkan.h>
#include <stdio.h>
#include <dlfcn.h>

#define COUNTOF(x) (sizeof(x) / sizeof(x[0]))

#define GPU_FOREACH_ANONYMOUS(X)\
  X(vkCreateInstance)
#define GPU_FOREACH_INSTANCE(X)\
  X(vkDestroyInstance);\
  X(vkEnumeratePhysicalDevices);\
  X(vkGetPhysicalDeviceProperties);\
  X(vkGetPhysicalDeviceQueueFamilyProperties);\
  X(vkCreateDevice);\
  X(vkDestroyDevice);\
  X(vkGetDeviceQueue)

#define GPU_DECL(fn) PFN_##fn fn
#define GPU_LOAD_INSTANCE(fn) fn = (PFN_##fn) vkGetInstanceProcAddr(state.instance, #fn)

GPU_FOREACH_ANONYMOUS(GPU_DECL);
GPU_FOREACH_INSTANCE(GPU_DECL);

static struct {
  void* library;
  VkInstance instance;
  uint32_t queueFamilyIndex;
  VkDevice device;
  VkQueue queue;
} state;

bool gpu_init() {
  state.library = dlopen("libvulkan.so", RTLD_NOW | RTLD_LOCAL);
  PFN_vkGetInstanceProcAddr vkGetInstanceProcAddr = (PFN_vkGetInstanceProcAddr) dlsym(state.library, "vkGetInstanceProcAddr");

  GPU_FOREACH_ANONYMOUS(GPU_LOAD_INSTANCE);

  // Instance
  VkInstanceCreateInfo instanceInfo = {
    .sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
    .pApplicationInfo = &(VkApplicationInfo) {
      .sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
      .apiVersion = VK_MAKE_VERSION(1, 1, 0)
    }
  };

  if (vkCreateInstance(&instanceInfo, NULL, &state.instance)) {
    goto hell;
  }

  GPU_FOREACH_INSTANCE(GPU_LOAD_INSTANCE);

  // Device
  uint32_t deviceCount = 1;
  VkPhysicalDevice physicalDevice;
  if (vkEnumeratePhysicalDevices(state.instance, &deviceCount, &physicalDevice) < 0) {
    goto hell;
  }

  VkPhysicalDeviceProperties deviceProperties;
  vkGetPhysicalDeviceProperties(physicalDevice, &deviceProperties);

  // Find a queue family
  state.queueFamilyIndex = ~0u;
  VkQueueFamilyProperties queueFamilies[4];
  uint32_t queueFamilyCount = COUNTOF(queueFamilies);
  vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueFamilyCount, queueFamilies);

  for (uint32_t i = 0; i < queueFamilyCount; i++) {
    uint32_t flags = queueFamilies[i].queueFlags;
    if ((flags & VK_QUEUE_GRAPHICS_BIT) && (flags & VK_QUEUE_COMPUTE_BIT)) {
      state.queueFamilyIndex = i;
      break;
    }
  }

  if (state.queueFamilyIndex == ~0u) {
    goto hell;
  }

  VkDeviceQueueCreateInfo queueInfo = {
    .sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO,
    .queueFamilyIndex = state.queueFamilyIndex,
    .queueCount = 1,
    .pQueuePriorities = &(float) { 1.f }
  };

  VkDeviceCreateInfo deviceInfo = {
    .sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO,
    .queueCreateInfoCount = 1,
    .pQueueCreateInfos = &queueInfo
  };

  if (vkCreateDevice(physicalDevice, &deviceInfo, NULL, &state.device)) {
    goto hell;
  }

  vkGetDeviceQueue(state.device, state.queueFamilyIndex, 0, &state.queue);

  return true;
hell:
  gpu_destroy();
  return false;
}

void gpu_destroy() {
  if (state.instance) vkDestroyInstance(state.instance, NULL);
  if (state.device) vkDestroyDevice(state.device, NULL);
  dlclose(state.library);
}
