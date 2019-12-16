#include "gpu.h"
#define VK_NO_PROTOTYPES
#include <vulkan/vulkan.h>
#include <dlfcn.h>
#include <string.h>
#include <stdlib.h>

#define COUNTOF(x) (sizeof(x) / sizeof(x[0]))

// Functions that don't require an instance
#define GPU_FOREACH_ANONYMOUS(X)\
  X(vkCreateInstance)

// Functions that require an instance but don't require a device
#define GPU_FOREACH_INSTANCE(X)\
  X(vkDestroyInstance);\
  X(vkEnumeratePhysicalDevices);\
  X(vkGetPhysicalDeviceProperties);\
  X(vkGetPhysicalDeviceMemoryProperties);\
  X(vkGetPhysicalDeviceQueueFamilyProperties);\
  X(vkCreateDevice);\
  X(vkDestroyDevice);\
  X(vkGetDeviceQueue);\
  X(vkGetDeviceProcAddr)

// Functions that require a device
#define GPU_FOREACH_DEVICE(X)\
  X(vkQueueSubmit);\
  X(vkDeviceWaitIdle);\
  X(vkCreateCommandPool);\
  X(vkDestroyCommandPool);\
  X(vkAllocateCommandBuffers);\
  X(vkFreeCommandBuffers);\
  X(vkBeginCommandBuffer);\
  X(vkEndCommandBuffer);\
  X(vkCreateFence);\
  X(vkDestroyFence);\
  X(vkWaitForFences);\
  X(vkResetFences);\
  X(vkCreateBuffer);\
  X(vkDestroyBuffer);\
  X(vkGetBufferMemoryRequirements);\
  X(vkBindBufferMemory);\
  X(vkCreateImage);\
  X(vkDestroyImage);\
  X(vkGetImageMemoryRequirements);\
  X(vkBindImageMemory);\
  X(vkAllocateMemory);\
  X(vkFreeMemory)

// Used to load/declare Vulkan functions without lots of clutter
#define GPU_LOAD_ANONYMOUS(fn) fn = (PFN_##fn) vkGetInstanceProcAddr(NULL, #fn)
#define GPU_LOAD_INSTANCE(fn) fn = (PFN_##fn) vkGetInstanceProcAddr(state.instance, #fn)
#define GPU_LOAD_DEVICE(fn) fn = (PFN_##fn) vkGetDeviceProcAddr(state.device, #fn)
#define GPU_DECLARE(fn) static PFN_##fn fn

// Declare function pointers
GPU_FOREACH_ANONYMOUS(GPU_DECLARE);
GPU_FOREACH_INSTANCE(GPU_DECLARE);
GPU_FOREACH_DEVICE(GPU_DECLARE);

struct gpu_buffer {
  VkBuffer handle;
  VkDeviceMemory memory;
};

struct gpu_texture {
  VkImage handle;
  VkDeviceMemory memory;
};

typedef struct {
  VkObjectType type;
  void* handle;
} gpu_ref;

typedef struct {
  gpu_ref* data;
  size_t capacity;
  size_t length;
} gpu_freelist;

typedef struct {
  VkFence fence;
  VkCommandBuffer commandBuffer;
  gpu_freelist freelist;
} gpu_frame;

static struct {
  void* library;
  VkInstance instance;
  VkPhysicalDeviceMemoryProperties memoryProperties;
  uint32_t queueFamilyIndex;
  VkDevice device;
  VkQueue queue;
  VkCommandPool commandPool;
  VkCommandBuffer cmd;
  gpu_frame frames[2];
  uint32_t frame;
} state;

// Condemns an object, marking it for deletion (objects can't be destroyed while the GPU is still
// using them).  Condemned objects are purged in gpu_begin_frame after waiting on a fence.  We have
// to manage the freelist memory ourselves because the user could immediately free memory after
// destroying a resource.
// TODO currently there is a bug where condemning a resource outside of begin/end frame will
// immediately purge it on the next call to begin_frame (it should somehow get added to the previous
// frame's freelist, ugh, maybe advance frame index in begin_frame instead of end_frame)
// TODO even though this is fairly lightweight it might be worth doing some object tracking to see
// if you actually need to delay the destruction, and try to destroy it immediately when possible.
// Because Lua objects are GC'd we probably already have our own delay and could get away with
// skipping the freelist a lot of the time.  Also you might need to track object access anyway for
// barriers, so this could wombo combo with the condemnation.  It might be complicated though if you
// have to track access across multiple frames.
static void gpu_condemn(VkObjectType type, void* handle) {
  gpu_freelist* freelist = &state.frames[state.frame].freelist;

  if (freelist->length >= freelist->capacity) {
    freelist->capacity = freelist->capacity ? (freelist->capacity * 2) : 1;
    freelist->data = realloc(freelist->data, freelist->capacity * sizeof(*freelist->data));
    if (!freelist->data) {
      // OOM
    }
  }

  freelist->data[freelist->length++] = (gpu_ref) { type, handle };
}

// Purges any previously condemned objects for a frame.  Should only be called if the objects in the
// frame are known to no longer be accessed by the GPU.
static void gpu_purge(gpu_frame* frame) {
  for (size_t i = 0; i < frame->freelist.length; i++) {
    gpu_ref* ref = &frame->freelist.data[i];
    switch (ref->type) {
      case VK_OBJECT_TYPE_BUFFER: vkDestroyBuffer(state.device, ref->handle, NULL); break;
      case VK_OBJECT_TYPE_IMAGE: vkDestroyImage(state.device, ref->handle, NULL); break;
      case VK_OBJECT_TYPE_DEVICE_MEMORY: vkFreeMemory(state.device, ref->handle, NULL); break;
      default: /* Unreachable */ break;
    }
  }
  frame->freelist.length = 0;
}

bool gpu_init() {
  state.library = dlopen("libvulkan.so", RTLD_NOW | RTLD_LOCAL); // TODO cross platform
  PFN_vkGetInstanceProcAddr vkGetInstanceProcAddr = (PFN_vkGetInstanceProcAddr) dlsym(state.library, "vkGetInstanceProcAddr");

  // Load all the toplevel functions like vkCreateInstance
  GPU_FOREACH_ANONYMOUS(GPU_LOAD_ANONYMOUS);

  const char* layers[] = {
    "VK_LAYER_LUNARG_standard_validation",
    "VK_LAYER_LUNARG_object_tracker",
    "VK_LAYER_LUNARG_core_validation",
    "VK_LAYER_LUNARG_parameter_validation"
  };

  // Instance
  VkInstanceCreateInfo instanceInfo = {
    .sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
    .pApplicationInfo = &(VkApplicationInfo) {
      .sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
      .apiVersion = VK_MAKE_VERSION(1, 1, 0)
    },
    .enabledLayerCount = COUNTOF(layers),
    .ppEnabledLayerNames = layers
  };

  if (vkCreateInstance(&instanceInfo, NULL, &state.instance)) {
    goto hell;
  }

  // Load the instance functions
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

  vkGetPhysicalDeviceMemoryProperties(physicalDevice, &state.memoryProperties);

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

  // Load device functions
  GPU_FOREACH_DEVICE(GPU_LOAD_DEVICE);

  VkCommandPoolCreateInfo commandPoolInfo = {
    .sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
    .flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT | VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT,
    .queueFamilyIndex = state.queueFamilyIndex
  };

  if (vkCreateCommandPool(state.device, &commandPoolInfo, NULL, &state.commandPool)) {
    goto hell;
  }

  for (uint32_t i = 0; i < COUNTOF(state.frames); i++) {
    VkCommandBufferAllocateInfo commandBufferInfo = {
      .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
      .commandPool = state.commandPool,
      .level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
      .commandBufferCount = 1
    };

    if (vkAllocateCommandBuffers(state.device, &commandBufferInfo, &state.frames[i].commandBuffer)) {
      goto hell;
    }

    VkFenceCreateInfo fenceInfo = {
      .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO,
      .flags = VK_FENCE_CREATE_SIGNALED_BIT
    };

    if (vkCreateFence(state.device, &fenceInfo, NULL, &state.frames[i].fence)) {
      goto hell;
    }
  }

  return true;
hell:
  gpu_destroy();
  return false;
}

void gpu_destroy() {
  if (state.device) vkDeviceWaitIdle(state.device);
  for (uint32_t i = 0; i < COUNTOF(state.frames); i++) {
    gpu_frame* frame = &state.frames[i];
    gpu_purge(frame);
    if (frame->fence) vkDestroyFence(state.device, frame->fence, NULL);
    if (frame->commandBuffer) vkFreeCommandBuffers(state.device, state.commandPool, 1, &frame->commandBuffer);
    free(frame->freelist.data);
  }
  if (state.commandPool) vkDestroyCommandPool(state.device, state.commandPool, NULL);
  if (state.device) vkDestroyDevice(state.device, NULL);
  if (state.instance) vkDestroyInstance(state.instance, NULL);
  dlclose(state.library);
  memset(&state, 0, sizeof(state));
}

void gpu_begin_frame() {
  gpu_frame* frame = &state.frames[state.frame];

  VkCommandBufferBeginInfo beginfo = {
    .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
    .flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT
  };

  if (vkWaitForFences(state.device, 1, &frame->fence, VK_FALSE, ~0ull)) {
    // OOM
  }

  if (vkResetFences(state.device, 1, &frame->fence)) {
    // OOM
  }

  gpu_purge(frame);

  state.cmd = frame->commandBuffer;
  if (vkBeginCommandBuffer(state.cmd, &beginfo)) {
    // OOM
  }
}

void gpu_end_frame() {
  gpu_frame* frame = &state.frames[state.frame];

  if (vkEndCommandBuffer(frame->commandBuffer)) {
    // OOM
  }

  VkSubmitInfo submitInfo = {
    .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
    .pCommandBuffers = &frame->commandBuffer,
    .commandBufferCount = 1
  };

  if (vkQueueSubmit(state.queue, 1, &submitInfo, frame->fence)) {
    // OOM
  }

  state.frame = (state.frame + 1) % COUNTOF(state.frames);
}

size_t gpu_sizeof_buffer() {
  return sizeof(gpu_buffer);
}

bool gpu_buffer_init(gpu_buffer* buffer, gpu_buffer_info* info) {
  buffer->handle = VK_NULL_HANDLE;
  buffer->memory = VK_NULL_HANDLE;

  VkBufferUsageFlags usage =
    ((!info->usage || info->usage & GPU_BUFFER_USAGE_VERTEX) ? VK_BUFFER_USAGE_VERTEX_BUFFER_BIT : 0) |
    ((!info->usage || info->usage & GPU_BUFFER_USAGE_INDEX) ? VK_BUFFER_USAGE_INDEX_BUFFER_BIT : 0) |
    ((!info->usage || info->usage & GPU_BUFFER_USAGE_UNIFORM) ? VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT : 0) |
    ((!info->usage || info->usage & GPU_BUFFER_USAGE_COMPUTE) ? VK_BUFFER_USAGE_STORAGE_BUFFER_BIT : 0) |
    ((!info->usage || info->usage & GPU_BUFFER_USAGE_COPY_SRC) ? VK_BUFFER_USAGE_TRANSFER_SRC_BIT : 0) |
    ((!info->usage || info->usage & GPU_BUFFER_USAGE_COPY_DST) ? VK_BUFFER_USAGE_TRANSFER_DST_BIT : 0);

  VkBufferCreateInfo bufferInfo = {
    .sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,
    .size = info->size,
    .usage = usage
  };

  if (vkCreateBuffer(state.device, &bufferInfo, NULL, &buffer->handle)) {
    return false;
  }

  VkMemoryRequirements requirements;
  vkGetBufferMemoryRequirements(state.device, buffer->handle, &requirements);

  uint32_t type = ~0u;
  for (uint32_t i = 0; i < state.memoryProperties.memoryTypeCount; i++) {
    uint32_t flags = state.memoryProperties.memoryTypes[i].propertyFlags;
    if ((requirements.memoryTypeBits & (1 << i)) && (flags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)) {
      type = i;
      break;
    }
  }

  if (type == ~0u) {
    vkDestroyBuffer(state.device, buffer->handle, NULL);
    return false;
  }

  VkMemoryAllocateInfo memoryInfo = {
    .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
    .allocationSize = requirements.size,
    .memoryTypeIndex = type
  };

  if (vkAllocateMemory(state.device, &memoryInfo, NULL, &buffer->memory)) {
    vkDestroyBuffer(state.device, buffer->handle, NULL);
    return false;
  }

  if (vkBindBufferMemory(state.device, buffer->handle, buffer->memory, 0)) {
    vkDestroyBuffer(state.device, buffer->handle, NULL);
    vkFreeMemory(state.device, buffer->memory, NULL);
    return false;
  }

  return true;
}

void gpu_buffer_destroy(gpu_buffer* buffer) {
  if (buffer->handle) gpu_condemn(VK_OBJECT_TYPE_BUFFER, buffer->handle);
  if (buffer->memory) gpu_condemn(VK_OBJECT_TYPE_DEVICE_MEMORY, buffer->memory);
  buffer->handle = VK_NULL_HANDLE;
  buffer->memory = VK_NULL_HANDLE;
}

size_t gpu_sizeof_texture() {
  return sizeof(gpu_texture);
}

bool gpu_texture_init(gpu_texture* texture, gpu_texture_info* info) {
  texture->handle = VK_NULL_HANDLE;
  texture->memory = VK_NULL_HANDLE;

  VkImageType type;
  switch (info->type) {
    case GPU_TEXTURE_TYPE_2D: type = VK_IMAGE_TYPE_2D; break;
    case GPU_TEXTURE_TYPE_3D: type = VK_IMAGE_TYPE_3D; break;
    case GPU_TEXTURE_TYPE_CUBE: type = VK_IMAGE_TYPE_2D; break;
    case GPU_TEXTURE_TYPE_ARRAY: type = VK_IMAGE_TYPE_3D; break;
    default: return false;
  }

  VkFormat format;
  switch (info->format) {
    case GPU_TEXTURE_FORMAT_RGBA8: format = VK_FORMAT_R8G8B8A8_UNORM; break;
    default: return false;
  }

  VkImageCreateFlags flags =
    (info->type == GPU_TEXTURE_TYPE_CUBE ? VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT : 0) |
    (info->type == GPU_TEXTURE_TYPE_ARRAY ? VK_IMAGE_CREATE_2D_ARRAY_COMPATIBLE_BIT : 0);

  VkImageUsageFlags usage =
    ((info->usage & GPU_TEXTURE_USAGE_RENDER) ? VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT : 0) | // TODO check depth format
    ((info->usage & GPU_TEXTURE_USAGE_SAMPLER) ? VK_IMAGE_USAGE_SAMPLED_BIT : 0) |
    ((info->usage & GPU_TEXTURE_USAGE_COMPUTE) ? VK_IMAGE_USAGE_STORAGE_BIT : 0) |
    ((info->usage & GPU_TEXTURE_USAGE_COPY_SRC) ? VK_IMAGE_USAGE_TRANSFER_SRC_BIT : 0) |
    ((info->usage & GPU_TEXTURE_USAGE_COPY_DST) ? VK_IMAGE_USAGE_TRANSFER_DST_BIT : 0);

  VkImageCreateInfo imageInfo = {
    .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
    .flags = flags,
    .imageType = type,
    .format = format,
    .extent.width = info->size[0],
    .extent.height = info->size[1],
    .extent.depth = info->size[2],
    .mipLevels = info->mipmaps,
    .arrayLayers = info->layers,
    .samples = VK_SAMPLE_COUNT_1_BIT,
    .tiling = VK_IMAGE_TILING_OPTIMAL,
    .usage = usage
  };

  if (vkCreateImage(state.device, &imageInfo, NULL, &texture->handle)) {
    return false;
  }

  VkMemoryRequirements requirements;
  vkGetImageMemoryRequirements(state.device, texture->handle, &requirements);

  uint32_t memoryType = ~0u;
  for (uint32_t i = 0; i < state.memoryProperties.memoryTypeCount; i++) {
    uint32_t flags = state.memoryProperties.memoryTypes[i].propertyFlags;
    if ((requirements.memoryTypeBits & (1 << i)) && (flags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)) {
      memoryType = i;
      break;
    }
  }

  if (memoryType == ~0u) {
    vkDestroyImage(state.device, texture->handle, NULL);
    return false;
  }

  VkMemoryAllocateInfo memoryInfo = {
    .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
    .allocationSize = requirements.size,
    .memoryTypeIndex = memoryType
  };

  if (vkAllocateMemory(state.device, &memoryInfo, NULL, &texture->memory)) {
    vkDestroyImage(state.device, texture->handle, NULL);
    return false;
  }

  if (vkBindImageMemory(state.device, texture->handle, texture->memory, 0)) {
    vkDestroyImage(state.device, texture->handle, NULL);
    vkFreeMemory(state.device, texture->memory, NULL);
    return false;
  }

  return true;
}

void gpu_texture_destroy(gpu_texture* texture) {
  if (texture->handle) gpu_condemn(VK_OBJECT_TYPE_IMAGE, texture->handle);
  if (texture->memory) gpu_condemn(VK_OBJECT_TYPE_DEVICE_MEMORY, texture->memory);
  texture->handle = VK_NULL_HANDLE;
  texture->memory = VK_NULL_HANDLE;
}
