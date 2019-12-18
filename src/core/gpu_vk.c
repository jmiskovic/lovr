#include "gpu.h"
#define VK_NO_PROTOTYPES
#include <vulkan/vulkan.h>
#include <dlfcn.h>
#include <string.h>
#include <stdlib.h>

#define COUNTOF(x) (sizeof(x) / sizeof(x[0]))
#define SCRATCHPAD_SIZE (16 * 1024 * 1024)
#define VOIDP_TO_U64(x) (((union { uint64_t u; void* p; }) { .p = x }).u)
#define GPU_THROW(s) if (state.config.callback) { state.config.callback(state.config.context, s, true); }
#define GPU_CHECK(c, s) if (!(c)) { GPU_THROW(s); }
#define GPU_VK(f) do { VkResult r = (f); GPU_CHECK(r >= 0, getErrorString(r)); } while (0)

// Functions that don't require an instance
#define GPU_FOREACH_ANONYMOUS(X)\
  X(vkCreateInstance)

// Functions that require an instance but don't require a device
#define GPU_FOREACH_INSTANCE(X)\
  X(vkDestroyInstance);\
  X(vkCreateDebugUtilsMessengerEXT);\
  X(vkDestroyDebugUtilsMessengerEXT);\
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
  X(vkSetDebugUtilsObjectNameEXT);\
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
  X(vkCmdCopyBuffer);\
  X(vkCreateImage);\
  X(vkDestroyImage);\
  X(vkGetImageMemoryRequirements);\
  X(vkBindImageMemory);\
  X(vkCmdCopyBufferToImage);\
  X(vkAllocateMemory);\
  X(vkFreeMemory);\
  X(vkMapMemory);\
  X(vkUnmapMemory)

// Used to load/declare Vulkan functions without lots of clutter
#define GPU_LOAD_ANONYMOUS(fn) fn = (PFN_##fn) vkGetInstanceProcAddr(NULL, #fn)
#define GPU_LOAD_INSTANCE(fn) fn = (PFN_##fn) vkGetInstanceProcAddr(state.instance, #fn)
#define GPU_LOAD_DEVICE(fn) fn = (PFN_##fn) vkGetDeviceProcAddr(state.device, #fn)
#define GPU_DECLARE(fn) static PFN_##fn fn

// Declare function pointers
GPU_FOREACH_ANONYMOUS(GPU_DECLARE);
GPU_FOREACH_INSTANCE(GPU_DECLARE);
GPU_FOREACH_DEVICE(GPU_DECLARE);

typedef struct {
  uint16_t frame;
  uint16_t scratchpad;
  uint32_t offset;
} gpu_mapping;

struct gpu_buffer {
  VkBuffer handle;
  VkDeviceMemory memory;
  gpu_mapping mapping;
  uint64_t targetOffset;
  uint64_t targetSize;
};

struct gpu_texture {
  VkImage handle;
  VkDeviceMemory memory;
  VkImageLayout layout;
};

struct gpu_canvas {
  VkRenderPass renderPass;
  VkFramebuffer framebuffer;
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
  VkBuffer buffer;
  VkDeviceMemory memory;
  void* data;
} gpu_scratchpad;

typedef struct {
  VkFence fence;
  VkCommandBuffer commandBuffer;
  gpu_freelist freelist;
  struct gpu_pool {
    gpu_scratchpad* list;
    uint16_t count;
    uint16_t current;
    uint32_t cursor;
  } pool;
} gpu_frame;

static struct {
  gpu_config config;
  void* library;
  VkInstance instance;
  VkDebugUtilsMessengerEXT messenger;
  VkPhysicalDeviceMemoryProperties memoryProperties;
  VkMemoryRequirements scratchpadMemoryRequirements;
  uint32_t scratchpadMemoryType;
  uint32_t queueFamilyIndex;
  VkDevice device;
  VkQueue queue;
  VkCommandPool commandPool;
  VkCommandBuffer cmd;
  gpu_frame frames[2];
  uint32_t frame;
} state;

static uint32_t findMemoryType(uint32_t bits, uint32_t mask);
static void nickname(uint64_t object, VkObjectType type, const char* name);
static VkBool32 debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT severity, VkDebugUtilsMessageTypeFlagsEXT flags, const VkDebugUtilsMessengerCallbackDataEXT* data, void* context);
static const char* getErrorString(VkResult result);

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
    GPU_CHECK(freelist->data, "Out of memory");
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
      default: GPU_THROW("Unreachable"); break;
    }
  }
  frame->freelist.length = 0;
}

static uint8_t* gpu_map(uint64_t size, gpu_mapping* mapping) {
  struct gpu_pool* pool = &state.frames[state.frame].pool;
  gpu_scratchpad* scratchpad = &pool->list[pool->current];

  if (pool->count == 0 || pool->cursor + size > SCRATCHPAD_SIZE) {
    GPU_CHECK(size <= SCRATCHPAD_SIZE, "Tried to map too much memory");

    pool->cursor = 0;
    pool->current = pool->count++;
    pool->list = realloc(pool->list, pool->count * sizeof(gpu_scratchpad));
    GPU_CHECK(pool->list, "Out of memory");
    scratchpad = &pool->list[pool->current];

    // Create buffer
    VkBufferCreateInfo bufferInfo = {
      .sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,
      .size = SCRATCHPAD_SIZE,
      .usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT
    };

    GPU_VK(vkCreateBuffer(state.device, &bufferInfo, NULL, &scratchpad->buffer));

    // Get memory requirements, once
    if (state.scratchpadMemoryRequirements.size == 0) {
      uint32_t mask = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
      vkGetBufferMemoryRequirements(state.device, scratchpad->buffer, &state.scratchpadMemoryRequirements);
      state.scratchpadMemoryType = findMemoryType(state.scratchpadMemoryRequirements.memoryTypeBits, mask);

      if (state.scratchpadMemoryType == ~0u) {
        vkDestroyBuffer(state.device, scratchpad->buffer, NULL);
        return NULL; // PANIC
      }
    }

    // Allocate, bind
    VkMemoryAllocateInfo memoryInfo = {
      .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
      .allocationSize = state.scratchpadMemoryRequirements.size,
      .memoryTypeIndex = state.scratchpadMemoryType
    };

    if (vkAllocateMemory(state.device, &memoryInfo, NULL, &scratchpad->memory)) {
      vkDestroyBuffer(state.device, scratchpad->buffer, NULL);
      GPU_THROW("Out of memory");
    }

    if (vkBindBufferMemory(state.device, scratchpad->buffer, scratchpad->memory, 0)) {
      vkDestroyBuffer(state.device, scratchpad->buffer, NULL);
      vkFreeMemory(state.device, scratchpad->memory, NULL);
      GPU_THROW("Out of memory");
    }

    if (vkMapMemory(state.device, scratchpad->memory, 0, VK_WHOLE_SIZE, 0, &scratchpad->data)) {
      vkDestroyBuffer(state.device, scratchpad->buffer, NULL);
      vkFreeMemory(state.device, scratchpad->memory, NULL);
      GPU_THROW("Out of memory");
    }
  }

  return (uint8_t*) scratchpad->data + pool->cursor;
}

bool gpu_init(gpu_config* config) {
  state.config = *config;

  state.library = dlopen("libvulkan.so", RTLD_NOW | RTLD_LOCAL); // TODO cross platform
  PFN_vkGetInstanceProcAddr vkGetInstanceProcAddr = (PFN_vkGetInstanceProcAddr) dlsym(state.library, "vkGetInstanceProcAddr");

  // Load all the toplevel functions like vkCreateInstance
  GPU_FOREACH_ANONYMOUS(GPU_LOAD_ANONYMOUS);

  const char* layers[] = {
    "VK_LAYER_LUNARG_object_tracker",
    "VK_LAYER_LUNARG_core_validation",
    "VK_LAYER_LUNARG_parameter_validation"
  };

  const char* extensions[] = {
    "VK_EXT_debug_utils"
  };

  // Instance
  VkInstanceCreateInfo instanceInfo = {
    .sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
    .pApplicationInfo = &(VkApplicationInfo) {
      .sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
      .apiVersion = VK_MAKE_VERSION(1, 1, 0)
    },
    .enabledLayerCount = state.config.debug ? COUNTOF(layers) : 0,
    .ppEnabledLayerNames = layers,
    .enabledExtensionCount = state.config.debug ? COUNTOF(extensions) : 0,
    .ppEnabledExtensionNames = extensions
  };

  if (vkCreateInstance(&instanceInfo, NULL, &state.instance)) {
    goto hell;
  }

  // Load the instance functions
  GPU_FOREACH_INSTANCE(GPU_LOAD_INSTANCE);

  // Debug callbacks
  if (state.config.debug) {
    VkDebugUtilsMessengerCreateInfoEXT messengerInfo = {
      .sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT,
      .messageSeverity =
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT |
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT,
      .messageType =
        VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
        VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
        VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT,
      .pfnUserCallback = debugCallback,
      .pUserData = state.config.context
    };

    if (vkCreateDebugUtilsMessengerEXT(state.instance, &messengerInfo, NULL, &state.messenger)) {
      goto hell;
    }
  }

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
    free(frame->freelist.data);

    if (frame->fence) vkDestroyFence(state.device, frame->fence, NULL);
    if (frame->commandBuffer) vkFreeCommandBuffers(state.device, state.commandPool, 1, &frame->commandBuffer);

    for (uint32_t j = 0; j < frame->pool.count; j++) {
      gpu_scratchpad* scratchpad = &frame->pool.list[j];
      vkDestroyBuffer(state.device, scratchpad->buffer, NULL);
      vkUnmapMemory(state.device, scratchpad->memory);
      vkFreeMemory(state.device, scratchpad->memory, NULL);
    }

    free(frame->pool.list);
  }
  if (state.commandPool) vkDestroyCommandPool(state.device, state.commandPool, NULL);
  if (state.device) vkDestroyDevice(state.device, NULL);
  if (state.messenger) vkDestroyDebugUtilsMessengerEXT(state.instance, state.messenger, NULL);
  if (state.instance) vkDestroyInstance(state.instance, NULL);
  dlclose(state.library);
  memset(&state, 0, sizeof(state));
}

void gpu_begin_frame() {
  gpu_frame* frame = &state.frames[state.frame];

  // Wait for GPU to process the frame, then reset its scratchpad pool and purge condemned resources
  GPU_VK(vkWaitForFences(state.device, 1, &frame->fence, VK_FALSE, ~0ull));
  GPU_VK(vkResetFences(state.device, 1, &frame->fence));
  frame->pool.current = 0;
  frame->pool.cursor = 0;
  gpu_purge(frame);

  state.cmd = frame->commandBuffer;

  VkCommandBufferBeginInfo beginfo = {
    .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
    .flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT
  };

  GPU_VK(vkBeginCommandBuffer(state.cmd, &beginfo));
}

void gpu_end_frame() {
  gpu_frame* frame = &state.frames[state.frame];

  GPU_VK(vkEndCommandBuffer(frame->commandBuffer));

  VkSubmitInfo submitInfo = {
    .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
    .pCommandBuffers = &frame->commandBuffer,
    .commandBufferCount = 1
  };

  GPU_VK(vkQueueSubmit(state.queue, 1, &submitInfo, frame->fence));

  state.frame = (state.frame + 1) % COUNTOF(state.frames);
}

size_t gpu_sizeof_buffer() {
  return sizeof(gpu_buffer);
}

bool gpu_buffer_init(gpu_buffer* buffer, gpu_buffer_info* info) {
  memset(buffer, 0, sizeof(*buffer));

  VkBufferUsageFlags usage =
    ((!info->usage || info->usage & GPU_BUFFER_USAGE_VERTEX) ? VK_BUFFER_USAGE_VERTEX_BUFFER_BIT : 0) |
    ((!info->usage || info->usage & GPU_BUFFER_USAGE_INDEX) ? VK_BUFFER_USAGE_INDEX_BUFFER_BIT : 0) |
    ((!info->usage || info->usage & GPU_BUFFER_USAGE_UNIFORM) ? VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT : 0) |
    ((!info->usage || info->usage & GPU_BUFFER_USAGE_COMPUTE) ? VK_BUFFER_USAGE_STORAGE_BUFFER_BIT : 0) |
    ((!info->usage || info->usage & GPU_BUFFER_USAGE_COPY) ? VK_BUFFER_USAGE_TRANSFER_SRC_BIT : 0) |
    ((!info->usage || info->usage & GPU_BUFFER_USAGE_PASTE) ? VK_BUFFER_USAGE_TRANSFER_DST_BIT : 0);

  VkBufferCreateInfo bufferInfo = {
    .sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,
    .size = info->size,
    .usage = usage
  };

  if (vkCreateBuffer(state.device, &bufferInfo, NULL, &buffer->handle)) {
    return false;
  }

  nickname(VOIDP_TO_U64(buffer->handle), VK_OBJECT_TYPE_BUFFER, info->name);

  VkMemoryRequirements requirements;
  vkGetBufferMemoryRequirements(state.device, buffer->handle, &requirements);
  uint32_t memoryType = findMemoryType(requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

  if (memoryType == ~0u) {
    vkDestroyBuffer(state.device, buffer->handle, NULL);
    return false;
  }

  VkMemoryAllocateInfo memoryInfo = {
    .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
    .allocationSize = requirements.size,
    .memoryTypeIndex = memoryType
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
  if (buffer->handle) gpu_condemn(VK_OBJECT_TYPE_BUFFER, buffer->handle), buffer->handle = VK_NULL_HANDLE;
  if (buffer->memory) gpu_condemn(VK_OBJECT_TYPE_DEVICE_MEMORY, buffer->memory), buffer->memory = VK_NULL_HANDLE;
}

uint8_t* gpu_buffer_map(gpu_buffer* buffer, uint64_t offset, uint64_t size) {
  buffer->targetOffset = offset;
  buffer->targetSize = size;
  return gpu_map(size, &buffer->mapping);
}

void gpu_buffer_unmap(gpu_buffer* buffer) {
  if (buffer->mapping.frame != state.frame) {
    return;
  }

  VkBuffer source = state.frames[buffer->mapping.frame].pool.list[buffer->mapping.scratchpad].buffer;
  VkBuffer destination = buffer->handle;

  VkBufferCopy copy = {
    .srcOffset = buffer->mapping.offset,
    .dstOffset = buffer->targetOffset,
    .size = buffer->targetSize
  };

  vkCmdCopyBuffer(state.cmd, source, destination, 1, &copy);
}

size_t gpu_sizeof_texture() {
  return sizeof(gpu_texture);
}

bool gpu_texture_init(gpu_texture* texture, gpu_texture_info* info) {
  memset(texture, 0, sizeof(*texture));

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
    ((info->usage & GPU_TEXTURE_USAGE_COPY) ? VK_IMAGE_USAGE_TRANSFER_SRC_BIT : 0) |
    ((info->usage & GPU_TEXTURE_USAGE_PASTE) ? VK_IMAGE_USAGE_TRANSFER_DST_BIT : 0);

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

  nickname(VOIDP_TO_U64(texture->handle), VK_OBJECT_TYPE_IMAGE, info->name);

  VkMemoryRequirements requirements;
  vkGetImageMemoryRequirements(state.device, texture->handle, &requirements);
  uint32_t memoryType = findMemoryType(requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

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

  texture->layout = VK_IMAGE_LAYOUT_UNDEFINED;

  return true;
}

void gpu_texture_destroy(gpu_texture* texture) {
  if (texture->handle) gpu_condemn(VK_OBJECT_TYPE_IMAGE, texture->handle), texture->handle = VK_NULL_HANDLE;
  if (texture->memory) gpu_condemn(VK_OBJECT_TYPE_DEVICE_MEMORY, texture->memory), texture->memory = VK_NULL_HANDLE;
}

void gpu_texture_paste(gpu_texture* texture, uint8_t* data, uint64_t size, uint16_t offset[4], uint16_t extent[4], uint16_t mip) {
  if (size > SCRATCHPAD_SIZE) {
    // TODO loop or big boi staging buffer
    return;
  }

  gpu_mapping mapping;
  uint8_t* scratch = gpu_map(size, &mapping);
  memcpy(data, scratch, size);

  VkBuffer source = state.frames[mapping.frame].pool.list[mapping.scratchpad].buffer;
  VkImage destination = texture->handle;

  VkBufferImageCopy copy = {
    .bufferOffset = mapping.offset,
    .imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
    .imageSubresource.mipLevel = mip,
    .imageSubresource.baseArrayLayer = offset[3],
    .imageSubresource.layerCount = extent[3],
    .imageOffset.x = offset[0],
    .imageOffset.y = offset[1],
    .imageOffset.z = offset[2],
    .imageExtent.width = extent[0],
    .imageExtent.height = extent[1],
    .imageExtent.depth = extent[2]
  };

  // TODO layout transition (barrier)

  vkCmdCopyBufferToImage(state.cmd, source, destination, texture->layout, 1, &copy);
}

size_t gpu_sizeof_canvas() {
  return sizeof(gpu_canvas);
}

bool gpu_canvas_init(gpu_canvas* canvas, gpu_canvas_info* info) {
  return false;
}

void gpu_canvas_destroy(gpu_canvas* canvas) {
  //
}

static uint32_t findMemoryType(uint32_t bits, uint32_t mask) {
  for (uint32_t i = 0; i < state.memoryProperties.memoryTypeCount; i++) {
    uint32_t flags = state.memoryProperties.memoryTypes[i].propertyFlags;
    if ((flags & mask) == mask && (bits & (1 << i))) {
      return i;
    }
  }
  return ~0u;
}

static void nickname(uint64_t handle, VkObjectType type, const char* name) {
  if (state.config.debug && name) {
    VkDebugUtilsObjectNameInfoEXT info = {
      .sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_OBJECT_NAME_INFO_EXT,
      .objectType = type,
      .objectHandle = handle,
      .pObjectName = name
    };

    GPU_VK(vkSetDebugUtilsObjectNameEXT(state.device, &info));
  }
}

static VkBool32 debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT severity, VkDebugUtilsMessageTypeFlagsEXT flags, const VkDebugUtilsMessengerCallbackDataEXT* data, void* context) {
  if (state.config.callback) {
    bool severe = severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    state.config.callback(state.config.context, data->pMessage, severe);
  }

  return VK_FALSE;
}

static const char* getErrorString(VkResult result) {
  switch (result) {
    case VK_ERROR_OUT_OF_HOST_MEMORY: return "Out of CPU memory";
    case VK_ERROR_OUT_OF_DEVICE_MEMORY: return "Out of GPU memory";
    case VK_ERROR_MEMORY_MAP_FAILED: return "Could not map memory";
    case VK_ERROR_DEVICE_LOST: return "Lost connection to GPU";
    case VK_ERROR_TOO_MANY_OBJECTS: return "Too many objects";
    case VK_ERROR_FORMAT_NOT_SUPPORTED: return "Unsupported format";
    default: return NULL;
  }
}
