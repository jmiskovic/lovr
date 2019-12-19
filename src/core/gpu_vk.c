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
  X(vkCmdPipelineBarrier);\
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
  X(vkUnmapMemory);\
  X(vkCreateSampler);\
  X(vkDestroySampler);\
  X(vkCreateRenderPass);\
  X(vkDestroyRenderPass);\
  X(vkCreateImageView);\
  X(vkDestroyImageView);\
  X(vkCreateFramebuffer);\
  X(vkDestroyFramebuffer);\
  X(vkCreateShaderModule);\
  X(vkDestroyShaderModule)

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
  VkImageLayout layout;
  VkDeviceMemory memory;
  VkImageView view;
  gpu_texture* source;
  gpu_texture_type type;
  VkFormat format;
  VkImageAspectFlagBits aspect;
};

struct gpu_sampler {
  VkSampler handle;
};

struct gpu_canvas {
  VkRenderPass handle;
  VkFramebuffer framebuffer;
};

struct gpu_shader {
  VkShaderModule handles[2];
  VkPipelineShaderStageCreateInfo pipelineInfo[2];
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
  gpu_scratchpad* list;
  uint16_t count;
  uint16_t current;
  uint32_t cursor;
} gpu_pool;

typedef struct {
  VkFence fence;
  VkCommandBuffer commandBuffer;
  gpu_freelist freelist;
  gpu_pool pool;
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
  VkCommandBuffer commandBuffer;
  gpu_frame frames[2];
  uint32_t frame;
} state;

static uint32_t findMemoryType(uint32_t bits, uint32_t mask);
static VkFormat convertTextureFormat(gpu_texture_format format);
static bool isDepthFormat(gpu_texture_format format);
static VkSamplerAddressMode convertWrap(gpu_wrap wrap);
static VkAttachmentLoadOp convertLoadOp(bool load, bool clear);
static bool loadShader(gpu_shader_source* source, VkShaderStageFlagBits stage, VkShaderModule* handle, VkPipelineShaderStageCreateInfo* pipelineInfo);
static void setLayout(gpu_texture* texture, VkImageLayout layout, VkPipelineStageFlags nextStages, VkAccessFlags nextActions);
static void nicknameObject(uint64_t object, VkObjectType type, const char* nickname);
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
      case VK_OBJECT_TYPE_IMAGE_VIEW: vkDestroyImageView(state.device, ref->handle, NULL); break;
      case VK_OBJECT_TYPE_SAMPLER: vkDestroySampler(state.device, ref->handle, NULL); break;
      case VK_OBJECT_TYPE_RENDER_PASS: vkDestroyRenderPass(state.device, ref->handle, NULL); break;
      case VK_OBJECT_TYPE_FRAMEBUFFER: vkDestroyFramebuffer(state.device, ref->handle, NULL); break;
      default: GPU_THROW("Unreachable"); break;
    }
  }
  frame->freelist.length = 0;
}

static uint8_t* gpu_map(uint64_t size, gpu_mapping* mapping) {
  gpu_pool* pool = &state.frames[state.frame].pool;
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

  mapping->frame = state.frame;
  mapping->offset = pool->cursor;
  mapping->scratchpad = pool->current;
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

  state.commandBuffer = frame->commandBuffer;

  VkCommandBufferBeginInfo beginfo = {
    .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
    .flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT
  };

  GPU_VK(vkBeginCommandBuffer(state.commandBuffer, &beginfo));
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

  nicknameObject(VOIDP_TO_U64(buffer->handle), VK_OBJECT_TYPE_BUFFER, info->nickname);

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

  vkCmdCopyBuffer(state.commandBuffer, source, destination, 1, &copy);
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

  bool depth = isDepthFormat(info->format);
  texture->format = convertTextureFormat(info->format);
  texture->aspect = depth ? (VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT) : VK_IMAGE_ASPECT_COLOR_BIT;

  VkImageCreateFlags flags =
    (info->type == GPU_TEXTURE_TYPE_CUBE ? VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT : 0) |
    (info->type == GPU_TEXTURE_TYPE_ARRAY ? VK_IMAGE_CREATE_2D_ARRAY_COMPATIBLE_BIT : 0);

  VkImageUsageFlags usage =
    (((info->usage & GPU_TEXTURE_USAGE_RENDER) && !depth) ? VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT : 0) |
    (((info->usage & GPU_TEXTURE_USAGE_RENDER) && depth) ? VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT : 0) |
    ((info->usage & GPU_TEXTURE_USAGE_SAMPLER) ? VK_IMAGE_USAGE_SAMPLED_BIT : 0) |
    ((info->usage & GPU_TEXTURE_USAGE_COMPUTE) ? VK_IMAGE_USAGE_STORAGE_BIT : 0) |
    ((info->usage & GPU_TEXTURE_USAGE_COPY) ? VK_IMAGE_USAGE_TRANSFER_SRC_BIT : 0) |
    ((info->usage & GPU_TEXTURE_USAGE_PASTE) ? VK_IMAGE_USAGE_TRANSFER_DST_BIT : 0);

  VkImageCreateInfo imageInfo = {
    .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
    .flags = flags,
    .imageType = type,
    .format = texture->format,
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

  nicknameObject(VOIDP_TO_U64(texture->handle), VK_OBJECT_TYPE_IMAGE, info->nickname);

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

  if (!gpu_texture_init_view(texture, texture, NULL)) {
    vkDestroyImage(state.device, texture->handle, NULL);
    vkFreeMemory(state.device, texture->memory, NULL);
    return false;
  }

  return true;
}

bool gpu_texture_init_view(gpu_texture* texture, gpu_texture* source, gpu_texture_view_info* info) {
  if (texture != source) {
    texture->handle = VK_NULL_HANDLE;
    texture->memory = VK_NULL_HANDLE;
    texture->layout = VK_IMAGE_LAYOUT_UNDEFINED;
    texture->source = source;
    texture->type = info ? info->type : source->type;
    texture->format = (info && info->format) ? convertTextureFormat(info->format) : source->format;
    texture->aspect = source->aspect;
  }

  VkImageViewType type;
  switch (texture->type) {
    case GPU_TEXTURE_TYPE_2D: type = VK_IMAGE_VIEW_TYPE_2D; break;
    case GPU_TEXTURE_TYPE_3D: type = VK_IMAGE_VIEW_TYPE_3D; break;
    case GPU_TEXTURE_TYPE_CUBE: type = VK_IMAGE_VIEW_TYPE_CUBE; break;
    case GPU_TEXTURE_TYPE_ARRAY: type = VK_IMAGE_VIEW_TYPE_2D_ARRAY; break;
    default: return false;
  }

  VkImageViewCreateInfo createInfo = {
    .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
    .image = source->handle,
    .viewType = type,
    .format = texture->format,
    .subresourceRange = {
      .aspectMask = texture->aspect,
      .baseMipLevel = info ? info->baseMipmap : 0,
      .levelCount = (info && info->mipmapCount) ? info->mipmapCount : VK_REMAINING_MIP_LEVELS,
      .baseArrayLayer = info ? info->baseLayer : 0,
      .layerCount = (info && info->layerCount) ? info->layerCount : VK_REMAINING_ARRAY_LAYERS
    }
  };

  GPU_VK(vkCreateImageView(state.device, &createInfo, NULL, &texture->view));

  return true;
}

void gpu_texture_destroy(gpu_texture* texture) {
  if (texture->handle) gpu_condemn(VK_OBJECT_TYPE_IMAGE, texture->handle), texture->handle = VK_NULL_HANDLE;
  if (texture->memory) gpu_condemn(VK_OBJECT_TYPE_DEVICE_MEMORY, texture->memory), texture->memory = VK_NULL_HANDLE;
  if (texture->view) gpu_condemn(VK_OBJECT_TYPE_IMAGE_VIEW, texture->view), texture->view = VK_NULL_HANDLE;
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
    .imageSubresource.aspectMask = texture->aspect,
    .imageSubresource.mipLevel = mip,
    .imageSubresource.baseArrayLayer = offset[3],
    .imageSubresource.layerCount = extent[3],
    .imageOffset = { offset[0], offset[1], offset[2] },
    .imageExtent = { extent[0], extent[1], extent[2] }
  };

  setLayout(texture, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT);

  vkCmdCopyBufferToImage(state.commandBuffer, source, destination, texture->layout, 1, &copy);
}

size_t gpu_sizeof_sampler() {
  return sizeof(gpu_sampler);
}

bool gpu_sampler_init(gpu_sampler* sampler, gpu_sampler_info* info) {
  VkSamplerCreateInfo createInfo = {
    .sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO,
    .magFilter = info->mag == GPU_FILTER_NEAREST ? VK_FILTER_NEAREST : VK_FILTER_LINEAR,
    .minFilter = info->min == GPU_FILTER_NEAREST ? VK_FILTER_NEAREST : VK_FILTER_LINEAR,
    .mipmapMode = info->mip == GPU_FILTER_NEAREST ? VK_SAMPLER_MIPMAP_MODE_NEAREST : VK_SAMPLER_MIPMAP_MODE_LINEAR,
    .addressModeU = convertWrap(info->wrapu),
    .addressModeV = convertWrap(info->wrapv),
    .addressModeW = convertWrap(info->wrapw),
    .anisotropyEnable = info->anisotropy > 0.f,
    .maxAnisotropy = info->anisotropy
  };

  GPU_VK(vkCreateSampler(state.device, &createInfo, NULL, &sampler->handle));

  return true;
}

void gpu_sampler_destroy(gpu_sampler* sampler) {
  if (sampler->handle) gpu_condemn(VK_OBJECT_TYPE_SAMPLER, sampler->handle), sampler->handle = VK_NULL_HANDLE;
}

size_t gpu_sizeof_canvas() {
  return sizeof(gpu_canvas);
}

bool gpu_canvas_init(gpu_canvas* canvas, gpu_canvas_info* info) {
  VkAttachmentDescription attachments[5];
  VkAttachmentReference references[5];
  VkImageView imageViews[5];
  uint32_t colorCount = 0;
  uint32_t totalCount = 0;

  for (uint32_t i = 0; i < COUNTOF(info->color) && info->color[i].texture; i++, colorCount++, totalCount++) {
    attachments[i] = (VkAttachmentDescription) {
      .format = info->color[i].texture->format,
      .samples = VK_SAMPLE_COUNT_1_BIT,
      .loadOp = convertLoadOp(info->color[i].load, info->color[i].clear),
      .storeOp = info->color[i].save ? VK_ATTACHMENT_STORE_OP_STORE : VK_ATTACHMENT_STORE_OP_DONT_CARE,
      .initialLayout = VK_IMAGE_LAYOUT_GENERAL,
      .finalLayout = VK_IMAGE_LAYOUT_GENERAL
    };

    references[i] = (VkAttachmentReference) {
      .attachment = i,
      .layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL
    };

    imageViews[i] = info->color[i].texture->view;
  }

  if (info->depthStencil.texture) {
    uint32_t i = totalCount++;

    attachments[i] = (VkAttachmentDescription) {
      .format = info->depthStencil.texture->format,
      .samples = VK_SAMPLE_COUNT_1_BIT,
      .loadOp = convertLoadOp(info->depthStencil.depth.load, info->depthStencil.depth.clear),
      .storeOp = info->depthStencil.depth.save ? VK_ATTACHMENT_STORE_OP_STORE : VK_ATTACHMENT_STORE_OP_DONT_CARE,
      .stencilLoadOp = convertLoadOp(info->depthStencil.stencil.load, info->depthStencil.stencil.clear),
      .stencilStoreOp = info->depthStencil.depth.save ? VK_ATTACHMENT_STORE_OP_STORE : VK_ATTACHMENT_STORE_OP_DONT_CARE,
      .initialLayout = VK_IMAGE_LAYOUT_GENERAL,
      .finalLayout = VK_IMAGE_LAYOUT_GENERAL
    };

    references[i] = (VkAttachmentReference) {
      .attachment = i,
      .layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL
    };

    imageViews[i] = info->depthStencil.texture->view;
  }

  VkSubpassDescription subpass = {
    .colorAttachmentCount = colorCount,
    .pColorAttachments = references,
    .pDepthStencilAttachment = info->depthStencil.texture ? &references[colorCount] : NULL
  };

  VkRenderPassCreateInfo renderPassInfo = {
    .sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO,
    .attachmentCount = totalCount,
    .pAttachments = attachments,
    .subpassCount = 1,
    .pSubpasses = &subpass
  };

  if (vkCreateRenderPass(state.device, &renderPassInfo, NULL, &canvas->handle)) {
    return false;
  }

  VkFramebufferCreateInfo framebufferInfo = {
    .sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO,
    .renderPass = canvas->handle,
    .attachmentCount = totalCount,
    .pAttachments = imageViews,
    .width = 0, // TODO
    .height = 0,
    .layers = 1
  };

  if (vkCreateFramebuffer(state.device, &framebufferInfo, NULL, &canvas->framebuffer)) {
    vkDestroyRenderPass(state.device, canvas->handle, NULL);
  }

  return false;
}

void gpu_canvas_destroy(gpu_canvas* canvas) {
  if (canvas->handle) gpu_condemn(VK_OBJECT_TYPE_RENDER_PASS, canvas->handle), canvas->handle = VK_NULL_HANDLE;
  if (canvas->framebuffer) gpu_condemn(VK_OBJECT_TYPE_FRAMEBUFFER, canvas->framebuffer), canvas->framebuffer = VK_NULL_HANDLE;
}

size_t gpu_sizeof_shader() {
  return sizeof(gpu_shader);
}

bool gpu_shader_init(gpu_shader* shader, gpu_shader_info* info) {
  memset(shader, 0, sizeof(*shader));

  if (info->compute.code) {
    loadShader(&info->compute, VK_SHADER_STAGE_COMPUTE_BIT, &shader->handles[0], &shader->pipelineInfo[0]);
  } else {
    loadShader(&info->vertex, VK_SHADER_STAGE_VERTEX_BIT, &shader->handles[0], &shader->pipelineInfo[0]);
    loadShader(&info->fragment, VK_SHADER_STAGE_FRAGMENT_BIT, &shader->handles[1], &shader->pipelineInfo[1]);
  }

  return true;
}

void gpu_shader_destroy(gpu_shader* shader) {
  if (shader->handles[0]) gpu_condemn(VK_OBJECT_TYPE_SHADER_MODULE, shader->handles[0]), shader->handles[0] = VK_NULL_HANDLE;
  if (shader->handles[1]) gpu_condemn(VK_OBJECT_TYPE_SHADER_MODULE, shader->handles[1]), shader->handles[1] = VK_NULL_HANDLE;
}

// Helpers

static uint32_t findMemoryType(uint32_t bits, uint32_t mask) {
  for (uint32_t i = 0; i < state.memoryProperties.memoryTypeCount; i++) {
    uint32_t flags = state.memoryProperties.memoryTypes[i].propertyFlags;
    if ((flags & mask) == mask && (bits & (1 << i))) {
      return i;
    }
  }
  return ~0u;
}

static VkFormat convertTextureFormat(gpu_texture_format format) {
  switch (format) {
    case GPU_TEXTURE_FORMAT_RGBA8: return VK_FORMAT_R8G8B8A8_UNORM;
    default: return VK_FORMAT_UNDEFINED;
  }
}

static bool isDepthFormat(gpu_texture_format format) {
  switch (format) {
    default: return false;
  }
}

static VkSamplerAddressMode convertWrap(gpu_wrap wrap) {
  switch (wrap) {
    case GPU_WRAP_CLAMP: return VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    case GPU_WRAP_REPEAT: return VK_SAMPLER_ADDRESS_MODE_REPEAT;
    case GPU_WRAP_MIRROR: return VK_SAMPLER_ADDRESS_MODE_MIRRORED_REPEAT;
    default: GPU_THROW("Unreachable");
  }
}

static VkAttachmentLoadOp convertLoadOp(bool load, bool clear) {
  if (clear) return VK_ATTACHMENT_LOAD_OP_CLEAR;
  return load ? VK_ATTACHMENT_LOAD_OP_LOAD : VK_ATTACHMENT_LOAD_OP_DONT_CARE;
}

static bool loadShader(gpu_shader_source* source, VkShaderStageFlagBits stage, VkShaderModule* handle, VkPipelineShaderStageCreateInfo* pipelineInfo) {
  VkShaderModuleCreateInfo info = {
    .sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
    .codeSize = source->size,
    .pCode = source->code
  };

  if (vkCreateShaderModule(state.device, &info, NULL, handle)) {
    return false;
  }

  *pipelineInfo = (VkPipelineShaderStageCreateInfo) {
    .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
    .stage = stage,
    .module = *handle,
    .pName = source->entrypoint, // TODO string ownership hooray
    .pSpecializationInfo = NULL // TODO shader flags
  };

  return true;
}

static void setLayout(gpu_texture* texture, VkImageLayout layout, VkPipelineStageFlags nextStages, VkAccessFlags nextActions) {
  if (texture->layout == layout) {
    return;
  }

  VkImageMemoryBarrier barrier = {
    .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
    .srcAccessMask = 0,
    .dstAccessMask = nextActions,
    .oldLayout = texture->layout,
    .newLayout = layout,
    .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
    .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
    .image = texture->handle,
    .subresourceRange = { texture->aspect, 0, VK_REMAINING_MIP_LEVELS, 0, VK_REMAINING_ARRAY_LAYERS }
  };

  // TODO Wait for nothing, but could we opportunistically sync with other pending writes?  Or is that weird
  VkPipelineStageFlags waitFor = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
  vkCmdPipelineBarrier(state.commandBuffer, waitFor, nextStages, 0, 0, NULL, 0, NULL, 1, &barrier);
  texture->layout = layout;
}

static void nicknameObject(uint64_t handle, VkObjectType type, const char* nickname) {
  if (nickname && state.config.debug) {
    VkDebugUtilsObjectNameInfoEXT info = {
      .sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_OBJECT_NAME_INFO_EXT,
      .objectType = type,
      .objectHandle = handle,
      .pObjectName = nickname
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
