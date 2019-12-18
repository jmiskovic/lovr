#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#pragma once

typedef struct gpu_buffer gpu_buffer;
typedef struct gpu_texture gpu_texture;
typedef struct gpu_canvas gpu_canvas;

typedef enum {
  GPU_BUFFER_USAGE_VERTEX   = (1 << 0),
  GPU_BUFFER_USAGE_INDEX    = (1 << 1),
  GPU_BUFFER_USAGE_UNIFORM  = (1 << 2),
  GPU_BUFFER_USAGE_COMPUTE  = (1 << 3),
  GPU_BUFFER_USAGE_COPY_SRC = (1 << 4),
  GPU_BUFFER_USAGE_COPY_DST = (1 << 5)
} gpu_buffer_usage;

typedef enum {
  GPU_TEXTURE_USAGE_RENDER   = (1 << 0),
  GPU_TEXTURE_USAGE_SAMPLER  = (1 << 1),
  GPU_TEXTURE_USAGE_COMPUTE  = (1 << 2),
  GPU_TEXTURE_USAGE_COPY_SRC = (1 << 3),
  GPU_TEXTURE_USAGE_COPY_DST = (1 << 4)
} gpu_texture_usage;

typedef enum {
  GPU_TEXTURE_FORMAT_RGBA8
} gpu_texture_format;

typedef enum {
  GPU_TEXTURE_TYPE_2D = 0,
  GPU_TEXTURE_TYPE_3D,
  GPU_TEXTURE_TYPE_CUBE,
  GPU_TEXTURE_TYPE_ARRAY
} gpu_texture_type;

typedef struct {
  uint64_t size;
  uint32_t usage;
  const char* name;
} gpu_buffer_info;

typedef struct {
  gpu_texture_type type;
  gpu_texture_format format;
  uint32_t size[3];
  uint32_t layers;
  uint32_t mipmaps;
  uint32_t usage;
  const char* name;
} gpu_texture_info;

typedef struct {
  uint32_t attachmentCount;
} gpu_canvas_info;

typedef void gpu_callback(void* context, const char* message, bool severe);

typedef struct {
  bool debug;
  gpu_callback* callback;
  void* context;
} gpu_config;

bool gpu_init(gpu_config* config);
void gpu_destroy();
void gpu_begin_frame();
void gpu_end_frame();

size_t gpu_sizeof_buffer();
bool gpu_buffer_init(gpu_buffer* buffer, gpu_buffer_info* info);
void gpu_buffer_destroy(gpu_buffer* buffer);
uint8_t* gpu_buffer_map(gpu_buffer* buffer, uint64_t offset, uint64_t size);
void gpu_buffer_unmap(gpu_buffer* buffer);

size_t gpu_sizeof_texture();
bool gpu_texture_init(gpu_texture* texture, gpu_texture_info* info);
void gpu_texture_destroy(gpu_texture* texture);
void gpu_texture_paste(gpu_texture* texture, uint8_t* data, uint64_t size, uint16_t offset[4], uint16_t extent[4], uint16_t mip);

size_t gpu_sizeof_canvas();
bool gpu_canvas_init(gpu_canvas* canvas, gpu_canvas_info* info);
void gpu_canvas_destroy(gpu_canvas* canvas);
