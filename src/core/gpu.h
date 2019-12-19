#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#pragma once

typedef struct gpu_buffer gpu_buffer;
typedef struct gpu_texture gpu_texture;
typedef struct gpu_sampler gpu_sampler;
typedef struct gpu_canvas gpu_canvas;
typedef struct gpu_shader gpu_shader;

typedef enum {
  GPU_BUFFER_USAGE_VERTEX  = (1 << 0),
  GPU_BUFFER_USAGE_INDEX   = (1 << 1),
  GPU_BUFFER_USAGE_UNIFORM = (1 << 2),
  GPU_BUFFER_USAGE_COMPUTE = (1 << 3),
  GPU_BUFFER_USAGE_COPY    = (1 << 4),
  GPU_BUFFER_USAGE_PASTE   = (1 << 5)
} gpu_buffer_usage;

typedef enum {
  GPU_TEXTURE_USAGE_RENDER  = (1 << 0),
  GPU_TEXTURE_USAGE_SAMPLER = (1 << 1),
  GPU_TEXTURE_USAGE_COMPUTE = (1 << 2),
  GPU_TEXTURE_USAGE_COPY    = (1 << 3),
  GPU_TEXTURE_USAGE_PASTE   = (1 << 4)
} gpu_texture_usage;

typedef enum {
  GPU_TEXTURE_FORMAT_NONE,
  GPU_TEXTURE_FORMAT_RGBA8
} gpu_texture_format;

typedef enum {
  GPU_TEXTURE_TYPE_2D,
  GPU_TEXTURE_TYPE_3D,
  GPU_TEXTURE_TYPE_CUBE,
  GPU_TEXTURE_TYPE_ARRAY
} gpu_texture_type;

typedef enum {
  GPU_FILTER_NEAREST,
  GPU_FILTER_LINEAR
} gpu_filter;

typedef enum {
  GPU_WRAP_CLAMP,
  GPU_WRAP_REPEAT,
  GPU_WRAP_MIRROR
} gpu_wrap;

typedef enum {
  GPU_LOAD_OP_LOAD,
  GPU_LOAD_OP_CLEAR,
  GPU_LOAD_OP_DISCARD
} gpu_load_op;

typedef struct {
  const void* code;
  size_t size;
} gpu_shader_source;

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
  gpu_texture_type type;
  gpu_texture_format format;
  uint32_t baseMipmap;
  uint32_t mipmapCount;
  uint32_t baseLayer;
  uint32_t layerCount;
  const char* name;
} gpu_texture_view_info;

typedef struct {
  gpu_filter min;
  gpu_filter mag;
  gpu_filter mip;
  gpu_wrap wrapu;
  gpu_wrap wrapv;
  gpu_wrap wrapw;
  float anisotropy;
  const char* name;
} gpu_sampler_info;

typedef struct {
  gpu_texture* texture;
  gpu_load_op load;
  bool temporary;
  float clear[4];
} gpu_color_attachment;

typedef struct {
  gpu_texture* texture;
  gpu_load_op load;
  bool temporary;
  float clear;
  struct {
    gpu_load_op load;
    bool temporary;
    uint8_t clear;
  } stencil;
} gpu_depth_attachment;

typedef struct {
  gpu_color_attachment color[4];
  gpu_depth_attachment depth;
  uint32_t size[2];
  uint32_t msaa;
  bool stereo;
  const char* name;
} gpu_canvas_info;

typedef struct {
  gpu_shader_source vertex;
  gpu_shader_source fragment;
  gpu_shader_source compute;
  const char* name;
} gpu_shader_info;

typedef void gpu_callback(void* context, const char* message, bool severe);

typedef struct {
  bool debug;
  gpu_callback* callback;
  void* context;
} gpu_config;

bool gpu_init(gpu_config* config);
void gpu_destroy(void);
void gpu_begin_frame(void);
void gpu_end_frame(void);
void gpu_begin_render(gpu_canvas* canvas);
void gpu_end_render(void);

size_t gpu_sizeof_buffer(void);
bool gpu_buffer_init(gpu_buffer* buffer, gpu_buffer_info* info);
void gpu_buffer_destroy(gpu_buffer* buffer);
uint8_t* gpu_buffer_map(gpu_buffer* buffer, uint64_t offset, uint64_t size);
void gpu_buffer_unmap(gpu_buffer* buffer);

size_t gpu_sizeof_texture(void);
bool gpu_texture_init(gpu_texture* texture, gpu_texture_info* info);
bool gpu_texture_init_view(gpu_texture* view, gpu_texture* source, gpu_texture_view_info* info);
void gpu_texture_destroy(gpu_texture* texture);
void gpu_texture_paste(gpu_texture* texture, uint8_t* data, uint64_t size, uint16_t offset[4], uint16_t extent[4], uint16_t mip);

size_t gpu_sizeof_sampler(void);
bool gpu_sampler_init(gpu_sampler* sampler, gpu_sampler_info* info);
void gpu_sampler_destroy(gpu_sampler* sampler);

size_t gpu_sizeof_canvas(void);
bool gpu_canvas_init(gpu_canvas* canvas, gpu_canvas_info* info);
void gpu_canvas_destroy(gpu_canvas* canvas);

size_t gpu_sizeof_shader(void);
bool gpu_shader_init(gpu_shader* shader, gpu_shader_info* info);
void gpu_shader_destroy(gpu_shader* shader);
