#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#pragma once

bool gpu_init();
void gpu_destroy();
void gpu_begin_frame();
void gpu_end_frame();

typedef enum {
  GPU_BUFFER_COPY_SRC = (1 << 0),
  GPU_BUFFER_COPY_DST = (1 << 1),
  GPU_BUFFER_VERTEX   = (1 << 2),
  GPU_BUFFER_INDEX    = (1 << 3),
  GPU_BUFFER_UNIFORM  = (1 << 4),
  GPU_BUFFER_STORAGE  = (1 << 5)
} gpu_buffer_usage;

typedef struct {
  uint64_t size;
  gpu_buffer_usage usage;
} gpu_buffer_info;

typedef struct gpu_buffer gpu_buffer;

size_t gpu_sizeof_buffer();
bool gpu_buffer_init(gpu_buffer* buffer, const gpu_buffer_info* info);
void gpu_buffer_destroy(gpu_buffer* buffer);
