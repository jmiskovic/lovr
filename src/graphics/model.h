#include "buffer.h"
#include "texture.h"
#include "../model/modelData.h"
#include "../glfw.h"

#ifndef LOVR_MODEL_TYPES
#define LOVR_MODEL_TYPES
typedef struct {
  ModelData* modelData;
  Buffer* buffer;
  Texture* texture;
} Model;
#endif

Model* lovrModelCreate(void* data, int size);
void lovrModelDestroy(Model* model);
void lovrModelDraw(Model* model, float x, float y, float z, float scale, float angle, float ax, float ay, float az);
Texture* lovrModelGetTexture(Model* model);
void lovrModelSetTexture(Model* model, Texture* texture);
