#include <stdlib.h>
#include "physics.h"
#include "util.h"
#include "joltc.h"
#include "core/maf.h"

static JPH_TempAllocator *temp_allocator;
static JPH_JobSystem *job_system;

struct World {
  uint32_t ref;
  JPH_PhysicsSystem *physics_system;
  JPH_BodyInterface *body_interface;
  int collision_steps;
  Collider* head;
/*
  dWorldID id;
  dSpaceID space;
  dJointGroupID contactGroup;
  arr_t(Shape*) overlaps;
  char* tags[MAX_TAGS];
  uint16_t masks[MAX_TAGS];
*/
};

struct Collider {
  uint32_t ref;
  JPH_BodyID id;
  JPH_Body *body;
  World* world;
  Collider* prev;
  Collider* next;
  arr_t(Shape*) shapes;
  arr_t(Joint*) joints;
  void* userdata;
/*
  uint32_t tag;
*/
};

struct Shape {
  uint32_t ref;
  ShapeType type;
  Collider* collider;
  JPH_Shape* shape;
  void* userdata;
};

struct Joint {
  uint32_t ref;
  JointType type;
  JPH_Constraint * constraint;
  void* userdata;
};

static void matrix_struct_to_array(const JPH_Matrix4x4* matrix, float arr[16]) {
    arr[0] = matrix->m11; arr[1] = matrix->m12; arr[2] = matrix->m13; arr[3] = matrix->m14;
    arr[4] = matrix->m21; arr[5] = matrix->m22; arr[6] = matrix->m23; arr[7] = matrix->m24;
    arr[8] = matrix->m31; arr[9] = matrix->m32; arr[10] = matrix->m33; arr[11] = matrix->m34;
    arr[12] = matrix->m41; arr[13] = matrix->m42; arr[14] = matrix->m43; arr[15] = matrix->m44;
}

bool lovrPhysicsInit(void) {
  JPH_Init();
  temp_allocator = JPH_TempAllocator_Create(32 * 1024 * 1024);
  job_system = JPH_JobSystemThreadPool_Create(2048, 8, -1);
}

void lovrPhysicsDestroy(void) {}

// Object layers
#define NUM_OBJ_LAYERS 2
#define OBJ_LAYER_NON_MOVING 0
#define OBJ_LAYER_MOVING 1

// Broad phase layers
#define NUM_BP_LAYERS 2
#define BP_LAYER_NON_MOVING 0
#define BP_LAYER_MOVING 1

JPH_BroadPhaseLayer broadphase_from_object[NUM_OBJ_LAYERS];
const char* broadphase_name_from_object[NUM_OBJ_LAYERS] = {
  "non-moving",
  "moving"
};

static uint32_t
BPLayerInterface_GetNumBroadPhaseLayers(const JPH_BroadPhaseLayerInterface* interface) {
    return NUM_BP_LAYERS;
}

static JPH_BroadPhaseLayer
BPLayerInterface_GetBroadPhaseLayer(const JPH_BroadPhaseLayerInterface* self, JPH_ObjectLayer layer) {
  lovrAssert(layer < NUM_BP_LAYERS, "Broad-phase layer out of range");
  return broadphase_from_object[layer];
}

static const char*
BPLayerInterface_GetBroadPhaseLayerName(const JPH_BroadPhaseLayerInterface* self, JPH_BroadPhaseLayer layer) {
  return broadphase_name_from_object[layer];
}


static JPH_Bool32
BroadPhaseLayerFilter_ShouldCollide(const JPH_ObjectVsBroadPhaseLayerFilter* filter, JPH_ObjectLayer layer1, JPH_BroadPhaseLayer bp_layer2) {
  // return true if one of them is moving
  switch (layer1) {
    case OBJ_LAYER_NON_MOVING:
      return bp_layer2 == BP_LAYER_MOVING;
    case OBJ_LAYER_MOVING:
      return true;
    default:
      lovrAssert(false, "Broad-phase collision check between unknown layers");
  }
  return true;
}

static JPH_Bool32
ObjectLayerPairFilter_ShouldCollide(const JPH_ObjectLayerPairFilter* filter, JPH_ObjectLayer object1, JPH_ObjectLayer object2) {
  return true;
  switch (object1) {
    case OBJ_LAYER_NON_MOVING:
      return object2 == OBJ_LAYER_MOVING;
    case OBJ_LAYER_MOVING:
      return true;
    default:
        lovrAssert(false, "Collision check between unknown object layers");
        return false;
  }
}

World* lovrWorldCreate(float xg, float yg, float zg, bool allowSleep, const char** tags, uint32_t tagCount) {
  World* world = calloc(1, sizeof(World));
  lovrAssert(world, "Out of memory");
  world->physics_system = JPH_PhysicsSystem_Create();
  world->body_interface = JPH_PhysicsSystem_GetBodyInterface(world->physics_system);
  world->collision_steps = 1;
  world->ref = 1;
  const uint32_t max_bodies = 1024 * 2;
  const uint32_t num_body_mutexes = 0; // zero is auto-detect
  const uint32_t max_body_pairs = 1024 * 2;
  const uint32_t max_contact_constraints = 1024 * 2;

  JPH_BroadPhaseLayerInterface* broad_phase_layer_interface = JPH_BroadPhaseLayerInterface_Create();
  JPH_BroadPhaseLayerInterface_SetProcs((JPH_BroadPhaseLayerInterface_Procs){
    .GetNumBroadPhaseLayers = BPLayerInterface_GetNumBroadPhaseLayers,
    .GetBroadPhaseLayer = BPLayerInterface_GetBroadPhaseLayer,
    .GetBroadPhaseLayerName = BPLayerInterface_GetBroadPhaseLayerName
  });

  JPH_ObjectVsBroadPhaseLayerFilter* broad_phase_layer_filter = JPH_ObjectVsBroadPhaseLayerFilter_Create();
  JPH_ObjectVsBroadPhaseLayerFilter_SetProcs((JPH_ObjectVsBroadPhaseLayerFilter_Procs) {
    .ShouldCollide = BroadPhaseLayerFilter_ShouldCollide
  });

  JPH_ObjectLayerPairFilter* object_layer_pair_filter = JPH_ObjectLayerPairFilter_Create();
  JPH_ObjectLayerPairFilter_SetProcs((JPH_ObjectLayerPairFilter_Procs) {
    .ShouldCollide = ObjectLayerPairFilter_ShouldCollide
  });

  JPH_PhysicsSystem_Init(
    world->physics_system,
    max_bodies,
    num_body_mutexes,
    max_body_pairs,
    max_contact_constraints,
    broad_phase_layer_interface,
    broad_phase_layer_filter,
    object_layer_pair_filter);

  return world;
}

void lovrWorldDestroy(void* ref) {}

void lovrWorldDestroyData(World* world) {}

void lovrWorldUpdate(World* world, float dt, CollisionResolver resolver, void* userdata) {
  JPH_PhysicsUpdateError err = JPH_PhysicsSystem_Update(
    world->physics_system,
    dt, world->collision_steps,
    temp_allocator,
    job_system);
}

int lovrWorldGetStepCount(World* world) {
  return world->collision_steps;
}

void lovrWorldSetStepCount(World* world, int iterations) {
  // todo: with too big count JobSystemThreadPool.cpp:124: (false) No jobs available!
  world->collision_steps = iterations;
}

void lovrWorldComputeOverlaps(World* world) {
  //arr_clear(&world->overlaps);
}

int lovrWorldGetNextOverlap(World* world, Shape** a, Shape** b) {}

int lovrWorldCollide(World* world, Shape* a, Shape* b, float friction, float restitution) {}

void lovrWorldGetContacts(World* world, Shape* a, Shape* b, Contact contacts[MAX_CONTACTS], uint32_t* count) {}

void lovrWorldRaycast(World* world, float x1, float y1, float z1, float x2, float y2, float z2, RaycastCallback callback, void* userdata) {}

bool lovrWorldQueryBox(World* world, float position[3], float size[3], QueryCallback callback, void* userdata) {}

bool lovrWorldQuerySphere(World* world, float position[3], float radius, QueryCallback callback, void* userdata) {}

Collider* lovrWorldGetFirstCollider(World* world) {
  return world->head;
}

void lovrWorldGetGravity(World* world, float* x, float* y, float* z) {
  JPH_Vec3 gravity;
  JPH_PhysicsSystem_GetGravity(world->physics_system, &gravity);
  *x = gravity.x;
  *y = gravity.y;
  *z = gravity.z;
}

void lovrWorldSetGravity(World* world, float x, float y, float z) {
  const JPH_Vec3 gravity = {
    .x = x,
    .y = y,
    .z = z
  };
  JPH_PhysicsSystem_SetGravity(world->physics_system, &gravity);
}

float lovrWorldGetResponseTime(World* world) {
  lovrLog(LOG_WARN, "PHY", "Jolt doesn't support global ResponseTime option");
}

void lovrWorldSetResponseTime(World* world, float responseTime) {
  lovrLog(LOG_WARN, "PHY", "Jolt doesn't support global ResponseTime option");
}

float lovrWorldGetTightness(World* world) {
  lovrLog(LOG_WARN, "PHY", "Jolt doesn't support Tightness option");
}

void lovrWorldSetTightness(World* world, float tightness) {
  lovrLog(LOG_WARN, "PHY", "Jolt doesn't support Tightness option");
}

void lovrWorldGetLinearDamping(World* world, float* damping, float* threshold) {
  lovrLog(LOG_WARN, "PHY", "Jolt doesn't support global LinearDamping option");
}

void lovrWorldSetLinearDamping(World* world, float damping, float threshold) {
  lovrLog(LOG_WARN, "PHY", "Jolt doesn't support global LinearDamping option");
}

void lovrWorldGetAngularDamping(World* world, float* damping, float* threshold) {
  lovrLog(LOG_WARN, "PHY", "Jolt doesn't support global AngularDamping option");
}

void lovrWorldSetAngularDamping(World* world, float damping, float threshold) {
  lovrLog(LOG_WARN, "PHY", "Jolt doesn't support global AngularDamping option");
}

bool lovrWorldIsSleepingAllowed(World* world) {
  lovrLog(LOG_WARN, "PHY", "Jolt doesn't support global SleepingAllowed option");
}

void lovrWorldSetSleepingAllowed(World* world, bool allowed) {
  lovrLog(LOG_WARN, "PHY", "Jolt doesn't support global SleepingAllowed option");
}

const char* lovrWorldGetTagName(World* world, uint32_t tag) {}

void lovrWorldDisableCollisionBetween(World* world, const char* tag1, const char* tag2) {}

void lovrWorldEnableCollisionBetween(World* world, const char* tag1, const char* tag2) {}

bool lovrWorldIsCollisionEnabledBetween(World* world, const char* tag1, const char* tag2) {}

Collider* lovrColliderCreate(World* world, float x, float y, float z) {
  // todo: crashes when too many are added
  Collider* collider = calloc(1, sizeof(Collider));
  lovrAssert(collider, "Out of memory");
  collider->ref = 1;
  collider->world = world;
  const JPH_RVec3 position = { .x = x, .y = y, .z = z };
  const JPH_Quat rotation = { .x = 0.f, .y = 0.f, .z = 0.f, .w = 1.f };

  // todo: a temp shape is created, to be replaced in lovrColliderAddShape
  JPH_Shape* shape = (JPH_Shape *) JPH_SphereShape_Create(1);
  JPH_BodyCreationSettings* settings = JPH_BodyCreationSettings_Create3(
    shape, &position, &rotation, JPH_MotionType_Dynamic, OBJ_LAYER_MOVING);
  collider->body = JPH_BodyInterface_CreateBody(world->body_interface, settings);
  collider->id = JPH_Body_GetID(collider->body);
  JPH_BodyInterface_AddBody(world->body_interface, collider->id, JPH_Activation_Activate);
  JPH_BodyInterface_SetUserData(world->body_interface, collider->id, (uint64_t) collider);

  arr_init(&collider->shapes, arr_alloc);
  arr_init(&collider->joints, arr_alloc);

  // Adjust the world's collider list
  if (!collider->world->head) {
    collider->world->head = collider;
  } else {
    collider->next = collider->world->head;
    collider->next->prev = collider;
    collider->world->head = collider;
  }
  // The world owns a reference to the collider
  lovrRetain(collider);
  return collider;
}

void lovrColliderDestroy(void* ref) {}

void lovrColliderDestroyData(Collider* collider) {}

bool lovrColliderIsDestroyed(Collider* collider) {}

void lovrColliderInitInertia(Collider* collider, Shape* shape) {}

World* lovrColliderGetWorld(Collider* collider) {
  return collider->world;
}

Collider* lovrColliderGetNext(Collider* collider) {
  return collider->next;
}

void lovrColliderAddShape(Collider* collider, Shape* shape) {
  lovrRetain(shape);
  shape->collider = collider;
  arr_push(&collider->shapes, shape);
  JPH_BodyInterface_SetShape(collider->world->body_interface, collider->id, shape->shape, true, JPH_Activation_Activate);
}

void lovrColliderRemoveShape(Collider* collider, Shape* shape) {}

Shape** lovrColliderGetShapes(Collider* collider, size_t* count) {
  *count = collider->shapes.length;
  return collider->shapes.data;
}

Joint** lovrColliderGetJoints(Collider* collider, size_t* count) {
  *count = collider->joints.length;
  return collider->joints.data;
}

void* lovrColliderGetUserData(Collider* collider) {
  return collider->userdata;
}

void lovrColliderSetUserData(Collider* collider, void* data) {
  collider->userdata = data;
}

const char* lovrColliderGetTag(Collider* collider) {}

bool lovrColliderSetTag(Collider* collider, const char* tag) {}

float lovrColliderGetFriction(Collider* collider) {
  return JPH_BodyInterface_GetFriction(collider->world->body_interface, collider->id);
}

void lovrColliderSetFriction(Collider* collider, float friction) {
  JPH_BodyInterface_SetFriction(collider->world->body_interface, collider->id, friction);
}

float lovrColliderGetRestitution(Collider* collider) {
  return JPH_BodyInterface_GetRestitution(collider->world->body_interface, collider->id);
}

void lovrColliderSetRestitution(Collider* collider, float restitution) {
  JPH_BodyInterface_SetRestitution(collider->world->body_interface, collider->id, restitution);
}

bool lovrColliderIsKinematic(Collider* collider) {
  JPH_MotionType type = JPH_BodyInterface_GetMotionType(collider->world->body_interface, collider->id);
  // todo: what about JPH_MotionType_Static?
  return type == JPH_MotionType_Kinematic;
}

void lovrColliderSetKinematic(Collider* collider, bool kinematic) {
  JPH_BodyInterface_SetMotionType(
    collider->world->body_interface,
    collider->id,
    kinematic ? JPH_MotionType_Kinematic : JPH_MotionType_Dynamic,
    JPH_Activation_Activate);
}

bool lovrColliderIsGravityIgnored(Collider* collider) {
  return JPH_BodyInterface_GetGravityFactor(collider->world->body_interface, collider->id) == 0.f;
}


void lovrColliderSetGravityIgnored(Collider* collider, bool ignored) {
  JPH_BodyInterface_SetGravityFactor(
    collider->world->body_interface,
    collider->id,
    ignored ? 0.f : 1.f);
}

bool lovrColliderIsSleepingAllowed(Collider* collider) {
  return JPH_Body_GetAllowSleeping(collider->body);
}

void lovrColliderSetSleepingAllowed(Collider* collider, bool allowed) {
  JPH_Body_SetAllowSleeping(collider->body, allowed);
}

bool lovrColliderIsAwake(Collider* collider) {
  return JPH_BodyInterface_IsActive(collider->world->body_interface, collider->id);
}

void lovrColliderSetAwake(Collider* collider, bool awake) {
  if (awake) {
    JPH_BodyInterface_ActivateBody(collider->world->body_interface, collider->id);
  } else {
    JPH_BodyInterface_DeactivateBody(collider->world->body_interface, collider->id);
  }
}

float lovrColliderGetMass(Collider* collider) {
   if (collider->shapes.length > 0) {
    JPH_MotionProperties * motion_properties = JPH_Body_GetMotionProperties(collider->body);
    return 1.f / JPH_MotionProperties_GetInverseMassUnchecked(motion_properties);
  }
  return 0.f;
}

void lovrColliderSetMass(Collider* collider, float mass) {
  if (collider->shapes.length > 0) {
    JPH_MotionProperties * motion_properties = JPH_Body_GetMotionProperties(collider->body);
    Shape * shape = collider->shapes.data[0];
    JPH_MassProperties * mass_properties = JPH_Shape_GetMassProperties(shape->shape);
    JPH_MassProperties_ScaleToMass(mass_properties, mass);
    JPH_MotionProperties_SetMassProperties(motion_properties, JPH_AllowedDOFs_All, mass_properties);
  }
}

void lovrColliderGetMassData(Collider* collider, float* cx, float* cy, float* cz, float* mass, float inertia[6]) {}

void lovrColliderSetMassData(Collider* collider, float cx, float cy, float cz, float mass, float inertia[6]) {}

void lovrColliderGetPosition(Collider* collider, float* x, float* y, float* z) {
  JPH_RVec3 position;
  JPH_Body_GetPosition(collider->body, &position);
  *x = position.x;
  *y = position.y;
  *z = position.z;
}

void lovrColliderSetPosition(Collider* collider, float x, float y, float z) {
  JPH_RVec3 position = {
    .x = x,
    .y = y,
    .z = z
  };
  JPH_BodyInterface_SetPosition(
    collider->world->body_interface,
    collider->id,
    &position,
    JPH_Activation_Activate);
}

void lovrColliderGetOrientation(Collider* collider, float* orientation) {
  JPH_Quat quat;
  JPH_Body_GetRotation(collider->body, &quat);
  orientation[0] = quat.x;
  orientation[1] = quat.y;
  orientation[2] = quat.z;
  orientation[3] = quat.w;
}

void lovrColliderSetOrientation(Collider* collider, float* orientation) {
  JPH_Quat rotation = {
   .x = orientation[0],
   .y = orientation[1],
   .z = orientation[2],
   .w = orientation[3]
 };
  JPH_BodyInterface_SetRotation(
    collider->world->body_interface,
    collider->id,
    &rotation,
    JPH_Activation_Activate);
}

void lovrColliderGetLinearVelocity(Collider* collider, float* x, float* y, float* z) {
  JPH_Vec3 velocity;
  JPH_BodyInterface_GetLinearVelocity(collider->world->body_interface, collider->id, &velocity);
  *x = velocity.x;
  *y = velocity.y;
  *z = velocity.z;
}

void lovrColliderSetLinearVelocity(Collider* collider, float x, float y, float z) {
  const JPH_Vec3 velocity = {
    .x = x,
    .y = y,
    .z = z
  };
  JPH_BodyInterface_SetLinearVelocity(collider->world->body_interface, collider->id, &velocity);
}

void lovrColliderGetAngularVelocity(Collider* collider, float* x, float* y, float* z) {
  JPH_Vec3 velocity;
  JPH_BodyInterface_GetAngularVelocity(collider->world->body_interface, collider->id, &velocity);
  *x = velocity.x;
  *y = velocity.y;
  *z = velocity.z;
}

void lovrColliderSetAngularVelocity(Collider* collider, float x, float y, float z) {
  JPH_Vec3 velocity = {
    .x = x,
    .y = y,
    .z = z
  };
  JPH_BodyInterface_SetAngularVelocity(collider->world->body_interface, collider->id, &velocity);
}

void lovrColliderGetLinearDamping(Collider* collider, float* damping, float* threshold) {
  JPH_MotionProperties * properties = JPH_Body_GetMotionProperties(collider->body);
  *damping = JPH_MotionProperties_GetLinearDamping(properties);
  *threshold = 0.f;
}

void lovrColliderSetLinearDamping(Collider* collider, float damping, float threshold) {
  JPH_MotionProperties * properties = JPH_Body_GetMotionProperties(collider->body);
  JPH_MotionProperties_SetLinearDamping(properties, damping);
  if (threshold != 0.f) {
    lovrLog(LOG_WARN, "PHY", "Jolt does not support velocity threshold parameter for damping");
  }
}

void lovrColliderGetAngularDamping(Collider* collider, float* damping, float* threshold) {
  JPH_MotionProperties * properties = JPH_Body_GetMotionProperties(collider->body);
  *damping = JPH_MotionProperties_GetAngularDamping(properties);
  *threshold = 0.f;
}

void lovrColliderSetAngularDamping(Collider* collider, float damping, float threshold) {
  JPH_MotionProperties * properties = JPH_Body_GetMotionProperties(collider->body);
  JPH_MotionProperties_SetAngularDamping(properties, damping);
  if (threshold != 0.f) {
    lovrLog(LOG_WARN, "PHY", "Jolt does not support velocity threshold parameter for damping");
  }
}

void lovrColliderApplyForce(Collider* collider, float x, float y, float z) {
  JPH_Vec3 force = {
    .x = x,
    .y = y,
    .z = z
  };
  JPH_BodyInterface_AddForce(collider->world->body_interface, collider->id, &force);
}

void lovrColliderApplyForceAtPosition(Collider* collider, float x, float y, float z, float cx, float cy, float cz) {
  JPH_Vec3 force = {
    .x = x,
    .y = y,
    .z = z
  };
  JPH_RVec3 position = {
    .x = cx,
    .y = cy,
    .z = cz
  };
  JPH_BodyInterface_AddForce2(collider->world->body_interface, collider->id, &force, &position);
}

void lovrColliderApplyTorque(Collider* collider, float x, float y, float z) {
  JPH_Vec3 torque = {
    .x = x,
    .y = y,
    .z = z
  };
  JPH_BodyInterface_AddTorque(collider->world->body_interface, collider->id, &torque);
}

void lovrColliderGetLocalCenter(Collider* collider, float* x, float* y, float* z) {}

void lovrColliderGetLocalPoint(Collider* collider, float wx, float wy, float wz, float* x, float* y, float* z) {}

void lovrColliderGetWorldPoint(Collider* collider, float x, float y, float z, float* wx, float* wy, float* wz) {}

void lovrColliderGetLocalVector(Collider* collider, float wx, float wy, float wz, float* x, float* y, float* z) {}

void lovrColliderGetWorldVector(Collider* collider, float x, float y, float z, float* wx, float* wy, float* wz) {}

void lovrColliderGetLinearVelocityFromLocalPoint(Collider* collider, float x, float y, float z, float* vx, float* vy, float* vz) {}

void lovrColliderGetLinearVelocityFromWorldPoint(Collider* collider, float wx, float wy, float wz, float* vx, float* vy, float* vz) {}

void lovrColliderGetAABB(Collider* collider, float aabb[6]) {
  JPH_AABox box = JPH_Body_GetWorldSpaceBounds(collider->body);
  aabb[0] = box.min.x;
  aabb[1] = box.max.x;
  aabb[2] = box.min.y;
  aabb[3] = box.max.y;
  aabb[4] = box.min.z;
  aabb[5] = box.max.z;
}

void lovrShapeDestroy(void* ref) {}

void lovrShapeDestroyData(Shape* shape) {}

ShapeType lovrShapeGetType(Shape* shape) {
  return shape->type;
}

Collider* lovrShapeGetCollider(Shape* shape) {
  return shape->collider;
}

bool lovrShapeIsEnabled(Shape* shape) {}

void lovrShapeSetEnabled(Shape* shape, bool enabled) {}

bool lovrShapeIsSensor(Shape* shape) {
  lovrLog(LOG_WARN, "PHY", "Jolt sensor property fetched from collider, not shape");
  return JPH_Body_IsSensor(shape->collider->body);
}

void lovrShapeSetSensor(Shape* shape, bool sensor) {
  lovrLog(LOG_WARN, "PHY", "Jolt sensor property is applied to collider, not shape");
  JPH_Body_SetIsSensor(shape->collider->body, sensor);
}

void* lovrShapeGetUserData(Shape* shape) {
  return shape->userdata;
}

void lovrShapeSetUserData(Shape* shape, void* data) {
  shape->userdata = data;
}

void lovrShapeGetPosition(Shape* shape, float* x, float* y, float* z) {
  // todo: composite shapes
  *x = 0.f;
  *y = 0.f;
  *z = 0.f;
}

void lovrShapeSetPosition(Shape* shape, float x, float y, float z) {
  // todo: composite shapes
}

void lovrShapeGetOrientation(Shape* shape, float* orientation) {
  // todo: composite shapes
  orientation[0] = 0.f;
  orientation[1] = 0.f;
  orientation[2] = 0.f;
  orientation[3] = 1.f;
}

void lovrShapeSetOrientation(Shape* shape, float* orientation) {
  // todo: composite shapes
}

void lovrShapeGetMass(Shape* shape, float density, float* cx, float* cy, float* cz, float* mass, float inertia[6]) {}

void lovrShapeGetAABB(Shape* shape, float aabb[6]) {
  // todo: with composite shapes this is no longer correct
  lovrColliderGetAABB(shape->collider, aabb);
}

SphereShape* lovrSphereShapeCreate(float radius) {
  lovrCheck(radius > 0.f, "SphereShape radius must be positive");
  SphereShape* sphere = calloc(1, sizeof(SphereShape));
  lovrAssert(sphere, "Out of memory");
  sphere->ref = 1;
  sphere->type = SHAPE_SPHERE;
  sphere->shape = (JPH_Shape *) JPH_SphereShape_Create(radius);
  return sphere;
}

float lovrSphereShapeGetRadius(SphereShape* sphere) {
  return JPH_SphereShape_GetRadius((JPH_SphereShape*) sphere->shape);
}

void lovrSphereShapeSetRadius(SphereShape* sphere, float radius) {
  lovrLog(LOG_WARN, "PHY", "Jolt SphereShape radius is read-only");
  // todo: no setter available, but the shape could be removed and re-added
}

BoxShape* lovrBoxShapeCreate(float w, float h, float d) {
  BoxShape* box = calloc(1, sizeof(BoxShape));
  lovrAssert(box, "Out of memory");
  box->ref = 1;
  box->type = SHAPE_BOX;
  const JPH_Vec3 halfExtent = {
    .x = w / 2,
    .y = h / 2,
    .z = d / 2
  };
  box->shape = (JPH_Shape *) JPH_BoxShape_Create(&halfExtent, 0.f);
  return box;
}

void lovrBoxShapeGetDimensions(BoxShape* box, float* w, float* h, float* d) {
  JPH_Vec3 halfExtent;
  JPH_BoxShape_GetHalfExtent((JPH_BoxShape *) box->shape, &halfExtent);
  *w = halfExtent.x * 2.f;
  *h = halfExtent.y * 2.f;
  *d = halfExtent.z * 2.f;
}

void lovrBoxShapeSetDimensions(BoxShape* box, float w, float h, float d) {
  lovrLog(LOG_WARN, "PHY", "Jolt BoxShape dimensions are read-only");
  // todo: no setter available, but the shape could be removed and re-added
}

CapsuleShape* lovrCapsuleShapeCreate(float radius, float length) {
  lovrCheck(radius > 0.f && length > 0.f, "CapsuleShape dimensions must be positive");
  CapsuleShape* capsule = calloc(1, sizeof(CapsuleShape));
  lovrAssert(capsule, "Out of memory");
  capsule->ref = 1;
  capsule->type = SHAPE_CAPSULE;
  capsule->shape = (JPH_Shape *) JPH_CapsuleShape_Create(length / 2, radius);
  return capsule;
}

float lovrCapsuleShapeGetRadius(CapsuleShape* capsule) {
  return JPH_CapsuleShape_GetRadius((JPH_CapsuleShape *) capsule->shape);
}

void lovrCapsuleShapeSetRadius(CapsuleShape* capsule, float radius) {
  lovrLog(LOG_WARN, "PHY", "Jolt CapsuleShape radius is read-only");
  // todo: no setter available, but the shape could be removed and re-added
}

float lovrCapsuleShapeGetLength(CapsuleShape* capsule) {
  return 2.f * JPH_CapsuleShape_GetHalfHeightOfCylinder((JPH_CapsuleShape *) capsule->shape);
}

void lovrCapsuleShapeSetLength(CapsuleShape* capsule, float length) {
  lovrLog(LOG_WARN, "PHY", "Jolt CapsuleShape length is read-only");
  // todo: no setter available, but the shape could be removed and re-added
}

CylinderShape* lovrCylinderShapeCreate(float radius, float length) {
  lovrCheck(radius > 0.f && length > 0.f, "CylinderShape dimensions must be positive");
  CylinderShape* Cylinder = calloc(1, sizeof(CylinderShape));
  lovrAssert(Cylinder, "Out of memory");
  Cylinder->ref = 1;
  Cylinder->type = SHAPE_CYLINDER;
  Cylinder->shape = (JPH_Shape *) JPH_CylinderShape_Create(length / 2.f, radius);
  return Cylinder;
}

float lovrCylinderShapeGetRadius(CylinderShape* Cylinder) {
  return JPH_CylinderShape_GetRadius((JPH_CylinderShape *) Cylinder->shape);
}

void lovrCylinderShapeSetRadius(CylinderShape* Cylinder, float radius) {
  lovrLog(LOG_WARN, "PHY", "Jolt CylinderShape radius is read-only");
  // todo: no setter available, but the shape could be removed and re-added
}

float lovrCylinderShapeGetLength(CylinderShape* Cylinder) {
  return JPH_CylinderShape_GetHalfHeight((JPH_CylinderShape *) Cylinder->shape) * 2.f;
}

void lovrCylinderShapeSetLength(CylinderShape* cylinder, float length) {
  lovrLog(LOG_WARN, "PHY", "Jolt CylinderShape length is read-only");
  // todo: no setter available, but the shape could be removed and re-added
}

MeshShape* lovrMeshShapeCreate(int vertexCount, float vertices[], int indexCount, uint32_t indices[]) {
  MeshShape* mesh = calloc(1, sizeof(MeshShape));
  lovrAssert(mesh, "Out of memory");
  mesh->ref = 1;
  mesh->type = SHAPE_MESH;

  int triangleCount = indexCount / 3;
  JPH_IndexedTriangle * indexedTriangles = malloc(triangleCount * sizeof(JPH_IndexedTriangle));
  for (int i = 0; i < triangleCount; i++) {
    indexedTriangles[i].i1 = indices[i * 3 + 0];
    indexedTriangles[i].i2 = indices[i * 3 + 1];
    indexedTriangles[i].i3 = indices[i * 3 + 2];
    indexedTriangles[i].materialIndex = 0;
  }
  JPH_MeshShapeSettings * shape_settings = JPH_MeshShapeSettings_Create2(
    (const JPH_Vec3*) vertices,
    vertexCount,
    indexedTriangles,
    triangleCount);
  mesh->shape = (JPH_Shape *) JPH_MeshShapeSettings_CreateShape(shape_settings);
  return mesh;
}

TerrainShape* lovrTerrainShapeCreate(float* vertices, uint32_t widthSamples, uint32_t depthSamples, float horizontalScale, float verticalScale) {}

void lovrJointGetAnchors(Joint* joint, float anchor1[3], float anchor2[3]) {
  JPH_Body * body1 = JPH_TwoBodyConstraint_GetBody1((JPH_TwoBodyConstraint *) joint->constraint);
  JPH_Body * body2 = JPH_TwoBodyConstraint_GetBody2((JPH_TwoBodyConstraint *) joint->constraint);
  JPH_Matrix4x4 centerOfMassTransformStruct1;
  JPH_Matrix4x4 centerOfMassTransformStruct2;
  JPH_Body_GetCenterOfMassTransform(body1, &centerOfMassTransformStruct1);
  JPH_Body_GetCenterOfMassTransform(body2, &centerOfMassTransformStruct2);
  JPH_Matrix4x4 constraintToBody1;
  JPH_Matrix4x4 constraintToBody2;
  JPH_TwoBodyConstraint_GetConstraintToBody1Matrix((JPH_TwoBodyConstraint *) joint->constraint, &constraintToBody1);
  JPH_TwoBodyConstraint_GetConstraintToBody2Matrix((JPH_TwoBodyConstraint *) joint->constraint, &constraintToBody2);
  float translation1[4] = {
    constraintToBody1.m41,
    constraintToBody1.m42,
    constraintToBody1.m43,
    constraintToBody1.m44
  };
  float translation2[4] = {
    constraintToBody2.m41,
    constraintToBody2.m42,
    constraintToBody2.m43,
    constraintToBody2.m44
  };
  float centerOfMassTransformArray1[16];
  float centerOfMassTransformArray2[16];
  matrix_struct_to_array(&centerOfMassTransformStruct1, centerOfMassTransformArray1);
  matrix_struct_to_array(&centerOfMassTransformStruct2, centerOfMassTransformArray2);
  mat4_mulVec4((mat4) &centerOfMassTransformArray1, translation1);
  mat4_mulVec4((mat4) &centerOfMassTransformArray2, translation2);
  anchor1[0] = translation1[0];
  anchor1[1] = translation1[1];
  anchor1[2] = translation1[2];
  anchor2[0] = translation2[0];
  anchor2[1] = translation2[1];
  anchor2[2] = translation2[2];
}

void lovrJointDestroy(void* ref) {}

void lovrJointDestroyData(Joint* joint) {}

JointType lovrJointGetType(Joint* joint) {
  return joint->type;
}

void lovrJointGetColliders(Joint* joint, Collider** a, Collider** b) {
  JPH_Body * bodyA = JPH_TwoBodyConstraint_GetBody1((JPH_TwoBodyConstraint *) joint->constraint);
  JPH_Body * bodyB = JPH_TwoBodyConstraint_GetBody2((JPH_TwoBodyConstraint *) joint->constraint);

  if (bodyA) {
    *a = (Collider*) JPH_Body_GetUserData(bodyA);
  }

  if (bodyB) {
    *b = (Collider*) JPH_Body_GetUserData(bodyB);
  }
}

void* lovrJointGetUserData(Joint* joint) {
  return joint->userdata;
}

void lovrJointSetUserData(Joint* joint, void* data) {
  joint->userdata = data;
}

bool lovrJointIsEnabled(Joint* joint) {
  return JPH_Constraint_GetEnabled(joint->constraint);
}

void lovrJointSetEnabled(Joint* joint, bool enable) {
  JPH_Constraint_SetEnabled(joint->constraint, enable);
}

BallJoint* lovrBallJointCreate(Collider* a, Collider* b, float anchor[3]) {
  lovrAssert(a->world == b->world, "Joint bodies must exist in same World");
  BallJoint* joint = calloc(1, sizeof(BallJoint));
  lovrAssert(joint, "Out of memory");
  joint->ref = 1;
  joint->type = JOINT_BALL;

  JPH_PointConstraintSettings * settings = JPH_PointConstraintSettings_Create();
  JPH_RVec3 point1 = {
    .x = anchor[0],
    .y = anchor[1],
    .z = anchor[2]
  };
  JPH_RVec3 point2 = {
    .x = anchor[0],
    .y = anchor[1],
    .z = anchor[2]
  };
  JPH_PointConstraintSettings_SetPoint1(settings, &point1);
  JPH_PointConstraintSettings_SetPoint2(settings, &point2);
  joint->constraint = (JPH_Constraint *) JPH_PointConstraintSettings_CreateConstraint(settings, a->body, b->body);
  JPH_PhysicsSystem_AddConstraint(a->world->physics_system, joint->constraint);
  arr_push(&a->joints, joint);
  arr_push(&b->joints, joint);
  lovrRetain(joint);
  return joint;
}

void lovrBallJointGetAnchors(BallJoint* joint, float anchor1[3], float anchor2[3]) {
  lovrJointGetAnchors((Joint*) joint, anchor1, anchor2);
}

void lovrBallJointSetAnchor(BallJoint* joint, float anchor[3]) {
  JPH_RVec3 point;
  point.x = anchor[0];
  point.y = anchor[1];
  point.z = anchor[2];
  JPH_PointConstraint_SetPoint1((JPH_PointConstraint *) joint->constraint, JPH_ConstraintSpace_WorldSpace, &point);
  JPH_PointConstraint_SetPoint2((JPH_PointConstraint *) joint->constraint, JPH_ConstraintSpace_WorldSpace, &point);
}

float lovrBallJointGetResponseTime(Joint* joint) {
  lovrLog(LOG_WARN, "PHY", "Jolt does not support BallJoint response time");
}

void lovrBallJointSetResponseTime(Joint* joint, float responseTime) {
  lovrLog(LOG_WARN, "PHY", "Jolt does not support BallJoint response time");
}

float lovrBallJointGetTightness(Joint* joint) {
  lovrLog(LOG_WARN, "PHY", "Jolt does not support BallJoint tightness");
}

void lovrBallJointSetTightness(Joint* joint, float tightness) {
  lovrLog(LOG_WARN, "PHY", "Jolt does not support BallJoint tightness");
}

DistanceJoint* lovrDistanceJointCreate(Collider* a, Collider* b, float anchor1[3], float anchor2[3]) {
  lovrAssert(a->world == b->world, "Joint bodies must exist in same World");
  DistanceJoint* joint = calloc(1, sizeof(DistanceJoint));
  lovrAssert(joint, "Out of memory");
  joint->ref = 1;
  joint->type = JOINT_DISTANCE;

  JPH_DistanceConstraintSettings * settings = JPH_DistanceConstraintSettings_Create();
  JPH_RVec3 point1 = {
    .x = anchor1[0],
    .y = anchor1[1],
    .z = anchor1[2]
  };
  JPH_RVec3 point2 = {
    .x = anchor2[0],
    .y = anchor2[1],
    .z = anchor2[2]
  };
  JPH_DistanceConstraintSettings_SetPoint1(settings, &point1);
  JPH_DistanceConstraintSettings_SetPoint2(settings, &point2);
  joint->constraint = (JPH_Constraint *) JPH_DistanceConstraintSettings_CreateConstraint(settings, a->body, b->body);
  JPH_PhysicsSystem_AddConstraint(a->world->physics_system, joint->constraint);
  arr_push(&a->joints, joint);
  arr_push(&b->joints, joint);
  lovrRetain(joint);
  return joint;
}

void lovrDistanceJointGetAnchors(DistanceJoint* joint, float anchor1[3], float anchor2[3]) {
  lovrJointGetAnchors((Joint*) joint, anchor1, anchor2);
}

void lovrDistanceJointSetAnchors(DistanceJoint* joint, float anchor1[3], float anchor2[3]) {
  lovrLog(LOG_WARN, "PHY", "Jolt does not support modifying joint anchors after creation");
  // todo: no setter available, but the constraint could be removed and re-added
}

float lovrDistanceJointGetDistance(DistanceJoint* joint) {
  return JPH_DistanceConstraint_GetMaxDistance((JPH_DistanceConstraint *) joint->constraint);
}

void lovrDistanceJointSetDistance(DistanceJoint* joint, float distance) {
  JPH_DistanceConstraint_SetDistance((JPH_DistanceConstraint *) joint->constraint, distance, distance);
}

float lovrDistanceJointGetResponseTime(Joint* joint) {
  JPH_SpringSettings* settings = JPH_DistanceConstraint_GetLimitsSpringSettings((JPH_DistanceConstraint *) joint->constraint);
  return 1.f / JPH_SpringSettings_GetFrequency(settings);
}

void lovrDistanceJointSetResponseTime(Joint* joint, float responseTime) {
  JPH_SpringSettings* settings = JPH_SpringSettings_Create(1.f / responseTime, 0.f);
  JPH_DistanceConstraint_SetLimitsSpringSettings((JPH_DistanceConstraint *) joint->constraint, settings);
}

float lovrDistanceJointGetTightness(Joint* joint) {
  // todo: jolt has spring damping instead of tightness, not compatible with lovr API
  // (but body's damping is not that different)
  lovrLog(LOG_WARN, "PHY", "Jolt does not support DistanceJoint tightness");
  return 0.f;
}

void lovrDistanceJointSetTightness(Joint* joint, float tightness) {
  lovrLog(LOG_WARN, "PHY", "Jolt does not support DistanceJoint tightness");
}

HingeJoint* lovrHingeJointCreate(Collider* a, Collider* b, float anchor[3], float axis[3]) {
  lovrAssert(a->world == b->world, "Joint bodies must exist in the same World");

  HingeJoint* joint = calloc(1, sizeof(HingeJoint));
  lovrAssert(joint, "Out of memory");
  joint->ref = 1;
  joint->type = JOINT_HINGE;

  JPH_HingeConstraintSettings* settings = JPH_HingeConstraintSettings_Create();

  JPH_RVec3 point = {
    .x = anchor[0],
    .y = anchor[1],
    .z = anchor[2]
  };

  JPH_RVec3 axisVec = {
    .x = axis[0],
    .y = axis[1],
    .z = axis[2]
  };
  JPH_HingeConstraintSettings_SetPoint1(settings, &point);
  JPH_HingeConstraintSettings_SetPoint2(settings, &point);
  JPH_HingeConstraintSettings_SetHingeAxis1(settings, &axisVec);
  JPH_HingeConstraintSettings_SetHingeAxis2(settings, &axisVec);
  joint->constraint = (JPH_Constraint *) JPH_HingeConstraintSettings_CreateConstraint(settings, a->body, b->body);
  JPH_PhysicsSystem_AddConstraint(a->world->physics_system, joint->constraint);
  arr_push(&a->joints, joint);
  arr_push(&b->joints, joint);
  lovrRetain(joint);
  return joint;
}

void lovrHingeJointGetAnchors(HingeJoint* joint, float anchor1[3], float anchor2[3]) {
  lovrJointGetAnchors((Joint*) joint, anchor1, anchor2);
}

void lovrHingeJointSetAnchor(HingeJoint* joint, float anchor[3]) {
  lovrLog(LOG_WARN, "PHY", "Jolt does not support modifying joint anchors after creation");
  // todo: no setter available, but the constraint could be removed and re-added
}

void lovrHingeJointGetAxis(HingeJoint* joint, float axis[3]) {
  JPH_RVec3 resultAxis;
  JPH_HingeConstraintSettings * settings = JPH_HingeConstraint_GetSettings((JPH_HingeConstraint *) joint->constraint);
  JPH_HingeConstraintSettings_GetHingeAxis1(settings, &resultAxis);
  // todo: convert to world coordinates
  axis[0] = resultAxis.x;
  axis[1] = resultAxis.y;
  axis[2] = resultAxis.z;
}

void lovrHingeJointSetAxis(HingeJoint* joint, float axis[3]) {
  lovrLog(LOG_WARN, "PHY", "Jolt does not support modifying joint axis after creation");
  // todo: no setter available, but the constraint could be removed and re-added
}


float lovrHingeJointGetAngle(HingeJoint* joint) {
  return -JPH_HingeConstraint_GetCurrentAngle((JPH_HingeConstraint *) joint->constraint);
}

float lovrHingeJointGetLowerLimit(HingeJoint* joint) {
  return JPH_HingeConstraint_GetLimitsMin((JPH_HingeConstraint *) joint->constraint);
}

void lovrHingeJointSetLowerLimit(HingeJoint* joint, float limit) {
  float upper_limit = JPH_HingeConstraint_GetLimitsMax((JPH_HingeConstraint *) joint->constraint);
  JPH_HingeConstraint_SetLimits((JPH_HingeConstraint *) joint->constraint, limit, upper_limit);
}

float lovrHingeJointGetUpperLimit(HingeJoint* joint) {
  return JPH_HingeConstraint_GetLimitsMax((JPH_HingeConstraint *) joint->constraint);
}

void lovrHingeJointSetUpperLimit(HingeJoint* joint, float limit) {
  float lower_limit = JPH_HingeConstraint_GetLimitsMin((JPH_HingeConstraint *) joint->constraint);
  JPH_HingeConstraint_SetLimits((JPH_HingeConstraint *) joint->constraint, lower_limit, limit);
}

SliderJoint* lovrSliderJointCreate(Collider* a, Collider* b, float axis[3]) {
  lovrAssert(a->world == b->world, "Joint bodies must exist in the same World");

  SliderJoint* joint = calloc(1, sizeof(SliderJoint));
  lovrAssert(joint, "Out of memory");
  joint->ref = 1;
  joint->type = JOINT_SLIDER;

  JPH_SliderConstraintSettings* settings = JPH_SliderConstraintSettings_Create();
  JPH_RVec3 axisVec = {
    .x = axis[0],
    .y = axis[1],
    .z = axis[2]
  };
  JPH_SliderConstraintSettings_SetSliderAxis(settings, &axisVec);
  joint->constraint = (JPH_Constraint *) JPH_SliderConstraintSettings_CreateConstraint(settings, a->body, b->body);
  JPH_PhysicsSystem_AddConstraint(a->world->physics_system, joint->constraint);
  arr_push(&a->joints, joint);
  arr_push(&b->joints, joint);
  lovrRetain(joint);
  return joint;
}

void lovrSliderJointGetAxis(SliderJoint* joint, float axis[3]) {}

void lovrSliderJointSetAxis(SliderJoint* joint, float axis[3]) {}

float lovrSliderJointGetPosition(SliderJoint* joint) {
  return JPH_SliderConstraint_GetCurrentPosition((JPH_SliderConstraint *) joint->constraint);
}

float lovrSliderJointGetLowerLimit(SliderJoint* joint) {
  return JPH_SliderConstraint_GetLimitsMin((JPH_SliderConstraint *) joint->constraint);
}

void lovrSliderJointSetLowerLimit(SliderJoint* joint, float limit) {
  float upper_limit = JPH_SliderConstraint_GetLimitsMax((JPH_SliderConstraint *) joint->constraint);
  JPH_SliderConstraint_SetLimits((JPH_SliderConstraint *) joint->constraint, limit, upper_limit);
}

float lovrSliderJointGetUpperLimit(SliderJoint* joint) {
  return JPH_SliderConstraint_GetLimitsMax((JPH_SliderConstraint *) joint->constraint);
}

void lovrSliderJointSetUpperLimit(SliderJoint* joint, float limit) {
  float lower_limit = JPH_SliderConstraint_GetLimitsMin((JPH_SliderConstraint *) joint->constraint);
  JPH_SliderConstraint_SetLimits((JPH_SliderConstraint *) joint->constraint, lower_limit, limit);
}
