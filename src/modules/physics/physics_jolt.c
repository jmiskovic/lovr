#include <stdlib.h>
#include "util.h"
#include "physics.h"
#include "JoltPhysicsC.h"

static JPC_TempAllocator *temp_allocator;
static JPC_JobSystem *job_system;
JPC_BodyInterface * body_interface; // todo

struct World {
  JPC_PhysicsSystem *physics_system;
  JPC_BodyInterface *body_interface;
};

struct Collider {
  JPC_BodyID id;
  JPC_Body *body;
};

struct Shape {
};

// Object layers
#define NUM_OBJ_LAYERS 2
#define OBJ_LAYER_NON_MOVING 0
#define OBJ_LAYER_MOVING 1

// Broad phase layers
#define NUM_BP_LAYERS 2
#define BP_LAYER_NON_MOVING 0
#define BP_LAYER_MOVING 1

typedef struct BPLayerInterfaceImpl
{
    const JPC_BroadPhaseLayerInterfaceVTable *vtable; // VTable has to be the first field in the struct.
    JPC_BroadPhaseLayer                       object_to_broad_phase[NUM_OBJ_LAYERS];
} BPLayerInterfaceImpl;


static uint32_t
BPLayerInterface_GetNumBroadPhaseLayers(const void *in_self)
{
    return NUM_BP_LAYERS;
}


static JPC_BroadPhaseLayer
BPLayerInterface_GetBroadPhaseLayer(const void *in_self, JPC_ObjectLayer in_layer)
{
  lovrAssert(in_layer < NUM_BP_LAYERS, "Broad-phase layer out of range");
  const BPLayerInterfaceImpl *self = (BPLayerInterfaceImpl *)in_self;
  return self->object_to_broad_phase[in_layer];
}


static BPLayerInterfaceImpl
BPLayerInterface_Create(void)
{
  static const JPC_BroadPhaseLayerInterfaceVTable vtable =
  {
    .GetNumBroadPhaseLayers = BPLayerInterface_GetNumBroadPhaseLayers,
    .GetBroadPhaseLayer   = BPLayerInterface_GetBroadPhaseLayer,
  };
  BPLayerInterfaceImpl impl =
  {
    .vtable = &vtable,
  };
  impl.object_to_broad_phase[OBJ_LAYER_NON_MOVING] = BP_LAYER_NON_MOVING;
  impl.object_to_broad_phase[OBJ_LAYER_MOVING]   = BP_LAYER_MOVING;

  return impl;
}



typedef struct MyObjectFilter
{
    const JPC_ObjectLayerPairFilterVTable *vtable; // VTable has to be the first field in the struct.
} MyObjectFilter;


static bool
MyObjectFilter_ShouldCollide(const void *in_self, JPC_ObjectLayer in_object1, JPC_ObjectLayer in_object2)
{
    switch (in_object1)
    {
        case OBJ_LAYER_NON_MOVING:
            return in_object2 == OBJ_LAYER_MOVING;
        case OBJ_LAYER_MOVING:
            return true;
        default:
            lovrAssert(false, "Collision check between unknown layers");
            return false;
    }
}

static MyObjectFilter
MyObjectFilter_Create(void)
{
    static const JPC_ObjectLayerPairFilterVTable vtable =
    {
        .ShouldCollide = MyObjectFilter_ShouldCollide,
    };
    MyObjectFilter impl =
    {
        .vtable = &vtable,
    };
    return impl;
}



typedef struct MyBroadPhaseFilter
{
    const JPC_ObjectVsBroadPhaseLayerFilterVTable *vtable; // VTable has to be the first field in the struct.
} MyBroadPhaseFilter;

static bool
MyBroadPhaseFilter_ShouldCollide(const void *in_self, JPC_ObjectLayer in_layer1, JPC_BroadPhaseLayer in_layer2)
{
    switch (in_layer1)
    {
        case OBJ_LAYER_NON_MOVING:
            return in_layer2 == BP_LAYER_MOVING;
        case OBJ_LAYER_MOVING:
            return true;
        default:
            lovrAssert(false, "Broad-phase collision check between unknown layers");
            return false;
    }
}


static MyBroadPhaseFilter
MyBroadPhaseFilter_Create(void)
{
    static const JPC_ObjectVsBroadPhaseLayerFilterVTable vtable =
    {
        .ShouldCollide = MyBroadPhaseFilter_ShouldCollide,
    };
    MyBroadPhaseFilter impl =
    {
        .vtable = &vtable,
    };
    return impl;
}




bool lovrPhysicsInit(void) {
  JPC_RegisterDefaultAllocator();
  JPC_CreateFactory();
  JPC_RegisterTypes();
  temp_allocator = JPC_TempAllocator_Create(32 * 1024 * 1024);
  job_system = JPC_JobSystem_Create(JPC_MAX_PHYSICS_JOBS, JPC_MAX_PHYSICS_BARRIERS, -1);
}

void lovrPhysicsDestroy(void) {}

World* lovrWorldCreate(float xg, float yg, float zg, bool allowSleep, const char** tags, uint32_t tagCount) {
  World* world = calloc(1, sizeof(World));
  lovrAssert(world, "Out of memory");
  const uint32_t max_bodies = 10240;
  const uint32_t num_body_mutexes = 0; // zero is auto-detect
  const uint32_t max_body_pairs = 65536;
  const uint32_t max_contact_constraints = 20480;

  BPLayerInterfaceImpl *broad_phase_layer_interface = malloc(sizeof(BPLayerInterfaceImpl));
  *broad_phase_layer_interface = BPLayerInterface_Create();

  MyBroadPhaseFilter *broad_phase_filter = malloc(sizeof(MyBroadPhaseFilter));
  *broad_phase_filter = MyBroadPhaseFilter_Create();

  MyObjectFilter *object_filter = malloc(sizeof(MyObjectFilter));
  *object_filter = MyObjectFilter_Create();

  world->physics_system = JPC_PhysicsSystem_Create(
    max_bodies,
    num_body_mutexes,
    max_body_pairs,
    max_contact_constraints,
    broad_phase_layer_interface,
    broad_phase_filter,
    object_filter);
  world->body_interface = JPC_PhysicsSystem_GetBodyInterface(world->physics_system);
  body_interface = world->body_interface;
  float const gravity[3] = {0.f, 0.f, 0.f};
  JPC_PhysicsSystem_SetGravity(world->physics_system , gravity);

  return world;
}

void lovrWorldDestroy(void* ref) {}

void lovrWorldDestroyData(World* world) {}

void lovrWorldUpdate(World* world, float dt, CollisionResolver resolver, void* userdata) {
  JPC_PhysicsUpdateError update_err = JPC_PhysicsSystem_Update(
    world->physics_system,
    dt,
    1,
    1,
    temp_allocator,
    job_system);
}

int lovrWorldGetStepCount(World* world) {}

void lovrWorldSetStepCount(World* world, int iterations) {}

void lovrWorldComputeOverlaps(World* world) {}

int lovrWorldGetNextOverlap(World* world, Shape** a, Shape** b) {}

int lovrWorldCollide(World* world, Shape* a, Shape* b, float friction, float restitution) {}

void lovrWorldGetContacts(World* world, Shape* a, Shape* b, Contact contacts[MAX_CONTACTS], uint32_t* count) {}

void lovrWorldRaycast(World* world, float x1, float y1, float z1, float x2, float y2, float z2, RaycastCallback callback, void* userdata) {}

bool lovrWorldQueryBox(World* world, float position[3], float size[3], QueryCallback callback, void* userdata) {}

bool lovrWorldQuerySphere(World* world, float position[3], float radius, QueryCallback callback, void* userdata) {}

Collider* lovrWorldGetFirstCollider(World* world) {}

void lovrWorldGetGravity(World* world, float* x, float* y, float* z) {}

void lovrWorldSetGravity(World* world, float x, float y, float z) {}

float lovrWorldGetResponseTime(World* world) {}

void lovrWorldSetResponseTime(World* world, float responseTime) {}

float lovrWorldGetTightness(World* world) {}

void lovrWorldSetTightness(World* world, float tightness) {}

void lovrWorldGetLinearDamping(World* world, float* damping, float* threshold) {}

void lovrWorldSetLinearDamping(World* world, float damping, float threshold) {}

void lovrWorldGetAngularDamping(World* world, float* damping, float* threshold) {}

void lovrWorldSetAngularDamping(World* world, float damping, float threshold) {}

bool lovrWorldIsSleepingAllowed(World* world) {}

void lovrWorldSetSleepingAllowed(World* world, bool allowed) {}

const char* lovrWorldGetTagName(World* world, uint32_t tag) {}

void lovrWorldDisableCollisionBetween(World* world, const char* tag1, const char* tag2) {}

void lovrWorldEnableCollisionBetween(World* world, const char* tag1, const char* tag2) {}

bool lovrWorldIsCollisionEnabledBetween(World* world, const char* tag1, const char* tag2) {}

Collider* lovrColliderCreate(World* world, float x, float y, float z) {
  Collider* collider = calloc(1, sizeof(Collider));
  lovrAssert(collider, "Out of memory");
  float w = .2f;
  float h = .2f;
  float d = .2f;
  JPC_BoxShapeSettings *box_shape_settings = JPC_BoxShapeSettings_Create((float[]){ w, h, d });
  JPC_Shape *box_shape = JPC_ShapeSettings_CreateShape((JPC_ShapeSettings *)box_shape_settings);

  JPC_BodyCreationSettings box_settings;
  JPC_BodyCreationSettings_Set(
      &box_settings,
      box_shape,
      (JPC_Real[]){ x, y, z },
      (float[]){ 0.0f, 0.0f, 0.0f, 1.0f },
      JPC_MOTION_TYPE_DYNAMIC,
      OBJ_LAYER_MOVING);

  JPC_Body *box = JPC_BodyInterface_CreateBody(world->body_interface, &box_settings);
  const JPC_BodyID box_id = JPC_Body_GetID(box);
  JPC_BodyInterface_AddBody(world->body_interface, box_id, JPC_ACTIVATION_ACTIVATE);
  collider->id = box_id;
  collider->body = box;

  JPC_PhysicsSystem_OptimizeBroadPhase(world->physics_system); // todo!
  const float force[3] = {1.f, 1.f, 0.f};
  JPC_Body_AddForce(box, force);
  return collider;
}

void lovrColliderDestroy(void* ref) {}

void lovrColliderDestroyData(Collider* collider) {}

bool lovrColliderIsDestroyed(Collider* collider) {}

void lovrColliderInitInertia(Collider* collider, Shape* shape) {}

World* lovrColliderGetWorld(Collider* collider) {}

Collider* lovrColliderGetNext(Collider* collider) {}

void lovrColliderAddShape(Collider* collider, Shape* shape) {}

void lovrColliderRemoveShape(Collider* collider, Shape* shape) {}

Shape** lovrColliderGetShapes(Collider* collider, size_t* count) {}

Joint** lovrColliderGetJoints(Collider* collider, size_t* count) {}

void* lovrColliderGetUserData(Collider* collider) {}

void lovrColliderSetUserData(Collider* collider, void* data) {}

const char* lovrColliderGetTag(Collider* collider) {}

bool lovrColliderSetTag(Collider* collider, const char* tag) {}

float lovrColliderGetFriction(Collider* collider) {}

void lovrColliderSetFriction(Collider* collider, float friction) {}

float lovrColliderGetRestitution(Collider* collider) {}

void lovrColliderSetRestitution(Collider* collider, float restitution) {}

bool lovrColliderIsKinematic(Collider* collider) {}

void lovrColliderSetKinematic(Collider* collider, bool kinematic) {}

bool lovrColliderIsGravityIgnored(Collider* collider) {}


void lovrColliderSetGravityIgnored(Collider* collider, bool ignored) {}

bool lovrColliderIsSleepingAllowed(Collider* collider) {}

void lovrColliderSetSleepingAllowed(Collider* collider, bool allowed) {}

bool lovrColliderIsAwake(Collider* collider) {}

void lovrColliderSetAwake(Collider* collider, bool awake) {}

float lovrColliderGetMass(Collider* collider) {}

void lovrColliderSetMass(Collider* collider, float mass) {}

void lovrColliderGetMassData(Collider* collider, float* cx, float* cy, float* cz, float* mass, float inertia[6]) {}

void lovrColliderSetMassData(Collider* collider, float cx, float cy, float cz, float mass, float inertia[6]) {}

void lovrColliderGetPosition(Collider* collider, float* x, float* y, float* z)
{
  JPC_Real position[3];
  JPC_Body_GetPosition(collider->body, &position[0]);
  *x = position[0];
  *y = position[1];
  *z = position[2];
}

void lovrColliderSetPosition(Collider* collider, float x, float y, float z) {}

void lovrColliderGetOrientation(Collider* collider, float* orientation) {
  JPC_BodyInterface_GetRotation(body_interface, collider->id, orientation);
}

void lovrColliderSetOrientation(Collider* collider, float* orientation) {}

void lovrColliderGetLinearVelocity(Collider* collider, float* x, float* y, float* z) {}

void lovrColliderSetLinearVelocity(Collider* collider, float x, float y, float z) {}

void lovrColliderGetAngularVelocity(Collider* collider, float* x, float* y, float* z) {}

void lovrColliderSetAngularVelocity(Collider* collider, float x, float y, float z) {}

void lovrColliderGetLinearDamping(Collider* collider, float* damping, float* threshold) {}

void lovrColliderSetLinearDamping(Collider* collider, float damping, float threshold) {}

void lovrColliderGetAngularDamping(Collider* collider, float* damping, float* threshold) {}

void lovrColliderSetAngularDamping(Collider* collider, float damping, float threshold) {}

void lovrColliderApplyForce(Collider* collider, float x, float y, float z) {}

void lovrColliderApplyForceAtPosition(Collider* collider, float x, float y, float z, float cx, float cy, float cz) {}

void lovrColliderApplyTorque(Collider* collider, float x, float y, float z) {}

void lovrColliderGetLocalCenter(Collider* collider, float* x, float* y, float* z) {}

void lovrColliderGetLocalPoint(Collider* collider, float wx, float wy, float wz, float* x, float* y, float* z) {}

void lovrColliderGetWorldPoint(Collider* collider, float x, float y, float z, float* wx, float* wy, float* wz) {}

void lovrColliderGetLocalVector(Collider* collider, float wx, float wy, float wz, float* x, float* y, float* z) {}

void lovrColliderGetWorldVector(Collider* collider, float x, float y, float z, float* wx, float* wy, float* wz) {}

void lovrColliderGetLinearVelocityFromLocalPoint(Collider* collider, float x, float y, float z, float* vx, float* vy, float* vz) {}

void lovrColliderGetLinearVelocityFromWorldPoint(Collider* collider, float wx, float wy, float wz, float* vx, float* vy, float* vz) {}

void lovrColliderGetAABB(Collider* collider, float aabb[6]) {}


void lovrShapeDestroy(void* ref) {}

void lovrShapeDestroyData(Shape* shape) {}

ShapeType lovrShapeGetType(Shape* shape) {}

Collider* lovrShapeGetCollider(Shape* shape) {}

bool lovrShapeIsEnabled(Shape* shape) {}

void lovrShapeSetEnabled(Shape* shape, bool enabled) {}

bool lovrShapeIsSensor(Shape* shape) {}

void lovrShapeSetSensor(Shape* shape, bool sensor) {}

void* lovrShapeGetUserData(Shape* shape) {}

void lovrShapeSetUserData(Shape* shape, void* data) {}

void lovrShapeGetPosition(Shape* shape, float* x, float* y, float* z) {}

void lovrShapeSetPosition(Shape* shape, float x, float y, float z) {}

void lovrShapeGetOrientation(Shape* shape, float* orientation) {}

void lovrShapeSetOrientation(Shape* shape, float* orientation) {}

void lovrShapeGetMass(Shape* shape, float density, float* cx, float* cy, float* cz, float* mass, float inertia[6]) {}

void lovrShapeGetAABB(Shape* shape, float aabb[6]) {}

SphereShape* lovrSphereShapeCreate(float radius) {}

float lovrSphereShapeGetRadius(SphereShape* sphere) {}

void lovrSphereShapeSetRadius(SphereShape* sphere, float radius) {}

BoxShape* lovrBoxShapeCreate(float w, float h, float d) {
  BoxShape* box = calloc(1, sizeof(BoxShape));
  return box;
}



void lovrBoxShapeGetDimensions(BoxShape* box, float* w, float* h, float* d) {}

void lovrBoxShapeSetDimensions(BoxShape* box, float w, float h, float d) {}

CapsuleShape* lovrCapsuleShapeCreate(float radius, float length) {}

float lovrCapsuleShapeGetRadius(CapsuleShape* capsule) {}

void lovrCapsuleShapeSetRadius(CapsuleShape* capsule, float radius) {}

float lovrCapsuleShapeGetLength(CapsuleShape* capsule) {}

void lovrCapsuleShapeSetLength(CapsuleShape* capsule, float length) {}

CylinderShape* lovrCylinderShapeCreate(float radius, float length) {}

float lovrCylinderShapeGetRadius(CylinderShape* cylinder) {}

void lovrCylinderShapeSetRadius(CylinderShape* cylinder, float radius) {}

float lovrCylinderShapeGetLength(CylinderShape* cylinder) {}

void lovrCylinderShapeSetLength(CylinderShape* cylinder, float length) {}

MeshShape* lovrMeshShapeCreate(int vertexCount, float vertices[], int indexCount, uint32_t indices[]) {}

TerrainShape* lovrTerrainShapeCreate(float* vertices, uint32_t widthSamples, uint32_t depthSamples, float horizontalScale, float verticalScale) {}

void lovrJointDestroy(void* ref) {}

void lovrJointDestroyData(Joint* joint) {}

JointType lovrJointGetType(Joint* joint) {}

void lovrJointGetColliders(Joint* joint, Collider** a, Collider** b) {}

void* lovrJointGetUserData(Joint* joint) {}

void lovrJointSetUserData(Joint* joint, void* data) {}

bool lovrJointIsEnabled(Joint* joint) {}

void lovrJointSetEnabled(Joint* joint, bool enable) {}

BallJoint* lovrBallJointCreate(Collider* a, Collider* b, float anchor[3]) {}

void lovrBallJointGetAnchors(BallJoint* joint, float anchor1[3], float anchor2[3]) {}

void lovrBallJointSetAnchor(BallJoint* joint, float anchor[3]) {}

float lovrBallJointGetResponseTime(Joint* joint) {}

void lovrBallJointSetResponseTime(Joint* joint, float responseTime) {}

float lovrBallJointGetTightness(Joint* joint) {}

void lovrBallJointSetTightness(Joint* joint, float tightness) {}

DistanceJoint* lovrDistanceJointCreate(Collider* a, Collider* b, float anchor1[3], float anchor2[3]) {}

void lovrDistanceJointGetAnchors(DistanceJoint* joint, float anchor1[3], float anchor2[3]) {}

void lovrDistanceJointSetAnchors(DistanceJoint* joint, float anchor1[3], float anchor2[3]) {}

float lovrDistanceJointGetDistance(DistanceJoint* joint) {}

void lovrDistanceJointSetDistance(DistanceJoint* joint, float distance) {}

float lovrDistanceJointGetResponseTime(Joint* joint) {}

void lovrDistanceJointSetResponseTime(Joint* joint, float responseTime) {}

float lovrDistanceJointGetTightness(Joint* joint) {}

void lovrDistanceJointSetTightness(Joint* joint, float tightness) {}

HingeJoint* lovrHingeJointCreate(Collider* a, Collider* b, float anchor[3], float axis[3]) {}

void lovrHingeJointGetAnchors(HingeJoint* joint, float anchor1[3], float anchor2[3]) {}

void lovrHingeJointSetAnchor(HingeJoint* joint, float anchor[3]) {}

void lovrHingeJointGetAxis(HingeJoint* joint, float axis[3]) {}

void lovrHingeJointSetAxis(HingeJoint* joint, float axis[3]) {}

float lovrHingeJointGetAngle(HingeJoint* joint) {}

float lovrHingeJointGetLowerLimit(HingeJoint* joint) {}

void lovrHingeJointSetLowerLimit(HingeJoint* joint, float limit) {}

float lovrHingeJointGetUpperLimit(HingeJoint* joint) {}

void lovrHingeJointSetUpperLimit(HingeJoint* joint, float limit) {}

SliderJoint* lovrSliderJointCreate(Collider* a, Collider* b, float axis[3]) {}

void lovrSliderJointGetAxis(SliderJoint* joint, float axis[3]) {}

void lovrSliderJointSetAxis(SliderJoint* joint, float axis[3]) {}

float lovrSliderJointGetPosition(SliderJoint* joint) {}

float lovrSliderJointGetLowerLimit(SliderJoint* joint) {}

void lovrSliderJointSetLowerLimit(SliderJoint* joint, float limit) {}

float lovrSliderJointGetUpperLimit(SliderJoint* joint) {}

void lovrSliderJointSetUpperLimit(SliderJoint* joint, float limit) {}
