#include <jni.h>
#include <android/asset_manager_jni.h>
#include <string>
#include "flecs.h"
#include <vector>
#include <android/log.h>
#include "box2d/include/box2d/box2d.h"
#include <unordered_map>
#include <iostream>
#include <time.h>
#include <stdlib.h>

#include "nanoflann.hpp"
#include "utils.h"
#include "fastnoiselite.h"

// Android Game SDK
//#include "swappy/swappyGL.h"

// https://developer.android.com/games/agdk/game-activity/get-started
//#include <game-activity/native_app_glue/android_native_app_glue.h>
//
//extern "C" {
//void android_main(struct android_app* state);
//};
//
//void android_main(struct android_app* app) {
//    NativeEngine *engine = new NativeEngine(app);
//    engine->GameLoop();
//    delete engine;
//}
//
//void NativeEngine::GameLoop() {
//    mApp->userData = this;
//    mApp->onAppCmd = _handle_cmd_proxy;  // register your command handler.
//    mApp->textInputState = 0;
//
//    while (1) {
//        int events;
//        struct android_poll_source* source;
//
//        // If not animating, block until we get an event;
//        // If animating, don't block.
//        while ((ALooper_pollAll(IsAnimating() ? 0 : -1, NULL, &events,
//                                (void **) &source)) >= 0) {
//            if (source != NULL) {
//                // process events, native_app_glue internally sends the outstanding
//                // application lifecycle events to mApp->onAppCmd.
//                source->process(source->app, source);
//            }
//            if (mApp->destroyRequested) {
//                return;
//            }
//        }
//        if (IsAnimating()) {
//            DoFrame();
//        }
//    }
//}

const float PIXELS_PER_METER = 16.0f;

extern "C" JNIEXPORT jstring JNICALL
Java_com_example_graphlife_MainActivity_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
};

struct GraphNode
{
    float x;
    float y;

    // Default constructor
    GraphNode() : x(0.0f), y(0.0f) {}

    // Constructor with parameters
    GraphNode(float _x, float _y) : x(_x), y(_y) {}

    size_t operator()(const GraphNode& node) const
    {
        size_t hashX = std::hash<float>{}(node.x);
        size_t hashY = std::hash<float>{}(node.y);
        return hashX ^ (hashY << 1);
    }

    bool operator==(const GraphNode& other) const
    {
        return x == other.x && y == other.y;
    }
};

struct NodeRenderState
{
    float radius;
};

struct GraphNodeHasher
{
    std::size_t operator()(const GraphNode& node) const
    {
        return node(node);
    }
};

struct NodeSnapDestination // Create a distance joint
{
    float radius;
    int enabled = true;
};

// TODO: Implement as relationship (pair) from head entity to tail entity?
//struct GraphEdge {
//    flecs::entity head; // TODO: ref<GraphNode>?
//    flecs::entity tail;
//};

struct Edge
{
    float head_x;
    float head_y;
    float tail_x;
    float tail_y;
    float Magnitude() const
    {
        return sqrt(pow(tail_x - head_x, 2) + pow(tail_y - head_y, 2));
    }
};

struct EdgeJoint
{
    b2Joint* joint;
};

struct ConicEdgeMod
{
    float mid_x;
    float mid_y;
    float weight;
};

struct CubicEdgeMod
{
    float mid_a_x;
    float mid_a_y;
    float mid_b_x;
    float mid_b_y;
};

struct EdgePath {
    float head_x;
    float head_y;
    float tail_x;
    float tail_y;
    int edge_type;
    float val_0;
    float val_1;
    float val_2;
    float val_3;
};

struct World
{
    b2World* physics;
    std::vector<b2Body*> bodyRemoveQueue;
    std::vector<b2Joint*> jointRemoveQueue;
};

struct PhysicalNodeSim
{
    b2Body* body;
    float start_spring_dist;
};
struct NodeAnchor
{
    int mock;
};

// TODO: Need a way to go from Kotlin MotionEvent to player movement
struct JoyDiff
{
    float x;
    float y;
};

struct Player
{
    b2Body* body;
    float minForce;
    float maxForce;
    float accelerationRate; // per pixel diff
    float minMoveSpeed;
    float maxMoveSpeed;
    float deccelerationRate;

    flecs::entity lastSnap;
};

enum struct CameraControlMode
{
    FOLLOW_PLAYER,
    FIXED
};

// TODO: Consider UNSNAP in a particular direction or 'pull event?'
enum struct SnapEventType
{
    SNAP = 0,
    UNSNAP = 1,
};

struct GraphEvent
{
    int snapEvent;
    const char* name;
};

struct CameraControl
{
    CameraControlMode mode;
};

struct FixedCamera
{
    b2Vec2 worldPos;
};

struct CameraInterpolator
{
    b2Vec2 origin;
    b2Vec2 target;
    float progress;
};

struct UIState
{
    bool menuVisible;
    bool mapVisible;
    std::vector<std::string> activatedMenuItems;
};

struct GraphViewState
{
    std::unordered_set<std::string>* hiddenCategories;
};

struct DistanceJointCreator
{
    b2DistanceJointDef distanceJointDef;
};

struct TriggerPlaidIntegration {
    int mock;
};

struct GraphInterpolator
{
    GraphInterpolator()
    {

    }
    GraphInterpolator(float p, float rate) {
        this->p = p;
        this->rate = rate;
    }

    float p;
    float rate;
};

struct NoiseSource
{
    FastNoiseLite* noise;
    int wanderers;
};

struct Wanderer
{
    b2Vec2 noisePos; //  = b2Vec2(0.0f, 0.0f);
};

struct Equivariant {};

struct GraphPathInterpolator
{
    // TODO
};

struct CreateSpringEdgeFromConfig {};
struct CreateSpringEdge {
    flecs::entity nodeTarget;
};
struct CreateSpringEdgeData
{
    float strength;
};
struct SpringEdge
{
    flecs::entity entity;  // child of GraphNode
};

struct DiEdge {};
struct Snapped
{
    b2Body* body; // may be temporary...
    b2DistanceJoint* spring;
};

struct CreateEdgeData {
    Edge data;
    flecs::entity config;
};

// TODO: Map this into enum/int from user created config file
struct GraphNodeCategory{
    const char* name;
};

struct GraphController
{
    // TODO: Graph bijection
    // Interpolate to target state by interpolation of system influence
};

// TODO: Physical spring simulation and animation interpolation
// TODO: scipy.sparse.coo_array vs SNAP vs PyG vs Flecs graph representations
// Start with Flecs as the graph representation, as it will provide flexibility in terms of systems

flecs::world ecs;

extern "C" JNIEXPORT void JNICALL
Java_com_example_graphlife_MainActivity_setJoyDiff(JNIEnv* env, jobject /* this */, float x, float y)
{
    JoyDiff* joy = ecs.lookup("player_controller").get_mut<JoyDiff>();
    joy->x = x;
    joy->y = y;
}

extern "C" JNIEXPORT jobject JNICALL
Java_com_example_graphlife_MainActivity_getPlayerPos(JNIEnv* env, jobject /* this */)
{
    Player* player = ecs.lookup("player").get_mut<Player>();
    b2Vec2 playerWorldPos = player->body->GetPosition();
    __android_log_print(ANDROID_LOG_DEBUG, "GraphFix", "Player pos (%f, %f)!", playerWorldPos.x, playerWorldPos.y);
    b2Vec2 playerScreenPos = playerWorldPos;
    playerScreenPos *= PIXELS_PER_METER;
    jclass nodeClass = env->FindClass("com/example/graphlife/GraphNode");
    jmethodID constructor = env->GetMethodID(nodeClass, "<init>", "(FFF)V");
    jobject javaNode = env->NewObject(nodeClass, constructor, playerScreenPos.x, playerScreenPos.y, 0.0f);
    return javaNode;
}

extern "C" JNIEXPORT jobject JNICALL
Java_com_example_graphlife_MainActivity_getCameraPos(JNIEnv* env, jobject /* this */)
{
    Player* player = ecs.lookup("player").get_mut<Player>();
    flecs::entity playerController = ecs.lookup("player_controller");
    const CameraControl* cameraControl = playerController.get<CameraControl>();
    const FixedCamera* fixedCamera = playerController.get<FixedCamera>();
    jclass nodeClass = env->FindClass("com/example/graphlife/GraphNode");
    jmethodID constructor = env->GetMethodID(nodeClass, "<init>", "(FFF)V");
    if (cameraControl->mode == CameraControlMode::FOLLOW_PLAYER)
    {
        Player* player = ecs.lookup("player").get_mut<Player>();
        b2Vec2 playerWorldPos = player->body->GetPosition();
        b2Vec2 playerScreenPos = playerWorldPos;
        playerScreenPos *= PIXELS_PER_METER;
        jobject javaNode = env->NewObject(nodeClass, constructor, playerScreenPos.x, playerScreenPos.y, 0.0f);
        return javaNode;
    }
    jobject javaNode = env->NewObject(nodeClass, constructor, fixedCamera->worldPos.x, fixedCamera->worldPos.y, 0.0f);
    return javaNode;
}

extern "C" JNIEXPORT jobject JNICALL
Java_com_example_graphlife_MainActivity_getUIUpdates(JNIEnv* env, jobject /* this */)
{
    jclass nodeClass = env->FindClass("com/example/graphlife/UIState");
    jmethodID constructor = env->GetMethodID(nodeClass, "<init>", "(Ljava/util/HashSet;ZZZ)V");
    auto uiState = ecs.lookup("ui_controller").get<UIState>();

    jclass hashSetClass = env->FindClass("java/util/HashSet");
    jmethodID hashSetConstructor = env->GetMethodID(hashSetClass, "<init>", "()V");
    jobject javaHashSet = env->NewObject(hashSetClass, hashSetConstructor);
    jmethodID hashSetAddMethod = env->GetMethodID(hashSetClass, "add", "(Ljava/lang/Object;)Z");
    jstring javaString = env->NewStringUTF("spacetime");
    env->CallBooleanMethod(javaHashSet, hashSetAddMethod, javaString);

    bool triggerPlaid = false;
    if (ecs.lookup("ui_controller").has<TriggerPlaidIntegration>())
    {
        triggerPlaid = true;
        ecs.lookup("ui_controller").remove<TriggerPlaidIntegration>();
    }

    jobject javaUIState = env->NewObject(nodeClass, constructor, javaHashSet, uiState->menuVisible, uiState->mapVisible, triggerPlaid);
    return javaUIState;
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_graphlife_MainActivity_setUIMenuOffset(JNIEnv* env, jobject /* this */, jfloat x, jfloat y)
{
    ecs.lookup("player_controller").set<FixedCamera>({b2Vec2(x, y)});
}

extern "C" JNIEXPORT jint JNICALL
Java_com_example_graphlife_MainActivity_stepWorld(JNIEnv* env, jobject /* this */) {
    ecs.progress(16.0f); // TODO: Better loop! 100/16.0f
    return 0;
}

float linearInterpolation(float start, float end, float progress) {
    if (start >= end)
    {
        return end;
    }
    return start * (1.0f - progress) + (end * progress);
};

// TODO: Consider just making categories pure flecs tags rather than strings :)
void enable_nodes_w_category(std::string cat)
{
    auto gvs = ecs.lookup("ui_controller").get_mut<GraphViewState>();
    gvs->hiddenCategories->erase(cat);
    auto q = ecs.query_builder<GraphNodeCategory, NodeSnapDestination>().build();
    ecs.defer_begin();
    q.iter([cat](flecs::iter& it, GraphNodeCategory* nodeCat, NodeSnapDestination* destiny) {
        for (auto i : it)
        {
            if (strcmp(nodeCat[i].name, cat.c_str()) == 0)
            {
                destiny[i].enabled = true;
            }
        }
    });
    ecs.defer_end();
}

void disable_nodes_w_category(std::string cat)
{
    auto gvs = ecs.lookup("ui_controller").get_mut<GraphViewState>();
    gvs->hiddenCategories->insert(cat);
    auto q = ecs.query_builder<GraphNodeCategory, NodeSnapDestination>().build();
    ecs.defer_begin();
    q.iter([cat](flecs::iter& it, GraphNodeCategory* nodeCat, NodeSnapDestination* destiny) {
        for (auto i : it)
        {
            if (strcmp(nodeCat[i].name, cat.c_str()) == 0)
            {
                destiny[i].enabled = false;
                if (it.entity(i).has<Snapped>())
                {
                    it.entity(i).remove<Snapped>();
                }
            }
        }
    });
    ecs.defer_end();
}

extern "C" JNIEXPORT jint JNICALL
Java_com_example_graphlife_MainActivity_initWorld(JNIEnv* env, jobject /* this */,  jobject assetManager) {
    ECS_COMPONENT(ecs, GraphNode);
    ECS_COMPONENT(ecs, Edge);
    ECS_COMPONENT(ecs, ConicEdgeMod);
    ECS_COMPONENT(ecs, CubicEdgeMod);
    ECS_COMPONENT(ecs, World);
    ECS_COMPONENT(ecs, PhysicalNodeSim);
    ECS_COMPONENT(ecs, GraphInterpolator);
    ECS_COMPONENT(ecs, NodeSnapDestination);
    ECS_COMPONENT(ecs, CreateSpringEdge);
    ECS_COMPONENT(ecs, CreateSpringEdgeData);
    ECS_TAG(ecs, Equivariant);
    ECS_TAG(ecs, CreateSpringEdgeFromConfig);
    ECS_COMPONENT(ecs, Snapped);
    ECS_COMPONENT(ecs, FixedCamera);
    ECS_COMPONENT(ecs, NodeAnchor);
    ECS_COMPONENT(ecs, CameraInterpolator);
    ECS_COMPONENT(ecs, GraphEvent);
    ECS_COMPONENT(ecs, GraphNodeCategory);
    ECS_COMPONENT(ecs, GraphViewState);
    ECS_COMPONENT(ecs, NodeRenderState);
    ECS_COMPONENT(ecs, Wanderer);
    ECS_COMPONENT(ecs, NoiseSource);
    ECS_TAG(ecs, DiEdge);
    ECS_COMPONENT(ecs, TriggerPlaidIntegration);

    srand (time(NULL));

//    __android_log_print(ANDROID_LOG_DEBUG, "GetPhysicalNode", "initWorld");

    ecs.observer<World>("OnAddWorld").event(flecs::OnAdd).each([](World& world) {
        world.physics = new b2World();
//        b2Vec2 gravity = b2Vec2(0.0f, 10.0f);
//        world.physics->SetGravity(gravity);
    });

    auto e = ecs.entity("world");
    e.add<World>();

    ecs.observer<Player, World>("OnAddPlayer").event(flecs::OnAdd).term_at(2).src(e).instanced().each([](Player& player, World& world) {
        player.minForce = 0.0f;
        player.maxForce = 0.03f;
        player.accelerationRate = 0.00036f;
        player.minMoveSpeed = 0.01f;
        player.maxMoveSpeed = 0.01f; // per diff length
        player.deccelerationRate = 0.15f;
        b2BodyDef bodyDef;
        bodyDef.position.Set(0.0f, 0.0f);
        bodyDef.type = b2_dynamicBody;
        player.body = world.physics->CreateBody(&bodyDef);
        b2CircleShape circleShape;
        circleShape.m_radius = 1.0f;
        b2FixtureDef fixtureDef;
        fixtureDef.shape = &circleShape;
        fixtureDef.density = 1.0f;
        player.body->CreateFixture(&fixtureDef);
    });

    auto playerController = ecs.entity("player_controller");
    playerController.add<JoyDiff>();
    playerController.set<CameraControl>({CameraControlMode::FOLLOW_PLAYER});
    playerController.add<FixedCamera>();

    auto uiController = ecs.entity("ui_controller");
    uiController.set<UIState>({false});
    uiController.add<GraphViewState>();
    auto gvs = uiController.get_mut<GraphViewState>();
    gvs->hiddenCategories = new std::unordered_set<std::string>();
    disable_nodes_w_category("ui");
    uiController.add<NoiseSource>();
    auto source = uiController.get_mut<NoiseSource>();
    source->noise = new FastNoiseLite();
    source->noise->SetNoiseType(FastNoiseLite::NoiseType_OpenSimplex2);

    auto player = ecs.entity("player");
    player.add<Player>();

    ecs.observer<GraphNode, PhysicalNodeSim, World>("GenPhysicalNode").term_at(3).src(e).instanced().event(flecs::OnAdd).iter([](flecs::iter& it, GraphNode* gn, PhysicalNodeSim* pns, World* world)
    {
        for (auto i : it)
        {
            b2BodyDef bodyDef;
            bodyDef.position.Set(gn->x/PIXELS_PER_METER, gn->y/PIXELS_PER_METER); // TODO: Should be a radial uniform distribution generated anew each time
//        bodyDef.type = b2_staticBody;
            if (it.entity(i).has<NodeAnchor>())
            {
                __android_log_print(ANDROID_LOG_DEBUG, "GraphConfig", "Anchor/static body created");
                bodyDef.type = b2_staticBody;
            } else
            {
                bodyDef.type = b2_dynamicBody;
            }
            pns->body = world->physics->CreateBody(&bodyDef);
            b2CircleShape circleShape;
            circleShape.m_radius = 1.0f;
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &circleShape;
            fixtureDef.isSensor = true;
            fixtureDef.density = 5.0f;
            pns->body->CreateFixture(&fixtureDef);
        }

    });

    ecs.system<GraphNode, PhysicalNodeSim>("PhysicsUpdateNode").kind(flecs::OnUpdate).iter([](flecs::iter& it, GraphNode* gn, PhysicalNodeSim* pns) {
        for (int i : it)
        {
            b2Vec2 pos = pns[i].body->GetPosition();
            // TODO: Map from physics world to screen space...
            gn[i].x = pos.x*PIXELS_PER_METER;
            gn[i].y = pos.y*PIXELS_PER_METER;
        }
    });

    ecs.system<Player, JoyDiff>("MovePlayer").interval(8.0f).term_at(2).src(playerController).instanced().each([](Player& player, JoyDiff& joyDiff) {
        b2Vec2 movementForce = {joyDiff.x * player.accelerationRate, joyDiff.y * player.accelerationRate};
        b2Vec2 movementMin = movementForce;
        movementMin.Normalize();
        movementMin *= player.minForce;
        if (movementMin.LengthSquared() > movementForce.LengthSquared())
        {
            movementForce = movementMin;
        }
        b2Vec2 movementCap = movementForce;
        movementCap.Normalize();
        movementCap *= player.maxForce;
        if (movementForce.LengthSquared() > movementCap.LengthSquared())
        {
            movementForce = movementCap;
        }
        // TODO: Friction, movespeed,

        if (movementForce.IsValid() && movementForce.LengthSquared() > 0.0f)
        {
            player.body->ApplyForceToCenter(movementForce, true);
        }
        b2Vec2 joy = {joyDiff.x, joyDiff.y};
        b2Vec2 velocity = player.body->GetLinearVelocity();
        b2Vec2 maxVelocity = velocity;
        maxVelocity.Normalize();
        maxVelocity *= player.minMoveSpeed;// + std::sqrt(player.maxMoveSpeed * joy.Length());
        if (velocity.LengthSquared() > maxVelocity.LengthSquared())
        {
            player.body->SetLinearVelocity(maxVelocity);
        }
        if (joy.LengthSquared() < 0.1f)
        {
        }
        b2Vec2 deccelerateVelocity = velocity;
        deccelerateVelocity.Normalize();
        deccelerateVelocity *= velocity.Length() - player.deccelerationRate;
        if (velocity.Length() < player.deccelerationRate)
        {
            deccelerateVelocity.x = 0;
        }
        if (velocity.Length() < player.deccelerationRate)
        {
            deccelerateVelocity.y = 0;
        }
        player.body->SetLinearVelocity(deccelerateVelocity);
    });

    ecs.observer<GraphNode, NodeRenderState, Snapped, World, Player>("NodeSnapCreateJoint").term_at(4).src(e).term_at(5).src(player).instanced().event(flecs::OnAdd).iter([](flecs::iter& it, GraphNode* gn, NodeRenderState* render, Snapped* snap, World* world, Player* player)
    {
        for (auto i : it)
        {
            __android_log_print(ANDROID_LOG_DEBUG, "GraphNavInst", "Snap Created");
            // TODO: Trigger Kotlin event here for UI display
            if (it.entity(i).has<GraphEvent>())
            {
                auto ge = it.entity(i).get<GraphEvent>();
                __android_log_print(ANDROID_LOG_DEBUG, "GraphNavInst", "Snap has GraphEvent %s with snap %i", ge->name, ge->snapEvent);
                if (ge->snapEvent == (int)SnapEventType::SNAP)
                {
                    if (strcmp(ge->name, "open_personal_data_menu") == 0)
                    {
                        __android_log_print(ANDROID_LOG_DEBUG, "GraphNavInst", "Open menu");
                        UIState* state = it.world().lookup("ui_controller").get_mut<UIState>();
                        state->menuVisible = true;
                        state->mapVisible = false;
                        it.world().lookup("player_controller").get_mut<CameraControl>()->mode = CameraControlMode::FIXED;
                        enable_nodes_w_category("ui");
                        disable_nodes_w_category("palace");
                    }
                    else if (strcmp(ge->name, "open_intention") == 0)
                    {

                    }
                    else if (strcmp(ge->name, "open_spacetime") == 0)
                    {
                        ecs.lookup("ui_controller").get_mut<UIState>()->mapVisible = true;
                    }
                    else if (strcmp(ge->name, "open_capital") == 0)
                    {
                        ecs.lookup("ui_controller").add<TriggerPlaidIntegration>();
                    }
                }
            }

            render[i].radius *= 3.0;

            b2BodyDef bodyDef;
            bodyDef.type = b2_staticBody;
            bodyDef.position = {gn[i].x/PIXELS_PER_METER, gn[i].y/PIXELS_PER_METER};
            snap[i].body = world->physics->CreateBody(&bodyDef);

            b2DistanceJointDef springJoint;
            springJoint.collideConnected = false;
            springJoint.bodyA = player->body;
            springJoint.bodyB = snap[i].body;
            springJoint.localAnchorA.Set(0.0f, 0.0f);
            springJoint.localAnchorB.Set(0.0f, 0.0f);
            b2Vec2 pos1 = player->body->GetWorldPoint(springJoint.localAnchorA);
            b2Vec2 pos2 = snap[i].body->GetWorldPoint(springJoint.localAnchorB);
            b2Vec2 d = pos2 - pos1;
//            springJoint.length = d.Length();
            springJoint.length = 0.0f;
//        b2LinearStiffness(springJoint.stiffness, springJoint.damping, frequencyHz, dampingRatio, player.body, snap.body);
            springJoint.stiffness = 0.0001f;
            springJoint.damping = 0.1f;
            snap[i].spring = (b2DistanceJoint*)world->physics->CreateJoint(&springJoint);
        }
    });

    ecs.observer<NodeRenderState, Snapped, World>("NodeSnapRemoveJoint").term_at(3).src(e).instanced().event(flecs::OnRemove).iter([](flecs::iter& it, NodeRenderState* render, Snapped* snap, World* world)
    {
        for (auto i : it)
        {
            render[i].radius *= 1.0/3.0;
            if (it.entity(i).has<GraphEvent>())
            {
                auto ge = it.entity(i).get<GraphEvent>();
                if (ge->snapEvent == (int)SnapEventType::UNSNAP && strcmp(ge->name, "close_personal_data_menu") == 0)
                {
                    UIState* state = it.world().lookup("ui_controller").get_mut<UIState>();
                    state->menuVisible = false;
                    it.world().lookup("player_controller").get_mut<CameraControl>()->mode = CameraControlMode::FOLLOW_PLAYER;

                    disable_nodes_w_category("ui");
                    enable_nodes_w_category("palace");
                }
            }
            __android_log_print(ANDROID_LOG_DEBUG, "GraphNav", "Snap removed");
            world->physics->DestroyJoint(snap[i].spring);
            world->physics->DestroyBody(snap[i].body);
        }
    });

    ecs.system<Snapped, GraphNode, Player, JoyDiff>("NodeSnapUpdateStiffness").term_at(3).src(player).term_at(4).src(playerController).kind(flecs::OnUpdate).instanced().iter([](flecs::iter& it, Snapped* snap, GraphNode* gn, Player* player, JoyDiff* joy)
    {
        b2Vec2 graphWorldPos = {gn->x/PIXELS_PER_METER, gn->y/PIXELS_PER_METER};
        b2Vec2 playerPos = player->body->GetPosition();
        b2Vec2 diff = playerPos - graphWorldPos;
//        snap->spring->SetStiffness(0.0006f); //  + 0.0001f *diff.Length())
        if (joy->x*joy->x + joy->y*joy->y == 0.0f) // TODO: pass active data, because diff might be 0 and player is just moving press :/
        {
            snap->spring->SetStiffness(0.0020f);
        } else
        {
            snap->spring->SetStiffness(0.0006f);
        }
    });

    ecs.system<GraphInterpolator>("GraphInterpolateProgress").kind(flecs::OnUpdate).iter([](flecs::iter& it, GraphInterpolator* gi)
    {
        for (auto i : it)
        {
            gi[i].p = std::min(1.0f, gi[i].p + gi[i].rate * it.delta_time());
        }
    });

    auto gi = GraphInterpolator(0.0f, 0.01f);
    e.set<GraphInterpolator>(gi);

    struct NodeSnapCompare
    {
        GraphNode* gn;
        NodeSnapDestination* nsd;
    };
    struct NodeSnapTest
    {
        b2Vec2 playerPos;
        std::vector<NodeSnapCompare> nodes;
        std::unordered_map<GraphNode, flecs::entity, GraphNodeHasher> treeMap;
        std::unordered_map<GraphNode, NodeSnapDestination*, GraphNodeHasher> nsdMap;
        Player* player;
    };
    void* temp = new NodeSnapTest();

    ecs.system<GraphNode, NodeSnapDestination, Snapped, Player>("GraphSnapExecute")
//            .interval(1000.0)
            .term_at(3).oper(flecs::Optional)
            .term_at(4).src(player)
            .ctx(temp)
            .instanced()
            .run([](flecs::iter_t *it) {
                auto nst = (NodeSnapTest*) it->ctx;
                nst->nodes.clear();
                nst->treeMap.clear();
                while (ecs_iter_next(it)) {
                    it->callback(it);
                }
                // TODO: Sort by difference of distance with NodeSnapDestination radius
                std::sort(nst->nodes.begin(), nst->nodes.end(), [&nst](const NodeSnapCompare& g_a, const NodeSnapCompare& g_b)
                {
                    float distSquaredA = std::pow(g_a.gn->x - nst->playerPos.x, 2) + std::pow(g_a.gn->y - nst->playerPos.y, 2);
                    float distSquaredB = std::pow(g_b.gn->x - nst->playerPos.x, 2) + std::pow(g_b.gn->y - nst->playerPos.y, 2);
                    return distSquaredA - std::pow(g_a.nsd->radius, 2)*g_a.nsd->enabled < distSquaredB - std::pow(g_a.nsd->radius, 2)*g_b.nsd->enabled;
                });

                Player* player = nst->player;
                GraphNode* snapToNode = nst->nodes[0].gn;
                auto snapDestination = nst->treeMap[*(nst->nodes[0].gn)];
                // TODO: If there are multiple valid snap destinations :(
                float distanceToSnap = std::sqrt(std::pow(snapToNode->x - nst->playerPos.x, 2) + std::pow(snapToNode->y - nst->playerPos.y, 2));

                if (ecs.is_valid(snapDestination) && ecs.is_alive(snapDestination))
                {
                    if (distanceToSnap < nst->nsdMap[*snapToNode]->radius)
                    {
                        if (!snapDestination.has<Snapped>()) {
                            if (ecs.is_valid(player->lastSnap)) {
                                player->lastSnap.remove<Snapped>();
                            }
                            snapDestination.add<Snapped>();
                        }
                    } else if(ecs.is_valid(player->lastSnap) && ecs.is_alive(player->lastSnap))
                    {
                        const GraphNode* currentSnap = player->lastSnap.get<GraphNode>();
                        float distanceToCurrentSnap = std::sqrt(std::pow(currentSnap->x - nst->playerPos.x, 2) + std::pow(currentSnap->y - nst->playerPos.y, 2));
                        if (distanceToCurrentSnap > nst->nsdMap[*currentSnap]->radius)
                        {
                            player->lastSnap.remove<Snapped>();
                            player->lastSnap = flecs::entity::null();
                        }
                    }
                }
                player->lastSnap = snapDestination;
            })
            .iter([](flecs::iter& it, GraphNode* gn, NodeSnapDestination* nsd, Snapped* snapped, Player* player) {
                auto nst = (NodeSnapTest*) it.ctx();
                b2Vec2 playerPos = player->body->GetPosition();
                playerPos *= PIXELS_PER_METER;
                nst->playerPos = playerPos;
                nst->player = player;
                for (auto i : it)
                {
                    if (nsd[i].enabled)
                    {
                        nst->nodes.push_back({&gn[i], &nsd[i]});
                        nst->nsdMap[GraphNode(gn[i].x, gn[i].y)] = &nsd[i];
                        nst->treeMap[GraphNode(gn[i].x, gn[i].y)] = it.entity(i);
                    }
                }
//                __android_log_print(ANDROID_LOG_DEBUG, "GraphNavX", "Matched %i nodes", nst->nodes.size());

            });

//    ecs.system<GraphNode, NodeSnapDestination, Snapped, Player>("GraphSnapExecute")
//            .interval(1000.0)
//            .term_at(3).oper(flecs::Optional)
//            .term_at(4).src(player)
//            .ctx(temp)
//            .instanced()
//            .run([](flecs::iter_t *it) {
//                while (ecs_iter_next(it)) {
//                    it->callback(it);
//                }
//            })
//            .iter([](flecs::iter& it, GraphNode* gn, NodeSnapDestination* nsd, Snapped* snapped, Player* player) {
//                b2Vec2 playerPos = player->body->GetPosition();
//                playerPos *= PIXELS_PER_METER;
//                std::vector<GraphNode*> nodes;
//                for (auto i : it)
//                {
//                    nodes.push_back(&gn[i]);
//                }
//                std::sort(nodes.begin(), nodes.end(), [&playerPos](const GraphNode* g_a, const GraphNode* g_b)
//                {
//                    float distSquaredA = std::pow(g_a->x - playerPos.x, 2) + std::pow(g_a->y - playerPos.y, 2);
//                    float distSquaredB = std::pow(g_b->x - playerPos.x, 2) + std::pow(g_b->y - playerPos.y, 2);
//                    return distSquaredA < distSquaredB;
//                });
//                std::unordered_map<GraphNode, flecs::entity, GraphNodeHasher> treeMap;
//                for (auto i : it)
//                {
//                    treeMap[GraphNode(gn[i].x, gn[i].y)] = it.entity(i);
//                }
//
////                __android_log_print(ANDROID_LOG_DEBUG, "GraphNav", "Tick");
////                PointCloud<float> cloud;
////                using my_kd_tree_t = nanoflann::KDTreeSingleIndexDynamicAdaptor<
////                nanoflann::L2_Simple_Adaptor<float, PointCloud<float>>,
////                PointCloud<float>, 3 /* dim */
////                                   >;
////                my_kd_tree_t index(3 /*dim*/, cloud, {10 /* max leaf */});
////                b2Vec2 playerPos = player->body->GetPosition();
//                int matchedNodesCount = 0;
//                for (auto i : it)
//                {
//                    matchedNodesCount++;
//                }
//                __android_log_print(ANDROID_LOG_DEBUG, "GraphNavX", "Matched %i nodes", matchedNodesCount);
////                cloud.pts.resize(matchedNodesCount);
////                std::unordered_map<GraphNode, flecs::entity, GraphNodeHasher> treeMap;
////                for (auto i : it)
////                {
////                    treeMap[GraphNode(gn[i].x, gn[i].y)] = it.entity(i);
////                    cloud.pts[i].x = gn[i].x;
////                    cloud.pts[i].y = gn[i].y;
////                    cloud.pts[i].z = 0.0f;
////                }
////                size_t chunk_size = 100;
////                for (size_t i = 0; i < matchedNodesCount; i = i + chunk_size)
////                {
////                    size_t end = std::min<size_t>(i + chunk_size, matchedNodesCount - 1);
////                    // Inserts all points from [i, end]
////                    index.addPoints(i, end);
////                }
////                const float query_pt[3] = {playerPos.x*PIXELS_PER_METER, playerPos.y*PIXELS_PER_METER, 0.0f};
////                const size_t                   num_results = 1;
////                size_t                         ret_index;
////                float                          out_dist_sqr;
////                nanoflann::KNNResultSet<float> resultSet(num_results);
////                resultSet.init(&ret_index, &out_dist_sqr);
////                index.findNeighbors(resultSet, query_pt);
////                GraphNode found = GraphNode(cloud.pts[ret_index].x, cloud.pts[ret_index].y);
////                size_t nodeIndex = treeMap[found];
////                auto snapDestination = treeMap[found];
////                __android_log_print(ANDROID_LOG_DEBUG, "GraphNav", "Snap destination is %i", snapDestination.id());
////                __android_log_print(ANDROID_LOG_DEBUG, "GraphNav", "Result: (%f, %f)",  found.x, found.y);
//                auto snapDestination = treeMap[*nodes[0]];
//                if (it.world().is_valid(snapDestination) && it.world().is_alive(snapDestination))
//                {
//                    __android_log_print(ANDROID_LOG_DEBUG, "GraphNav", "Snap destination is valid");
//                    if (!snapDestination.has<Snapped>()) {
//                        if (it.world().is_valid(player->lastSnap)) {
//                            __android_log_print(ANDROID_LOG_DEBUG, "GraphNav",
//                                                "Play last snap is valid");
//                            player->lastSnap.remove<Snapped>();
//                        }
//                        __android_log_print(ANDROID_LOG_DEBUG, "GraphNav", "Add Snapped component");
//                        snapDestination.add<Snapped>();
//                    }
//                } else
//                {
//                    __android_log_print(ANDROID_LOG_DEBUG, "GraphNav", "SnapDestination not found");
//                }
//                player->lastSnap = snapDestination;
//                // TODO: Create a spring joint at this location: tag GraphNode entity
//                // When new location is found (or distance > NodeSnapDestination[i].radius), then delete the spring joint and create a new one
//            });

    // TODO: GraphInterpolation should no longer be global
    // Places it could go: back node of the menu -> Just lookup and add it
    // distinct entity -> Create and destroy
    // actually make it global but reference a specific graph being interpolated (will there ever need to be multiple graphs interpolating simultaneously?)
//    ecs.system<PhysicalNodeSim, GraphInterpolator>("GraphTest")
//            .term_at(2).src(e)
//            .term<CreateSpringEdge>(flecs::Wildcard)
//            .term(Equivariant, flecs::Wildcard)
//            .instanced()
//            .kind(flecs::OnUpdate)
//            .iter([](flecs::iter& it, PhysicalNodeSim* pns, GraphInterpolator* gi) {
//                return 0;
//        for (auto i : it)
//        {
//            flecs::entity self = it.entity(i);
//            // Find the distance of the spring joint on this node
//            if (pns[i].body)
//            {
//                b2JointEdge* jointEdge = pns[i].body->GetJointList();
//                for (int j = 0; j < i; j++) // TODO: Store in ctx...
//                {
//                    jointEdge = jointEdge->next;
//                }
//                auto distJoint = (b2DistanceJoint*)(jointEdge->joint);
//                flecs::entity target = it.id(4).second();
//                auto otherEdge = it.field<CreateSpringEdge>(3)[i].edgeChild.get<Edge>();
//                __android_log_print(ANDROID_LOG_DEBUG, "GraphDistUpdate", "Other edge is %i", it.field<CreateSpringEdge>(3)[i].edgeChild.id());
//                float targetDistance = otherEdge->Magnitude()/PIXELS_PER_METER;
//                float oldLength = distJoint->GetLength();
//                float newLength = linearInterpolation(pns[i].start_spring_dist, targetDistance, gi->p);
//                if (i == 0)
//                {
//                    __android_log_print(ANDROID_LOG_DEBUG, "GraphDistUpdate", "Old length %f -> New length %f", oldLength, newLength);
//                }
//                distJoint->SetLength(newLength);
//            }
//        }
//    });

    ecs.system<World>("ProgressWorld").interval(8.0f).iter([](flecs::iter& it, World* world) {
        for (int i : it)
        {
            world[i].physics->Step(8.0f, 8, 4); // TODO: Constant...
            for (b2Joint* joint : world[i].jointRemoveQueue)
            {
                world[i].physics->DestroyJoint(joint);
            }
            for (b2Body* body : world[i].bodyRemoveQueue)
            {
                world[i].physics->DestroyBody(body);
            }
        }
    });

    ecs.system<Edge, PhysicalNodeSim>("PhysicsUpdateChildEdge").term_at(2).parent().instanced().kind(flecs::OnUpdate).iter([](flecs::iter& it, Edge* edge, PhysicalNodeSim* pns)
    {
        __android_log_print(ANDROID_LOG_DEBUG, "GraphEqDebug", "PhysicsUpdateChildEdge it count is %i", it.count());
        b2JointEdge* jointEdge = pns->body->GetJointList();
        // TODO: THIS NEEDS TO BE REFACTORED TO SUPPORT N JOINTS WHERE ORDER INSERTION AWARENESS IS UNKNOWN AT RUNTIME
        for (auto i : it)
        {
//            for (;jointEdge && jointEdge->joint;jointEdge = jointEdge->next)
//            {
            auto distJoint = (b2DistanceJoint*)(jointEdge->joint);
            b2Vec2 head_pos = distJoint->GetBodyA()->GetPosition(); // ERROR: Consider whether joint bodies (for duplicate CreateSpringEdge, spring_personal_node_parent are correct
            b2Vec2 tail_pos = distJoint->GetBodyB()->GetPosition();
            edge[i].tail_x = tail_pos.x*PIXELS_PER_METER;
            edge[i].tail_y = tail_pos.y*PIXELS_PER_METER;
            edge[i].head_x = head_pos.x*PIXELS_PER_METER;
            edge[i].head_y = head_pos.y*PIXELS_PER_METER;
            jointEdge = jointEdge->next;
//                __android_log_print(ANDROID_LOG_DEBUG, "GraphJoints", "PARENT PNS: Head pos is (%f, %f)", edge[i].head_x, edge[i].head_y);
//            }
        }
    });

//    ecs.system<PhysicalNodeSim, Edge>("PhysicsUpdateEdge").term(CreateSpringEdge, flecs::Wildcard).kind(flecs::OnUpdate).iter([](flecs::iter& it, PhysicalNodeSim* pns, Edge* edge) {
//        for (int i : it)
//        {
//            b2JointEdge* jointEdge = pns[i].body->GetJointList();
//            for (;jointEdge && jointEdge->joint;jointEdge = jointEdge->next)
//            {
//                auto distJoint = (b2DistanceJoint*)(jointEdge->joint);
//                b2Vec2 head_pos = distJoint->GetBodyA()->GetPosition();
//                b2Vec2 tail_pos = distJoint->GetBodyB()->GetPosition();
//                edge[i].tail_x = tail_pos.x*PIXELS_PER_METER;
//                edge[i].tail_y = tail_pos.y*PIXELS_PER_METER;
//                edge[i].head_x = head_pos.x*PIXELS_PER_METER;
//                edge[i].head_y = head_pos.y*PIXELS_PER_METER;
////                __android_log_print(ANDROID_LOG_DEBUG, "GraphJoints", "Head pos is (%f, %f)", edge[i].head_x, edge[i].head_y);
//            }
//        }
//    });

 // Wind drifitng/random disturbances in graph
//    ecs.system<PhysicalNodeSim>("WindDrift").interval(10
//    ).iter([](flecs::iter& it, PhysicalNodeSim* pns) {
//        for (auto i : it)
//        {
//            int forceTicket = rand() % 1000;
//            if (forceTicket > 900)
//            {
//                b2Vec2 randomForce(((float)(rand()%100)-50.0f) * 0.00049f, ((float)(rand()%100)-50.0f) * 0.00049f);
//                pns[i].body->ApplyForceToCenter(randomForce, true);
//            }
//        }
//    });

//    ecs.observer<PhysicalNodeSim, NoiseSource>("WanderZone").term_at(2).src(uiController).instanced().event(flecs::OnAdd).iter([](flecs::iter& it, PhysicalNodeSim* pns, NoiseSource* source) {
//        ecs.defer_begin();
//        for (auto i : it)
//        {
//            it.entity(i).set<Wanderer>({b2Vec2(source->wanderers * 0.05, source->wanderers * 0.05)});
//            source->wanderers++;
//        }
//        ecs.defer_end();
//    });

    ecs.system<PhysicalNodeSim, Wanderer, NoiseSource>().term_at(3).src(uiController).instanced().interval(8.0f).iter([](flecs::iter& it, PhysicalNodeSim* pns, Wanderer* wander, NoiseSource* source) {
        for (auto i : it)
        {
            float noiseX = source->noise->GetNoise(wander[i].noisePos.x, 0.0f);
            float noiseY = source->noise->GetNoise(0.0f, wander[i].noisePos.y);
            b2Vec2 force = {noiseX, noiseY};
            force.Normalize();
            __android_log_print(ANDROID_LOG_DEBUG, "GraphAesthetics", "Wander (%f, %f)", noiseX, noiseY);
            force *= 0.02f;
            if (force.IsValid() && force.LengthSquared() > 0)
            {
                pns[i].body->ApplyForceToCenter(force, true);
            }
            wander[i].noisePos += b2Vec2(0.4f, 0.4f);
        }
    });

//
//    // TODO: Create equivalent random network...
//    // Let's use Python to generate plecs...
//
    AAssetManager* am = AAssetManager_fromJava(env, assetManager);
    AAsset* asset = AAssetManager_open(am, "text_graph.flecs", AASSET_MODE_BUFFER);
//    if (asset != nullptr) {
//        // Read asset data
//        const void* data = AAsset_getBuffer(asset);
//        off_t length = AAsset_getLength(asset);
//
//        char* cdata = (char*) data;
//        ecs.plecs_from_str("graph_text", cdata);
//
////        __android_log_print(ANDROID_LOG_DEBUG, "YourTag", "AAsset Data: %.*s", static_cast<int>(length), cdata);
//
//        AAsset_close(asset); // Close the asset when done
//    }

//    asset = AAssetManager_open(am, "test.flecs", AASSET_MODE_BUFFER);
//    if (asset != nullptr) {
//        const void* data = AAsset_getBuffer(asset);
//        off_t length = AAsset_getLength(asset);
//
//        char* cdata = (char*) data;
//        ecs.plecs_from_str("test", cdata);
//        AAsset_close(asset);
//    }
//    auto menuTest = ecs.lookup("graph::compose");
//    auto sec = menuTest.target(Equivariant);
//    __android_log_print(ANDROID_LOG_DEBUG, "GraphComposition", "Hierarchy %s", sec.name().c_str());


    asset = AAssetManager_open(am, "menu.flecs", AASSET_MODE_BUFFER);
    if (asset != nullptr) {
        const void* data = AAsset_getBuffer(asset);
        off_t length = AAsset_getLength(asset);

        char* cdata = (char*) data;
        ecs.plecs_from_str("menu", cdata);
        AAsset_close(asset);
    }

    asset = AAssetManager_open(am, "palace.flecs", AASSET_MODE_BUFFER);
    if (asset != nullptr) {
        const void* data = AAsset_getBuffer(asset);
        off_t length = AAsset_getLength(asset);

        char* cdata = (char*) data;
        ecs.plecs_from_str("palace", cdata);
        AAsset_close(asset);
    }

    ecs.lookup("spring_personal_node_parent").set<Wanderer>({b2Vec2_zero});

//    auto palace = ecs.lookup("palace");
//    __android_log_print(ANDROID_LOG_DEBUG, "GraphComposition", "Palace lookup is %s", palace.name().c_str());
    // TODO:
    // Use a 1 with a distinct radius (ie we need to sort node snaps based on distance AND range)

//    asset = AAssetManager_open(am, "spring_graph.flecs", AASSET_MODE_BUFFER);
//    if (asset != nullptr) {
//        // Read asset data
//        const void* data = AAsset_getBuffer(asset);
//        off_t length = AAsset_getLength(asset);
//        char* cdata = (char*) data;
//        ecs.plecs_from_str("spring_graph", cdata);
//        AAsset_close(asset); // Close the asset when done
//    }

    auto csq = ecs.query_builder<PhysicalNodeSim>().term(CreateSpringEdgeFromConfig, flecs::Wildcard).term(Equivariant, flecs::Wildcard).optional().build();
    auto edgesToAdd = std::unordered_map<const char*, std::vector<CreateEdgeData>>();
    auto jointDefs = std::vector<b2DistanceJointDef>();
    csq.iter([&jointDefs, &edgesToAdd](flecs::iter& it, PhysicalNodeSim* pns) {
        for (auto i : it)
        {
//        CreateSpringEdgeData* springCreatorData = &it.field<CreateSpringEdgeData>(4)[i];
        flecs::entity config = it.id(2).second();
//        auto nodeTargetQuery = ecs.query_builder<>().term<CreateSpringEdge>(flecs::Wildcard).inout().build();
//        CreateSpringEdge* cse;
        flecs::entity directedEdgeTarget = config.target<CreateSpringEdge>();
        //        nodeTargetQuery.iter([&cse, &directedEdgeTarget](flecs::iter& it)
//        {
//            cse = &it.field<CreateSpringEdge>(1)[0];
//            directedEdgeTarget = it.id(1).second();
//        });
        //flecs::entity directedEdgeTarget = config.get<CreateSpringEdge>(flecs::Wildcard); // spring edge entity...
//            __android_log_print(ANDROID_LOG_DEBUG, "GraphEdgeDebug", "Directed edge target of %s is %s", it.entity(i).name().c_str(), directedEdgeTarget.name().c_str());
        b2DistanceJointDef springJoint;
        b2Body* otherBody = directedEdgeTarget.get_mut<PhysicalNodeSim>()->body;
        springJoint.bodyA = pns[i].body;
        springJoint.bodyB = otherBody;
        float frequencyHz = 0.1f;
        float dampingRatio = 0.1f;
        springJoint.localAnchorA.Set(0.0f, 0.0f);
        springJoint.localAnchorB.Set(0.0f, 0.0f);
        b2Vec2 pos1 = pns[i].body->GetWorldPoint(springJoint.localAnchorA);
        b2Vec2 pos2 = otherBody->GetWorldPoint(springJoint.localAnchorB);
        b2Vec2 d = pos2 - pos1;
//        springJoint.minLength = d.Length();
        springJoint.length = d.Length();
        b2LinearStiffness(springJoint.stiffness, springJoint.damping, frequencyHz, dampingRatio, pns[i].body, otherBody);

        springJoint.stiffness = 0.01f;
        if (config.has<CreateSpringEdgeData>())
        {
            auto springData = config.get<CreateSpringEdgeData>();
            springJoint.stiffness = springData->strength;
        }
        springJoint.damping = 0.1f;
        if (pns[i].body != otherBody)
        {
            jointDefs.push_back(springJoint);
        }
        flecs::entity node = it.entity(i);
        Edge dir = {pos1.x, pos1.y, pos2.x, pos2.y};
        pns[i].start_spring_dist = dir.Magnitude();

//        if (it.id(2).first().is_a<CreateSpringEdge>())
//        {
//            __android_log_print(ANDROID_LOG_DEBUG, "GraphEdgeDebug", "it.id(2).first() is a CreateSpringEdge");
//        } else
//        {
//            __android_log_print(ANDROID_LOG_DEBUG, "GraphEdgeDebug", "it.id(2).first() is NOT a CreateSpringEdge");
//        }
//        CreateSpringEdge* cse = it.id(2).range().get<CreateSpringEdge>();
        if (edgesToAdd.count(node.name().c_str()))
        {
//            CreateSpringEdge* cse = it.field<CreateSpringEdge>(2);
            // it.id(2).first() is an entity of type CreateSpringComponent
            edgesToAdd[node.name().c_str()].push_back({dir, config});
        } else
        {
            edgesToAdd[node.name().c_str()] = {{dir, config}};
        }
        }
    });

    for (auto it : edgesToAdd)
    {
        for (auto edge : it.second)
        {
            // TODO: Make Edge system from PhysicalNodeSim parent...

            flecs::entity edgeChild = ecs.entity();
            edgeChild.child_of(ecs.lookup(it.first));
//            edge.config.set<SpringEdge>({edgeChild})
//            edge.config.get_mut<SpringEdge>()->entity = edgeChild;
            edgeChild.set<Edge>(edge.data);
        }
    }

    World* world = e.get_mut<World>();
    for (b2DistanceJointDef& jd : jointDefs)
    {
        b2Joint* joint = world->physics->CreateJoint(&jd);
    }

    return 0.0f;



    asset = AAssetManager_open(am, "keyboard_graph.flecs", AASSET_MODE_BUFFER);
    if (asset != nullptr) {
        const void* data = AAsset_getBuffer(asset);
        off_t length = AAsset_getLength(asset);
        char* cdata = (char*) data;
        ecs.plecs_from_str("keyboard_graph", cdata);
        AAsset_close(asset); // Close the asset when done
    }

    return 0;
}

extern "C" JNIEXPORT jobject JNICALL
Java_com_example_graphlife_MainActivity_createArrayOfNodes(JNIEnv* env, jobject /* this */) {
    // Get the Java ArrayList class
    jclass arrayListClass = env->FindClass("java/util/ArrayList");

    // Create an instance of ArrayList
    jmethodID arrayListConstructor = env->GetMethodID(arrayListClass, "<init>", "()V");
    jobject javaArrayList = env->NewObject(arrayListClass, arrayListConstructor);

    // Get the ArrayList add method
    jmethodID arrayListAddMethod = env->GetMethodID(arrayListClass, "add", "(Ljava/lang/Object;)Z");

    // Create GraphNode instances and add them to the ArrayList
    std::vector<GraphNode> nodes;
    std::vector<NodeRenderState> renderStates;

    // Populate the vector with GraphNode instances (e.g., one node with coordinates (100.0f, 100.0f))
    // flecs::iter& it, GraphNodeCategory
    auto uiController = ecs.lookup("ui_controller");
    auto q = ecs.query_builder<GraphNode, GraphNodeCategory, NodeRenderState, GraphViewState>().term_at(4).src(uiController).instanced().build(); // TODO: Make this work when a GraphNode has multiple node categories with bitmask
//    auto q = ecs.query_builder<GraphNode, GraphNodeCategory>().build();
    q.iter([&nodes, &renderStates](flecs::iter& it, GraphNode* gn, GraphNodeCategory* cat, NodeRenderState* render, GraphViewState* gvs) { // GraphViewState* gvs
        for (auto i : it)
        {
            if (gvs->hiddenCategories->count(std::string(cat[i].name)) == 0)
            {
                nodes.push_back({gn[i].x, gn[i].y});
                renderStates.push_back(render[i]);
            }
        }
    });
//    nodes.push_back({(float)pixelWidth/2, (float)pixelHeight/2-200});

    // Get the Java class for GraphNode (replace with your actual package and class name)
    jclass nodeClass = env->FindClass("com/example/graphlife/GraphNode");
    jmethodID constructor = env->GetMethodID(nodeClass, "<init>", "(FFF)V");

    for (size_t i = 0; i < nodes.size(); i++) {
        jobject javaNode = env->NewObject(nodeClass, constructor, nodes[i].x, nodes[i].y, renderStates[i].radius);
        env->CallBooleanMethod(javaArrayList, arrayListAddMethod, javaNode);
    }

    return javaArrayList;
}

extern "C" JNIEXPORT jobject JNICALL
Java_com_example_graphlife_MainActivity_createArrayOfEdges(JNIEnv* env, jobject /* this */) {
    ECS_TAG(ecs, DiEdge);
    // Get the Java ArrayList class
    jclass arrayListClass = env->FindClass("java/util/ArrayList");

    // Create an instance of ArrayList
    jmethodID arrayListConstructor = env->GetMethodID(arrayListClass, "<init>", "()V");
    jobject javaArrayList = env->NewObject(arrayListClass, arrayListConstructor);

    // Get the ArrayList add method
    jmethodID arrayListAddMethod = env->GetMethodID(arrayListClass, "add", "(Ljava/lang/Object;)Z");

    std::vector<EdgePath> edges;

    auto sq = ecs.query_builder<Snapped>().build();
    sq.iter([&edges](flecs::iter& it, Snapped* snap) {
        if (snap->spring)
        {
            for (auto i : it)
            {
                b2Vec2 pos_a = snap[i].spring->GetAnchorA();
                b2Vec2 pos_b = snap[i].spring->GetAnchorB();
                edges.push_back({pos_a.x*PIXELS_PER_METER, pos_a.y*PIXELS_PER_METER, pos_b.x*PIXELS_PER_METER, pos_b.y*PIXELS_PER_METER, 0});
            }
        }
    });

    auto uiController = ecs.lookup("ui_controller");
    auto q = ecs.query_builder<Edge, ConicEdgeMod, CubicEdgeMod, GraphViewState>().term_at(2).optional().term_at(3).optional().term_at(4).src(uiController).instanced().build();
//    auto q = world.query_builder<Edge, ConicEdgeMod, CubicEdgeMod>().build();
//    q.each([&edges](Edge& edge, ConicEdgeMod& conicEdgeMod, CubicEdgeMod& cubicEdgeMod) {
//        edges.push_back(edge);
//    });
    q.iter([&edges](flecs::iter& it,
            Edge* edge, ConicEdgeMod* conicEdgeMod, CubicEdgeMod* cubicEdgeMod, GraphViewState* gvs) {
        for (auto i : it)
        {
            auto parent = it.entity(i).parent();
            if (parent.is_valid() && parent.has<GraphNodeCategory>())
            {
                __android_log_print(ANDROID_LOG_DEBUG, "GraphComposition", "parent of edge has GraphNodeCategory");
                const GraphNodeCategory* cat = parent.get<GraphNodeCategory>();
                if (gvs->hiddenCategories->count(std::string(cat->name)))
                {
                    continue;
                }
            }
            if (conicEdgeMod)
            {
                edges.push_back({edge[i].head_x, edge[i].head_y, edge[i].tail_x, edge[i].tail_y, 1, conicEdgeMod[i].mid_x, conicEdgeMod[i].mid_y, conicEdgeMod[i].weight});
            } else if (cubicEdgeMod)
            {
                edges.push_back({edge[i].head_x, edge[i].head_y, edge[i].tail_x, edge[i].tail_y, 2, cubicEdgeMod[i].mid_a_x, cubicEdgeMod[i].mid_a_y, cubicEdgeMod[i].mid_b_x, cubicEdgeMod[i].mid_b_y});
            } else
            {
                edges.push_back({edge[i].head_x, edge[i].head_y, edge[i].tail_x, edge[i].tail_y, 0});
            }
        }
    });

    auto fiq = ecs.query_builder<GraphNode>().term(DiEdge, flecs::Wildcard).build();
    fiq.iter([&edges](flecs::iter& it, GraphNode* gn)
    {
        for (auto i : it)
        {
            flecs::entity connected = it.pair(2).second();
            const GraphNode* cn = connected.get<GraphNode>();
            edges.push_back({gn[i].x, gn[i].y, cn->x, cn->y, 0});
        }
    });

    jclass edgeClass = env->FindClass("com/example/graphlife/Edge");
    jmethodID constructor = env->GetMethodID(edgeClass, "<init>", "(FFFFIFFFF)V");

    for (size_t i = 0; i < edges.size(); i++) {
        jobject javaNode = env->NewObject(edgeClass, constructor, edges[i].head_x, edges[i].head_y, edges[i].tail_x, edges[i].tail_y, edges[i].edge_type, edges[i].val_0, edges[i].val_1, edges[i].val_2, edges[i].val_3);
        env->CallBooleanMethod(javaArrayList, arrayListAddMethod, javaNode);
    }

    return javaArrayList;
}
