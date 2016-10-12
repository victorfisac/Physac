/**********************************************************************************************
*
*   Physac - 2D Physics library for videogames
*
*   // TODO: add description
* 
*   CONFIGURATION:
*   
*   #define PHYSAC_IMPLEMENTATION
*       Generates the implementation of the library into the included file.
*       If not defined, the library is in header only mode and can be included in other headers 
*       or source files without problems. But only ONE file should hold the implementation.
*
*   #define PHYSAC_STATIC (defined by default)
*       The generated implementation will stay private inside implementation file and all 
*       internal symbols and functions will only be visible inside that file.
*
*   #define PHYSAC_NO_THREADS
*       The generated implementation won't include pthread library and user must create a secondary thread to call PhysicsThread().
*       It is so important that the thread where PhysicsThread() is called must not have v-sync or any other CPU limitation.
*
*   #define PHYSAC_STANDALONE
*       Avoid raylib.h header inclusion in this file. Data types defined on raylib are defined
*       internally in the library and input management and drawing functions must be provided by
*       the user (check library implementation for further details).
*
*   #define PHYSAC_MALLOC()
*   #define PHYSAC_FREE()
*       You can define your own malloc/free implementation replacing stdlib.h malloc()/free() functions.
*       Otherwise it will include stdlib.h and use the C standard library malloc()/free() function.
*
*   VERY THANKS TO:
*       - Ramón Santamaria (@raysan5)
*
*   LICENSE: zlib/libpng
*
*   Copyright (c) 2016 Victor Fisac
*
*   This software is provided "as-is", without any express or implied warranty. In no event
*   will the authors be held liable for any damages arising from the use of this software.
*
*   Permission is granted to anyone to use this software for any purpose, including commercial
*   applications, and to alter it and redistribute it freely, subject to the following restrictions:
*
*     1. The origin of this software must not be misrepresented; you must not claim that you
*     wrote the original software. If you use this software in a product, an acknowledgment
*     in the product documentation would be appreciated but is not required.
*
*     2. Altered source versions must be plainly marked as such, and must not be misrepresented
*     as being the original software.
*
*     3. This notice may not be removed or altered from any source distribution.
*
**********************************************************************************************/

#ifndef PHYSAC_H
#define PHYSAC_H

#include "raylib.h"

#define PHYSAC_STATIC
#ifdef PHYSAC_STATIC
    #define PHYSACDEF static            // Functions just visible to module including this file
#else
    #ifdef __cplusplus
        #define PHYSACDEF extern "C"    // Functions visible from other files (no name mangling of functions in C++)
    #else
        #define PHYSACDEF extern        // Functions visible from other files
    #endif
#endif

//----------------------------------------------------------------------------------
// Defines and Macros
//----------------------------------------------------------------------------------
#define     COLLISION_INTERATIONS           10
#define     MAX_TIMESTEP                    0.02
#define     MAX_PHYSICS_BODIES              256
#define     MAX_PHYSICS_MANIFOLDS           256
#define     PHYSAC_MALLOC(size)             malloc(size)
#define     PHYSAC_FREE(ptr)                free(ptr)

//----------------------------------------------------------------------------------
// Types and Structures Definition
// NOTE: Below types are required for PHYSAC_STANDALONE usage
//----------------------------------------------------------------------------------
#if defined(PHYSAC_STANDALONE)
    typedef struct Vector2 {
        float x;
        float y;
    } Vector2;
#endif

typedef enum PhysicsShapeType { PHYSICS_CIRCLE, PHYSICS_POLYGON } PhysicsShapeType;

// Previously defined to be used in PhysicsShape struct as circular dependencies
typedef struct PhysicsBodyData *PhysicsBody;

typedef struct PhysicsShape {
    PhysicsShapeType type;
    PhysicsBody body;
    float radius;               // Used for circle shapes
    // TODO: add polygon information
} PhysicsShape;

typedef struct PhysicsBodyData {
    unsigned int id;
    unsigned int index;
    Vector2 position;
    Vector2 velocity;
    Vector2 force;
    
    float angularVelocity;
    float torque;
    float orient;               // Rotation in radians
    
    float inertia;              // Moment of inertia
    float inverseInertia;       // Inverse value of inertia
    float mass;                 // Physics body mass
    float inverseMass;          // Inverse value of mass
    
    PhysicsShape shape;         // Physics body shape information (type, radius, vertexs, normals)
} PhysicsBodyData;

//----------------------------------------------------------------------------------
// Module Functions Declaration
//----------------------------------------------------------------------------------
PHYSACDEF void InitPhysics(Vector2 gravity);                        // Initializes physics values, pointers and creates physics loop thread

PHYSACDEF PhysicsBody CreatePhysicsBody(Vector2 pos, float mass);   // Creates a new physics body with generic parameters
void DestroyPhysicsBody(PhysicsBody body);                          // Unitializes and destroy a physics body

PHYSACDEF void DrawPhysicsBodies(void);                             // Draw all created physics physics bodies shapes
PHYSACDEF void DrawPhysicsInfo(void);                               // Draw debug information about physics states and values

PHYSACDEF void ClosePhysics(void);                                  // Unitializes physics pointers and closes physics loop thread

#endif // PHYSAC_H

/***********************************************************************************
*
*   PHYSAC IMPLEMENTATION
*
************************************************************************************/

#if defined(PHYSAC_IMPLEMENTATION)

#ifndef PHYSAC_NO_THREADS
    #include <pthread.h>        // Required for: pthread_t, pthread_create()
#endif

#include <math.h>               // Required for: clamp()
#include <stdlib.h>             // Required for: malloc(), free()
#include "C:\raylib\raylib\src\utils.h"     // Required for: TraceLog()

// Functions required to query time on Windows
int __stdcall QueryPerformanceCounter(unsigned long long int *lpPerformanceCount);
int __stdcall QueryPerformanceFrequency(unsigned long long int *lpFrequency);

//----------------------------------------------------------------------------------
// Defines and Macros
//----------------------------------------------------------------------------------
#define     STATIC_DELTATIME        1.0/60.0

//----------------------------------------------------------------------------------
// Types and Structures Definition
//----------------------------------------------------------------------------------
typedef struct PhysicsManifoldData {
    int id;
    int index;
    PhysicsBody bodyA;
    PhysicsBody bodyB;
    float penetration;      // Depth of penetration from collision
    Vector2 normal;         // Normal direction vector from 'a' to 'b'
    Vector2 contacts[2];    // Points of contact during collision
    int contactsCount;      // Current collision number of contacts
    float e;                // Mixed restitution during collision
    float df;               // Mixed dynamic friction during collision
    float sf;               // Mixed static friction during collision
} PhysicsManifoldData, *PhysicsManifold;

//----------------------------------------------------------------------------------
// Global Variables Definition
//----------------------------------------------------------------------------------
static bool frameStepping = false;                          // Physics frame stepping state
static bool canStep = false;                                // Physics frame stepping input state
static bool physicsThreadEnabled = false;                   // Physics thread enabled state

static double currentTime = 0;                              // Current time in milliseconds
static double startTime = 0;                                // Start time in milliseconds
static double deltaTime = STATIC_DELTATIME;                 // Delta time used for physics steps
static double accumulator = 0;                              // Physics time step delta time accumulator

static int stepsCount = 0;                                  // Total physics steps processed
static Vector2 gravityForce = { 0, 0 };                     // Physics world gravity force

static PhysicsBody bodies[MAX_PHYSICS_BODIES];              // Physics bodies pointers array
static int physicsBodiesCount = 0;                          // Physics world current bodies counter

static PhysicsManifold contacts[MAX_PHYSICS_MANIFOLDS];     // Physics bodies pointers array
static int physicsManifoldsCount = 0;                       // Physics world current manifolds counter

static int usedMemory = 0;                                  // Total allocated dynamic memory

//----------------------------------------------------------------------------------
// Module Internal Functions Declaration
//----------------------------------------------------------------------------------
static void *PhysicsLoop(void *arg);                                                // Physics loop thread function
static void PhysicsStep(void);                                                      // Physics steps calculations (dynamics, collisions and position corrections)

static PhysicsManifold CreatePhysicsManifold(PhysicsBody a, PhysicsBody b);         // Creates a new physics manifold to solve collision
static void DestroyPhysicsManifold(PhysicsManifold manifold);                       // Unitializes and destroys a physics manifold
void SolvePhysicsManifold(PhysicsManifold manifold);                                // Solves a created physics manifold between two physics bodies
void IntegratePhysicsForces(PhysicsBody body);                                      // Integrates physics forces into velocity
void InitializePhysicsManifolds(PhysicsManifold manifold);                          // Initializes physics manifolds to solve collisions
void IntegratePhysicsImpulses(PhysicsManifold manifold);                            // Integrates physics collisions impulses to solve collisions
void IntegratePhysicsVelocity(PhysicsBody body);                                    // Integrates physics velocity into position and forces
void CorrectPhysicsPositions(PhysicsBody body);                                     // Corrects physics bodies positions

static double GetCurrentTime(void);                                                 // Get current time in milliseconds
static void MathClamp(double *value, double min, double max);                       // Clamp a value in a range

//----------------------------------------------------------------------------------
// Module Functions Definition
//----------------------------------------------------------------------------------
// Initializes physics values, pointers and creates physics loop thread
PHYSACDEF void InitPhysics(Vector2 gravity)
{
    TraceLog(WARNING, "[PHYSAC] physics module initialized successfully");
    
    // Initialize world gravity
    gravityForce = gravity;
    
    #ifndef PHYSAC_NO_THREADS
        // NOTE: if defined, user will need to create a thread for PhysicsThread function manually
        // Create physics thread using POSIXS thread libraries
        pthread_t tid;
        pthread_create(&tid, NULL, &PhysicsLoop, NULL);
    #endif
}

// Creates a new physics body with generic parameters
PhysicsBody CreatePhysicsBody(Vector2 pos, float mass)
{
    PhysicsBody newBody = (PhysicsBody)PHYSAC_MALLOC(sizeof(PhysicsBodyData));
    usedMemory += sizeof(PhysicsBodyData);
    
    int newId = -1;
    for (int i = 0; i < MAX_PHYSICS_BODIES; i++)
    {
        int currentId = i;
        
        // Check if current id already exist in other physics body
        for (int k = 0; k < physicsBodiesCount; k++)
        {
            if (bodies[k]->id == currentId)
            {
                currentId++;
                break;
            }
        }
        
        // If it is not used, use it as new physics body id
        if (currentId == i)
        {
            newId = i;
            break;
        }
    }
    
    if (newId != -1)
    {
        // Initialize new body with generic values
        newBody->id = newId;
        newBody->index = physicsBodiesCount;
        newBody->position = pos;    
        newBody->velocity = (Vector2){ 0, 0 };
        newBody->force = (Vector2){ 0, 0 };
        
        newBody->angularVelocity = 0;
        newBody->torque = 0;
        newBody->orient = 0;
        
        newBody->inertia = 0;
        newBody->inverseInertia = 0;
        newBody->mass = mass;
        newBody->inverseMass = 1/mass;
        
        newBody->shape.type = PHYSICS_CIRCLE;
        newBody->shape.body = newBody;
        newBody->shape.radius = 30;
        
        // Add new body to bodies pointers array and update bodies count
        bodies[physicsBodiesCount] = newBody;
        physicsBodiesCount++;
        
        TraceLog(WARNING, "[PHYSAC] created physics body id %i (index: %i) [USED RAM: %i bytes]", newBody->id, newBody->index, usedMemory);
    }
    else TraceLog(ERROR, "[PHYSAC] new physics body creation failed because there is any available id to use");
    
    return newBody;
}

// Unitializes and destroys a physics body
void DestroyPhysicsBody(PhysicsBody body)
{
    if (body != NULL)
    {
        int id = body->id;
        int index = body->index;
        
        // Free body allocated memory
        PHYSAC_FREE(bodies[index]);
        usedMemory -= sizeof(PhysicsBodyData);
        bodies[index] = NULL;
        
        // Reorder physics bodies pointers array and its catched index
        for (int i = index; i < physicsBodiesCount; i++)
        {
            if ((i + 1) < physicsBodiesCount)
            {
                bodies[i] = bodies[i + 1];
                bodies[i]->index = i;
            }
        }
        
        // Update physics bodies count
        physicsBodiesCount--;
        
        TraceLog(WARNING, "[PHYSAC] destroyed physics body id %i (index: %i) [USED RAM: %i bytes]", id, index, usedMemory);
    }
    else TraceLog(ERROR, "[PHYSAC] error trying to destroy a null referenced body");
}

// Draw all created physics bodies shapes
PHYSACDEF void DrawPhysicsBodies(void)
{
    for (int i = 0; i < physicsBodiesCount; i++)
    {
        if (bodies[i]->shape.type == PHYSICS_CIRCLE)
        {
            DrawCircleLines(bodies[i]->position.x, bodies[i]->position.y, bodies[i]->shape.radius, BLACK);
            
            Vector2 lineEndPos;
            lineEndPos.x = bodies[i]->position.x + cos(bodies[i]->orient)*bodies[i]->shape.radius;
            lineEndPos.y = bodies[i]->position.y + sin(bodies[i]->orient)*bodies[i]->shape.radius;
            
            DrawLineV(bodies[i]->position, lineEndPos, BLACK);
        }
        // TODO: add polygon shapes drawing
    }
}

// Draw debug information about physics states and values
PHYSACDEF void DrawPhysicsInfo(void)
{
    DrawText(FormatText("Steps: %i. Accumulator: %f", stepsCount, accumulator), 10, 10, 10, BLACK);
}

// Unitializes physics pointers and exits physics loop thread
PHYSACDEF void ClosePhysics(void)
{
    // Exit physics loop thread
    physicsThreadEnabled = false;
    
    // Unitialize physics bodies dynamic memory allocations
    int currentCount = physicsBodiesCount;
    for (int i = 0; i < currentCount; i++) DestroyPhysicsBody(bodies[i]);
    
    // Unitialize physics manifolds dynamic memory allocations
    currentCount = physicsManifoldsCount;
    for (int i = 0; i < currentCount; i++) DestroyPhysicsManifold(contacts[i]);
    
    if (physicsBodiesCount > 0 || usedMemory > 0) TraceLog(WARNING, "[PHYSAC] physics module closed with %i still allocated bodies [USED RAM: %i bytes]", physicsBodiesCount, usedMemory);
    else if (physicsManifoldsCount > 0 || usedMemory > 0) TraceLog(WARNING, "[PHYSAC] physics module closed with %i still allocated manifolds [USED RAM: %i bytes]", physicsManifoldsCount, usedMemory);
    else TraceLog(WARNING, "[PHYSAC] physics module closed successfully");
}

//----------------------------------------------------------------------------------
// Module Internal Functions Definition
//----------------------------------------------------------------------------------
// Physics loop thread function
static void *PhysicsLoop(void *arg)
{
    TraceLog(WARNING, "[PHYSAC] physics thread created with successfully");
    
    // Initialize physics loop thread values
    physicsThreadEnabled = true;
    accumulator = 0;
    
    // Initialize high resolution timer
    startTime = GetCurrentTime();
    
    // Physics update loop
    while (physicsThreadEnabled)
    {
        // Calculate current time
        currentTime = GetCurrentTime();
        
        // Store the time elapsed since the last frame began
        accumulator += currentTime - startTime;
        
        // Clamp accumulator to max time step to avoid bad performance
        MathClamp(&accumulator, 0.0, MAX_TIMESTEP);
        
        // Record the starting of this frame
        startTime = currentTime;
        
        // Fixed time stepping loop
        while (accumulator >= deltaTime)
        {
            if (!frameStepping) PhysicsStep();
            else
            {
                if (canStep)
                {
                    PhysicsStep();
                    canStep = false;
                }
            }
            
            accumulator -= deltaTime;
        }
        
        const double alpha = accumulator/deltaTime;
    }
    
    return NULL;
}

// Physics steps calculations (dynamics, collisions and position corrections)
static void PhysicsStep(void)
{
    stepsCount++;
    
    // Clear previous generated collisions information
    int currentCount = physicsManifoldsCount;
    for (int i = 0; i < currentCount; i++) DestroyPhysicsManifold(contacts[i]);
    
    // Generate new collision information
    for (int i = 0; i < physicsBodiesCount; i++)
    {
        PhysicsBody A = bodies[i];
        
        for (int j = i + 1; j < physicsBodiesCount; j++)
        {
            PhysicsBody B = bodies[j];
            
            if ((A->inverseMass == 0) && (B->inverseMass == 0)) continue;
            
            PhysicsManifold manifold = CreatePhysicsManifold(A, B);
            SolvePhysicsManifold(manifold);
            
            if (manifold->contactsCount > 0)
            {
                // Create a new manifold with same information as previously solved manifold and add it to the manifolds pool last slot
                PhysicsManifold newManifold = CreatePhysicsManifold(A, B);
                newManifold->penetration = manifold->penetration;
                newManifold->normal = manifold->normal;
                newManifold->contacts[0] = manifold->contacts[0];
                newManifold->contacts[1] = manifold->contacts[1];
                newManifold->contactsCount = manifold->contactsCount;
                newManifold->e = manifold->e;
                newManifold->df = manifold->df;
                newManifold->sf = manifold->sf;
            }
        }
    }
    
    // Integrate forces to physics bodies
    for (int i = 0; i < physicsBodiesCount; i++) IntegratePhysicsForces(bodies[i]);
    
    // Initialize physics manifolds to solve collisions
    for (int i = 0; i < physicsManifoldsCount; i++) InitializePhysicsManifolds(contacts[i]);
    
    // Integrate physics collisions impulses to solve collisions
    for (int i = 0; i < physicsManifoldsCount; i++) IntegratePhysicsImpulses(contacts[i]);
    
    // Integrate velocity to physics bodies
    for (int i = 0; i < physicsBodiesCount; i++) IntegratePhysicsVelocity(bodies[i]);
    
    // Correct physics bodies positions
    for (int i = 0; i < physicsBodiesCount; i++) CorrectPhysicsPositions(bodies[i]);
    
    // Clear physics bodies forces
    for (int i = 0; i < physicsBodiesCount; i++)
    {
        bodies[i]->force = (Vector2){ 0, 0 };
        bodies[i]->torque = 0;
    }
}

// Creates a new physics manifold to solve collision
static PhysicsManifold CreatePhysicsManifold(PhysicsBody a, PhysicsBody b)
{
    PhysicsManifold newManifold = (PhysicsManifold)PHYSAC_MALLOC(sizeof(PhysicsManifold));
    usedMemory += sizeof(PhysicsManifold);
    
    int newId = -1;
    for (int i = 0; i < MAX_PHYSICS_MANIFOLDS; i++)
    {
        int currentId = i;
        
        // Check if current id already exist in other physics body
        for (int k = 0; k < physicsManifoldsCount; k++)
        {
            if (contacts[k]->id == currentId)
            {
                currentId++;
                break;
            }
        }
        
        // If it is not used, use it as new physics body id
        if (currentId == i)
        {
            newId = i;
            break;
        }
    }
    
    if (newId != -1)
    {    
        // Initialize new manifold with generic values
        newManifold->id = newId;
        newManifold->index = physicsManifoldsCount;
        newManifold->bodyA = a;
        newManifold->bodyB = b;
        newManifold->penetration = 0;
        newManifold->normal = (Vector2){ 0, 0 };
        newManifold->contacts[0] = (Vector2){ 0, 0 };
        newManifold->contacts[1] = (Vector2){ 0, 0 };
        newManifold->contactsCount = 0;
        newManifold->e = 0;
        newManifold->df = 0;
        newManifold->sf = 0;
        
        // Add new body to bodies pointers array and update bodies count
        contacts[physicsManifoldsCount] = newManifold;
        physicsManifoldsCount++;
        
        TraceLog(WARNING, "[PHYSAC] created physics manifold id %i (index: %i) with physics bodies id %i and %i [USED RAM: %i bytes]", newManifold->id, newManifold->index, newManifold->bodyA->id, newManifold->bodyB->id, usedMemory);
    }
    else TraceLog(ERROR, "[PHYSAC] new physics manifold creation failed because there is any available id to use");
    
    return newManifold;
}

// Unitializes and destroys a physics manifold
static void DestroyPhysicsManifold(PhysicsManifold manifold)
{
    if (manifold != NULL)
    {
        int id = manifold->id;
        int index = manifold->index;
        
        // Free manifold allocated memory
        PHYSAC_FREE(contacts[index]);
        usedMemory -= sizeof(PhysicsManifold);
        contacts[index] = NULL;
        
        // Reorder physics manifolds pointers array and its catched index
        for (int i = index; i < physicsManifoldsCount; i++)
        {
            if ((i + 1) < physicsBodiesCount)
            {
                contacts[i] = contacts[i + 1];
                contacts[i]->index = i;
            }
        }
        
        // Update physics manifolds count
        physicsManifoldsCount--;
        
        TraceLog(WARNING, "[PHYSAC] destroyed physics manifold id %i (index: %i) [USED RAM: %i bytes]", id, index, usedMemory);
    }
    else TraceLog(ERROR, "[PHYSAC] error trying to destroy a null referenced manifold");
}

// Solves a created physics manifold between two physics bodies
void SolvePhysicsManifold(PhysicsManifold manifold)
{
    // TODO: implement it
}

// Integrates physics forces into velocity
void IntegratePhysicsForces(PhysicsBody body)
{
    if (body->inverseMass != 0)
    {
        body->velocity.x += (body->force.x*body->inverseMass + gravityForce.x)*(deltaTime/2);
        body->velocity.y += (body->force.y*body->inverseMass + gravityForce.y)*(deltaTime/2);
        
        body->angularVelocity += body->torque*body->inverseInertia*(deltaTime/2);
    }
}

// Initializes physics manifolds to solve collisions
void InitializePhysicsManifolds(PhysicsManifold manifold)
{
    // TODO: implement it
}

// Integrates physics collisions impulses to solve collisions
void IntegratePhysicsImpulses(PhysicsManifold manifold)
{
    // TODO: implement it
}

// Integrates physics velocity into position and forces
void IntegratePhysicsVelocity(PhysicsBody body)
{
    if (body->inverseMass != 0)
    {
        body->position.x += body->velocity.x*deltaTime;
        body->position.y -= body->velocity.y*deltaTime;
        
        body->orient += body->angularVelocity*deltaTime;
        
        // TODO: create set orient function for polygon shapes
        
        IntegratePhysicsForces(body);
    }
}

// Corrects physics bodies positions
void CorrectPhysicsPositions(PhysicsBody body)
{
    // TODO: implement it
}

// Get current time in milliseconds
static double GetCurrentTime(void)
{
    double time = 0;
    
    unsigned long long int clockFrequency, currentTime;
    
    QueryPerformanceFrequency(&clockFrequency);
    QueryPerformanceCounter(&currentTime);
    
    time = (double)((double)currentTime/clockFrequency)*1000;

    return time;
}

// Clamp a value in a range
static void MathClamp(double *value, double min, double max)
{
    if (*value < min) *value = min;
    else if (*value > max) *value = max;
}

#endif  // PHYSAC_IMPLEMENTATION
