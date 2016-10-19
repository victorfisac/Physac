/**********************************************************************************************
*
*   Physac - 2D Physics library for videogames
*
*   Description: Physac is a small 2D physics engine written in pure C. The engine uses a fixed time-step thread loop 
*   to simluate physics. A physics step contains the following phases: get collision information, apply dynamics, 
*   collision solving and position correction. It uses a very simple struct for physic bodies with a position vector 
*   to be used in any 3D rendering API.
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

// #define PHYSAC_STANDALONE

#ifndef PHYSAC_STANDALONE
    #include "raylib.h"
#endif

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
#define     COLLISION_ITERATIONS            100
#define     MAX_TIMESTEP                    0.02
#define     MAX_PHYSICS_BODIES              2048
#define     MAX_PHYSICS_MANIFOLDS           2048
#define     PHYSAC_MAX_VERTICES             8
#define     PHYSAC_MALLOC(size)             malloc(size)
#define     PHYSAC_FREE(ptr)                free(ptr)
#define     PHYSAC_SHAPES_COLOR             GREEN
#define     PHYSAC_INFO_COLOR               WHITE
#define     PHYSAC_CONTACTS_RADIUS          3
#define     PHYSAC_CONTACTS_NORMAL          10
#define     PHYSAC_CONTACTS_COLOR           RED

//----------------------------------------------------------------------------------
// Types and Structures Definition
// NOTE: Below types are required for PHYSAC_STANDALONE usage
//----------------------------------------------------------------------------------
#if defined(PHYSAC_STANDALONE)
    typedef struct Vector2 {
        float x;
        float y;
    } Vector2;
    
    // Boolean type
    #if !defined(_STDBOOL_H)
        typedef enum { false, true } bool;
        #define _STDBOOL_H
    #endif
#endif

typedef enum PhysicsShapeType { PHYSICS_CIRCLE, PHYSICS_POLYGON } PhysicsShapeType;

// Previously defined to be used in PhysicsShape struct as circular dependencies
typedef struct PhysicsBodyData *PhysicsBody;

typedef struct Mat2
{
    float m00;
    float m01;
    float m10;
    float m11;
} Mat2;

typedef struct PolygonData {
    Mat2 transform;                             // Vertices transform matrix 2x2
    unsigned int vertexCount;                   // Current used vertex and normals count
    Vector2 vertices[PHYSAC_MAX_VERTICES];      // Polygon vertex positions vectors
    Vector2 normals[PHYSAC_MAX_VERTICES];       // Polygon vertex normals vectors
} PolygonData;

typedef struct PhysicsShape {
    PhysicsShapeType type;
    PhysicsBody body;
    
    // Used for circle shapes
    float radius;                       // Circle shape radius (used for drawing and mass/inertia computation)

    // Used for polygon shapes
    PolygonData vertexData;             // Polygon shape vertices position and normals data
} PhysicsShape;

typedef struct PhysicsBodyData {
    unsigned int id;
    Vector2 position;           // Physics body shape pivot
    Vector2 velocity;           // Current linear velocity applied to position
    Vector2 force;              // Current linear force (reset to 0 every step)
    
    float angularVelocity;      // Current angular velocity applied to orient
    float torque;               // Current angular force (reset to 0 every step)
    float orient;               // Rotation in radians
    
    float inertia;              // Moment of inertia
    float inverseInertia;       // Inverse value of inertia
    float mass;                 // Physics body mass
    float inverseMass;          // Inverse value of mass
    
    float staticFriction;       // Friction when the body has not movement (0 to 1)
    float dynamicFriction;      // Friction when the body has movement (0 to 1)
    float restitution;          // Restitution coefficient of the body (0 to 1)
    
    bool enabled;               // Enabled dynamics state (collisions are calculated anyway)
    bool useGravity;            // Apply gravity force to dynamics
    
    PhysicsShape shape;         // Physics body shape information (type, radius, vertices, normals)
} PhysicsBodyData;

typedef struct PhysicsManifoldData {
    int id;
    PhysicsBody bodyA;              // Manifold first physics body reference
    PhysicsBody bodyB;              // Manifold second physics body reference
    float penetration;              // Depth of penetration from collision
    Vector2 normal;                 // Normal direction vector from 'a' to 'b'
    Vector2 contacts[2];            // Points of contact during collision
    unsigned int contactsCount;     // Current collision number of contacts
    float e;                        // Mixed restitution during collision
    float df;                       // Mixed dynamic friction during collision
    float sf;                       // Mixed static friction during collision
} PhysicsManifoldData, *PhysicsManifold;

//----------------------------------------------------------------------------------
// Module Functions Declaration
//----------------------------------------------------------------------------------
PHYSACDEF void InitPhysics(Vector2 gravity);                                                    // Initializes physics values, pointers and creates physics loop thread

PHYSACDEF PhysicsBody CreatePhysicsBodyCircle(Vector2 pos, float density, float radius);        // Creates a new physics body with generic parameters
PHYSACDEF PhysicsBody CreatePhysicsBodyPolygon(Vector2 pos, float density);                     // Creates a new physics body with generic parameters

PHYSACDEF void DrawPhysicsBodies(void);                 // Draws all created physics bodies shapes
PHYSACDEF void DrawPhysicsContacts(void);               // Draws all calculated physics contacts points and its normals
PHYSACDEF void DrawPhysicsInfo(void);                   // Draws debug information about physics states and values

PHYSACDEF void DestroyPhysicsBody(PhysicsBody body);    // Unitializes and destroy a physics body
PHYSACDEF void ClosePhysics(void);                      // Unitializes physics pointers and closes physics loop thread

#endif // PHYSAC_H

/***********************************************************************************
*
*   PHYSAC IMPLEMENTATION
*
************************************************************************************/

#if defined(PHYSAC_IMPLEMENTATION)

#ifndef PHYSAC_NO_THREADS
    #include <pthread.h>                    // Required for: pthread_t, pthread_create()
#endif

#include <math.h>                           // Required for: sqrt()
#include <stdlib.h>                         // Required for: malloc(), free()
#include "C:\raylib\raylib\src\utils.h"     // Required for: TraceLog()

// Functions required to query time on Windows
int __stdcall QueryPerformanceCounter(unsigned long long int *lpPerformanceCount);
int __stdcall QueryPerformanceFrequency(unsigned long long int *lpFrequency);

//----------------------------------------------------------------------------------
// Defines and Macros
//----------------------------------------------------------------------------------
#define     STATIC_DELTATIME            1.0/60.0
#define     min(a,b)                    (((a)<(b))?(a):(b))
#define     max(a,b)                    (((a)>(b))?(a):(b))
#define     MATH_FLT_MAX                3.402823466e+38f
#define     MATH_PI                     3.14159265358979323846
#define     MATH_EPSILON                0.000001f
#define     MATH_DEG2RAD                (MATH_PI/180.0f)
#define     MATH_RAD2DEG                (180.0f/MATH_PI)
#define     PENETRATION_ALLOWANCE       0.05f
#define     PENETRATION_CORRECTION      0.4f

//----------------------------------------------------------------------------------
// Types and Structures Definition
//----------------------------------------------------------------------------------
// ...

//----------------------------------------------------------------------------------
// Global Variables Definition
//----------------------------------------------------------------------------------
static unsigned int usedMemory = 0;                         // Total allocated dynamic memory
static bool frameStepping = false;                          // Physics frame stepping state
static bool canStep = false;                                // Physics frame stepping input state
static bool physicsThreadEnabled = false;                   // Physics thread enabled state

static double currentTime = 0;                              // Current time in milliseconds
static double startTime = 0;                                // Start time in milliseconds
static double deltaTime = STATIC_DELTATIME;                 // Delta time used for physics steps
static double accumulator = 0;                              // Physics time step delta time accumulator

static unsigned int stepsCount = 0;                         // Total physics steps processed
static Vector2 gravityForce = { 0 };                        // Physics world gravity force

static PhysicsBody bodies[MAX_PHYSICS_BODIES];              // Physics bodies pointers array
static unsigned int physicsBodiesCount = 0;                 // Physics world current bodies counter

static PhysicsManifold contacts[MAX_PHYSICS_MANIFOLDS];     // Physics bodies pointers array
static unsigned int physicsManifoldsCount = 0;              // Physics world current manifolds counter

//----------------------------------------------------------------------------------
// Module Internal Functions Declaration
//----------------------------------------------------------------------------------
static PolygonData CreateRandomPolygon(int minDistance, int maxDistance);           // Creates a random polygon shape with max vertex distance from polygon pivot
static void *PhysicsLoop(void *arg);                                                // Physics loop thread function
static void PhysicsStep(void);                                                      // Physics steps calculations (dynamics, collisions and position corrections)

static PhysicsManifold CreatePhysicsManifold(PhysicsBody a, PhysicsBody b);         // Creates a new physics manifold to solve collision
static void DestroyPhysicsManifold(PhysicsManifold manifold);                       // Unitializes and destroys a physics manifold
static void SolvePhysicsManifold(PhysicsManifold manifold);                         // Solves a created physics manifold between two physics bodies
static void SolvePhysicsCircleToCircle(PhysicsManifold manifold);                   // Solves collision between two circle shape physics bodies
static void SolvePhysicsCircleToPolygon(PhysicsManifold manifold);                  // Solves collision between a circle to a polygon shape physics bodies
static void SolvePhysicsPolygonToCircle(PhysicsManifold manifold);                  // Solves collision between a polygon to a circle shape physics bodies
static void SolvePhysicsPolygonToPolygon(PhysicsManifold manifold);                 // Solves collision between two polygons shape physics bodies
static void IntegratePhysicsForces(PhysicsBody body);                               // Integrates physics forces into velocity
static void InitializePhysicsManifolds(PhysicsManifold manifold);                   // Initializes physics manifolds to solve collisions
static void IntegratePhysicsImpulses(PhysicsManifold manifold);                     // Integrates physics collisions impulses to solve collisions
static void IntegratePhysicsVelocity(PhysicsBody body);                             // Integrates physics velocity into position and forces
static void CorrectPhysicsPositions(PhysicsManifold manifold);                      // Corrects physics bodies positions based on manifolds collision information

static double GetCurrentTime(void);                                                 // Get current time in milliseconds
static void MathClamp(double *value, double min, double max);                       // Clamp a value in a range
static Vector2 MathCross(float a, Vector2 v);                                       // Returns the cross product of a vector and a value
static float MathCrossVector2(Vector2 a, Vector2 b);                                // Returns the cross product of two vectors
static float MathLenSqr(Vector2 v);                                                 // Returns the len square root of a vector
static float MathDot(Vector2 a, Vector2 b);                                         // Returns the dot product of two vectors
static void MathNormalize(Vector2 *v);                                              // Returns the normalized values of a vector
static Vector2 Vector2Add(Vector2 a, Vector2 b);                                    // Returns the sum of two given vectors
static Vector2 Vector2Subtract(Vector2 a, Vector2 b);                               // Returns the subtract of two given vectors

static Mat2 Mat2Radians(float radians);                                             // Creates a matrix 2x2 from a given radians value
static Mat2 CreateMat2(float a, float b, float c, float d);                         // Creates a matrix 2x2 from specific values
static void Mat2Set(Mat2 *matrix, float radians);                                   // Set values from radians to a created matrix 2x2
static void Mat2Abs(Mat2 *matrix, float radians);                                   // Converts to absolute value every matrix values
static Vector2 Mat2AxisX(Mat2 matrix);                                              // Returns m00 and m10 as a Vector2 struct of a matrix 2x2
static Vector2 Mat2AxisY(Mat2 matrix);                                              // Returns m01 and m11 as a Vector2 struct of a matrix 2x2
static Mat2 Mat2Transpose(Mat2 m);                                                  // Returns the transpose of a given matrix 2x2
static Vector2 Mat2MultiplyVector2(Mat2 m, Vector2 v);                              // Multiplies a vector by a matrix 2x2
static Mat2 Mat2Multiply(Mat2 m1, Mat2 m2);                                         // Multiplies two given matrices 2x2
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
PHYSACDEF PhysicsBody CreatePhysicsBodyCircle(Vector2 pos, float density, float radius)
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
        newBody->position = pos;    
        newBody->velocity = (Vector2){ 0 };
        newBody->force = (Vector2){ 0 };
        
        newBody->angularVelocity = 0;
        newBody->torque = 0;
        newBody->orient = 0;
        
        newBody->mass = MATH_PI*radius*radius*density;
        newBody->inverseMass = ((newBody->mass != 0.0f) ? 1.0f/newBody->mass : 0.0f);
        newBody->inertia = newBody->mass*radius*radius;
        newBody->inverseInertia = ((newBody->inertia != 0.0f) ? 1.0f/newBody->inertia : 0.0f);
        
        newBody->staticFriction = 0.4f;
        newBody->dynamicFriction = 0.2f;
        newBody->restitution = 0.2;
        
        newBody->enabled = true;
        newBody->useGravity = true;
        
        newBody->shape.type = PHYSICS_CIRCLE;
        newBody->shape.body = newBody;
        newBody->shape.radius = radius;
        
        // Add new body to bodies pointers array and update bodies count
        bodies[physicsBodiesCount] = newBody;
        physicsBodiesCount++;
        
        TraceLog(WARNING, "[PHYSAC] created circle physics body id %i [USED RAM: %i bytes]", newBody->id, usedMemory);
    }
    else TraceLog(ERROR, "[PHYSAC] new physics body creation failed because there is any available id to use");
    
    return newBody;
}

// Creates a new physics body with generic parameters
PHYSACDEF PhysicsBody CreatePhysicsBodyPolygon(Vector2 pos, float density)
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
        newBody->position = pos;    
        newBody->velocity = (Vector2){ 0 };
        newBody->force = (Vector2){ 0 };
        
        newBody->angularVelocity = 0;
        newBody->torque = 0;
        newBody->orient = 0;
        
        newBody->shape.type = PHYSICS_POLYGON;
        newBody->shape.body = newBody;
        newBody->shape.vertexData = CreateRandomPolygon(25, 200);
        
        // Calculate centroid and moment of inertia
        Vector2 center = { 0 };
        float area = 0;
        float inertia = 0;
        const float k = 1.0f/3.0f;
        
        for (int i = 0; i < newBody->shape.vertexData.vertexCount; i++)
        {
            // Triangle vertices, third vertex implied as (0, 0)
            Vector2 p1 = newBody->shape.vertexData.vertices[i];
            int ii = (((i + 1) < newBody->shape.vertexData.vertexCount) ? (i + 1) : 0);
            Vector2 p2 = newBody->shape.vertexData.vertices[ii];
            
            float D = MathCrossVector2(p1, p2);
            float triangleArea = D/2;
            
            area += triangleArea;
            
            // Use area to weight the centroid average, not just vertex position
            center.x += triangleArea*k*(p1.x + p2.x);
            center.y += triangleArea*k*(p1.y + p2.y);
            
            float intx2 = p1.x*p1.x + p2.x*p1.x + p2.x*p2.x;
            float inty2 = p1.y*p1.y + p2.y*p1.y + p2.y*p2.y;
            inertia += (0.25f*k*D)*(intx2 + inty2);
        }
        
        center.x *= 1.0f/area;
        center.y *= 1.0f/area;
        
        // Translate vertices to centroid (make the centroid (0, 0) for the polygon in model space)
        // Note: this is not really necessary
        for (int i = 0; i < newBody->shape.vertexData.vertexCount; i++)
        {
            newBody->shape.vertexData.vertices[i].x -= center.x;
            newBody->shape.vertexData.vertices[i].y -= center.y;
        }
        
        newBody->mass = density*area;
        newBody->inverseMass = ((newBody->mass != 0.0f) ? 1.0f/newBody->mass : 0.0f);
        newBody->inertia = density*inertia;
        newBody->inverseInertia = ((newBody->inertia != 0.0f) ? 1.0f/newBody->inertia : 0.0f);
        
        newBody->staticFriction = 0.4f;
        newBody->dynamicFriction = 0.2f;
        newBody->restitution = 0.2;
        
        newBody->enabled = true;
        newBody->useGravity = true;

        // Add new body to bodies pointers array and update bodies count
        bodies[physicsBodiesCount] = newBody;
        physicsBodiesCount++;
        
        TraceLog(WARNING, "[PHYSAC] created polygon physics body id %i [USED RAM: %i bytes]", newBody->id, usedMemory);
    }
    else TraceLog(ERROR, "[PHYSAC] new physics body creation failed because there is any available id to use");
    
    return newBody;
}

// Draws all created physics bodies shapes
PHYSACDEF void DrawPhysicsBodies(void)
{
    for (int i = 0; i < physicsBodiesCount; i++)
    {
        PhysicsBody body = bodies[i];
        
        if (body != NULL)
        {
            switch (bodies[i]->shape.type)
            {
                case PHYSICS_CIRCLE:
                {
                    DrawCircleLines(body->position.x, body->position.y, body->shape.radius, PHYSAC_SHAPES_COLOR);
                    
                    Vector2 lineEndPos;
                    lineEndPos.x = body->position.x + cos(body->orient)*body->shape.radius;
                    lineEndPos.y = body->position.y + sin(body->orient)*body->shape.radius;
                    
                    DrawLineV(body->position, lineEndPos, PHYSAC_SHAPES_COLOR);
                } break;
                case PHYSICS_POLYGON:
                {
                    PolygonData data = body->shape.vertexData;
                    Vector2 position = { 0 };
                    
                    for (int i = 0; i < data.vertexCount; i++)
                    {                        
                        // Draw vertex face with lines
                        int ii = (((i + 1) < data.vertexCount) ? (i + 1) : 0);
                        Vector2 vertexA = Mat2MultiplyVector2(data.transform, data.vertices[i]);
                        Vector2 startPosition = Vector2Add(body->position, vertexA);
                        
                        Vector2 vertexB = Mat2MultiplyVector2(data.transform, data.vertices[ii]);
                        Vector2 endPosition = Vector2Add(body->position, vertexB);
                        
                        DrawLineV(startPosition, endPosition, PHYSAC_SHAPES_COLOR);
                        
                        startPosition.x = data.vertices[i].x + (data.vertices[ii].x - data.vertices[i].x)/2;
                        startPosition.y = data.vertices[i].y + (data.vertices[ii].y - data.vertices[i].y)/2;
                        
                        startPosition = Mat2MultiplyVector2(data.transform, startPosition);
                        
                        startPosition = Vector2Add(startPosition, body->position);
                        
                        // Draw vertex normals with lines             
                        vertexA = Mat2MultiplyVector2(data.transform, data.normals[i]);
                        endPosition.x = startPosition.x + vertexA.x*PHYSAC_CONTACTS_NORMAL;
                        endPosition.y = startPosition.y + vertexA.y*PHYSAC_CONTACTS_NORMAL;
                        
                        DrawLineV(startPosition, endPosition, PHYSAC_CONTACTS_COLOR);
                    }
                } break;
            }
        }
    }
}

// Draws all calculated physics contacts points and its normals
PHYSACDEF void DrawPhysicsContacts(void)
{
    for (int i = 0; i < physicsManifoldsCount; i++)
    {
        PhysicsManifold manifold = contacts[i];
        
        if (manifold != NULL)
        {
            for (int j = 0; j < manifold->contactsCount; j++)
            {
                DrawCircleV(manifold->contacts[j], PHYSAC_CONTACTS_RADIUS, PHYSAC_CONTACTS_COLOR);
                
                Vector2 startPosition = manifold->contacts[j];
                Vector2 endPosition = { startPosition.x + manifold->normal.x*PHYSAC_CONTACTS_NORMAL, startPosition.y + manifold->normal.y*PHYSAC_CONTACTS_NORMAL };
                
                DrawLineV(startPosition, endPosition, PHYSAC_CONTACTS_COLOR);
            }
        }
    }
}

// Draws debug information about physics states and values
PHYSACDEF void DrawPhysicsInfo(void)
{
    DrawText(FormatText("Steps: %i. Accumulator: %f", stepsCount, accumulator), 10, 10, 10, PHYSAC_INFO_COLOR);
}

// Unitializes and destroys a physics body
PHYSACDEF void DestroyPhysicsBody(PhysicsBody body)
{
    if (body != NULL)
    {
        int id = body->id;
        int index = -1;
        
        for (int i = 0; i < physicsBodiesCount; i++)
        {
            if (bodies[i]->id == id)
            {
                index = i;
                break;
            }
        }
        
        if (index == -1) TraceLog(ERROR, "[PHYSAC] cannot find body id %i in pointers array", id);
        
        // Free body allocated memory
        PHYSAC_FREE(bodies[index]);
        usedMemory -= sizeof(PhysicsBodyData);
        bodies[index] = NULL;
        
        // Reorder physics bodies pointers array and its catched index
        for (int i = index; i < physicsBodiesCount; i++)
        {
            if ((i + 1) < physicsBodiesCount) bodies[i] = bodies[i + 1];
        }
        
        // Update physics bodies count
        physicsBodiesCount--;
        
        TraceLog(WARNING, "[PHYSAC] destroyed physics body id %i (index: %i) [USED RAM: %i bytes]", id, index, usedMemory);
    }
    else TraceLog(ERROR, "[PHYSAC] error trying to destroy a null referenced body");
}

// Unitializes physics pointers and exits physics loop thread
PHYSACDEF void ClosePhysics(void)
{
    // Exit physics loop thread
    physicsThreadEnabled = false;
    
    // Unitialize physics bodies dynamic memory allocations
    int currentCount = physicsBodiesCount;
    for (int i = physicsBodiesCount - 1; i >= 0; i--) DestroyPhysicsBody(bodies[i]);
    
    // Unitialize physics manifolds dynamic memory allocations
    for (int i = physicsManifoldsCount - 1; i >= 0; i--) DestroyPhysicsManifold(contacts[i]);
    
    if (physicsBodiesCount > 0 || usedMemory > 0) TraceLog(WARNING, "[PHYSAC] physics module closed with %i still allocated bodies [USED RAM: %i bytes]", physicsBodiesCount, usedMemory);
    else if (physicsManifoldsCount > 0 || usedMemory > 0) TraceLog(WARNING, "[PHYSAC] physics module closed with %i still allocated manifolds [USED RAM: %i bytes]", physicsManifoldsCount, usedMemory);
    else TraceLog(WARNING, "[PHYSAC] physics module closed successfully");
}

//----------------------------------------------------------------------------------
// Module Internal Functions Definition
//----------------------------------------------------------------------------------
// Creates a random polygon shape with max vertex distance from polygon pivot
static PolygonData CreateRandomPolygon(int minDistance, int maxDistance)
{
    PolygonData data = { 0 };
    
    unsigned int count = GetRandomValue(3, PHYSAC_MAX_VERTICES);
    data.vertexCount = count;
    
    float orient = GetRandomValue(0, 360);
    data.transform = Mat2Radians(orient*MATH_DEG2RAD);
    
    int distance = GetRandomValue(minDistance, maxDistance);
    
    // Calculate polygon vertices positions
    for (int i = 0; i < count; i++)
    {
        data.vertices[i].x = cos(360/count*i*MATH_DEG2RAD)*distance;
        data.vertices[i].y = sin(360/count*i*MATH_DEG2RAD)*distance;
    }
    
    // Calculate polygon faces normals
    for (int i = 0; i < count; i++)
    {
        int ii = (((i + 1) < count) ? (i + 1) : 0);
        Vector2 face = Vector2Subtract(data.vertices[ii], data.vertices[i]);
        
        data.normals[i] = (Vector2){ face.y, -face.x };
        MathNormalize(&data.normals[i]);
    }
    
    return data;
}

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
    for (int i = physicsManifoldsCount - 1; i >= 0; i--) DestroyPhysicsManifold(contacts[i]);
    
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
    for (int i = 0; i < COLLISION_ITERATIONS; i++)
    {
        for (int j = 0; j < physicsManifoldsCount; j++) IntegratePhysicsImpulses(contacts[j]);
    }
    
    // Integrate velocity to physics bodies
    for (int i = 0; i < physicsBodiesCount; i++) IntegratePhysicsVelocity(bodies[i]);

    // Correct physics bodies positions based on manifolds collision information
    for (int i = 0; i < physicsManifoldsCount; i++) CorrectPhysicsPositions(contacts[i]);
    
    // Clear physics bodies forces
    for (int i = 0; i < physicsBodiesCount; i++)
    {
        bodies[i]->force = (Vector2){ 0 };
        bodies[i]->torque = 0;
    }
}

// Creates a new physics manifold to solve collision
static PhysicsManifold CreatePhysicsManifold(PhysicsBody a, PhysicsBody b)
{
    PhysicsManifold newManifold = (PhysicsManifold)PHYSAC_MALLOC(sizeof(PhysicsManifoldData));
    usedMemory += sizeof(PhysicsManifoldData);
    
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
        newManifold->bodyA = a;
        newManifold->bodyB = b;
        newManifold->penetration = 0;
        newManifold->normal = (Vector2){ 0 };
        newManifold->contacts[0] = (Vector2){ 0 };
        newManifold->contacts[1] = (Vector2){ 0 };
        newManifold->contactsCount = 0;
        newManifold->e = 0;
        newManifold->df = 0;
        newManifold->sf = 0;
        
        // Add new body to bodies pointers array and update bodies count
        contacts[physicsManifoldsCount] = newManifold;
        physicsManifoldsCount++;
        
        // Avoided trace log due to bad performance for tracing messages each physics step
        // TraceLog(WARNING, "[PHYSAC] created physics manifold id %i with physics bodies id %i and %i [USED RAM: %i bytes]", newManifold->id, newManifold->bodyA->id, newManifold->bodyB->id, usedMemory);
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
        int index = -1;
        
        for (int i = 0; i < physicsManifoldsCount; i++)
        {
            if (contacts[i]->id == id)
            {
                index = i;
                break;
            }
        }
        
        if (index == -1) TraceLog(ERROR, "[PHYSAC] cannot find manifold id %i in pointers array", id);
        
        // Free manifold allocated memory
        PHYSAC_FREE(contacts[index]);
        usedMemory -= sizeof(PhysicsManifoldData);
        contacts[index] = NULL;
        
        // Reorder physics manifolds pointers array and its catched index
        for (int i = index; i < physicsManifoldsCount; i++)
        {
            if ((i + 1) < physicsManifoldsCount) contacts[i] = contacts[i + 1];
        }
        
        // Update physics manifolds count
        physicsManifoldsCount--;
        
        // Avoided trace log due to bad performance for tracing messages each physics step
        // TraceLog(WARNING, "[PHYSAC] destroyed physics manifold id %i (index: %i) [USED RAM: %i bytes]", id, index, usedMemory);
    }
    else TraceLog(ERROR, "[PHYSAC] error trying to destroy a null referenced manifold");
}

// Solves a created physics manifold between two physics bodies
static void SolvePhysicsManifold(PhysicsManifold manifold)
{
    switch (manifold->bodyA->shape.type)
    {
        case PHYSICS_CIRCLE:
        {
            switch (manifold->bodyB->shape.type)
            {
                case PHYSICS_CIRCLE: SolvePhysicsCircleToCircle(manifold); break;
                case PHYSICS_POLYGON: SolvePhysicsCircleToPolygon(manifold); break;
                default: break;
            }
        } break;
        case PHYSICS_POLYGON:
        {
            switch (manifold->bodyB->shape.type)
            {
                case PHYSICS_CIRCLE: SolvePhysicsPolygonToCircle(manifold); break;
                case PHYSICS_POLYGON: SolvePhysicsPolygonToPolygon(manifold); break;
                default: break;
            }
        } break;
        default: break;
    }
}

// Solves collision between two circle shape physics bodies
static void SolvePhysicsCircleToCircle(PhysicsManifold manifold)
{
    PhysicsBody A = manifold->bodyA;
    PhysicsBody B = manifold->bodyB;
    
    // Calculate translational vector, which is normal
    Vector2 normal = Vector2Subtract(B->position, A->position);
    
    float dist_sqr = MathLenSqr(normal);
    float radius = A->shape.radius + B->shape.radius;
    
    // Check if circles are not in contact
    if (dist_sqr >= radius*radius) manifold->contactsCount = 0;
    else
    {
        float distance = sqrt(dist_sqr);
        manifold->contactsCount = 1;
        
        if (distance == 0)
        {
            manifold->penetration = A->shape.radius;
            manifold->normal = (Vector2){ 1, 0 };
            manifold->contacts[0] = A->position;
        }
        else
        {
            manifold->penetration = radius - distance;
            manifold->normal = (Vector2){ normal.x/distance, normal.y/distance }; // Faster than using MathNormalize() due to sqrt is already performed
            manifold->contacts[0] = (Vector2){ manifold->normal.x*A->shape.radius + A->position.x, manifold->normal.y*A->shape.radius + A->position.y };
        }
    }
}

// Solves collision between a circle to a polygon shape physics bodies
static void SolvePhysicsCircleToPolygon(PhysicsManifold manifold)
{
    PhysicsBody A = manifold->bodyA;
    PhysicsBody B = manifold->bodyB;
    
    manifold->contactsCount = 0;
    
    // Transform circle center to polygon transform space
    Vector2 center = A->position;
    center = Mat2MultiplyVector2(Mat2Transpose(B->shape.vertexData.transform), Vector2Subtract(center, B->position));
    
    // Find edge with minimum penetration
    // It is the same concept as using support points in SolvePhysicsPolygonToPolygon
    float separation = -MATH_FLT_MAX;
    int faceNormal = 0;
    PolygonData vertexData = B->shape.vertexData;
    
    for (int i = 0; i < B->shape.vertexData.vertexCount; i++)
    {
        float s = MathDot(vertexData.normals[i], Vector2Subtract(center, vertexData.vertices[i]));
        
        if (s > A->shape.radius) return;
        
        if (s > separation)
        {
            separation = s;
            faceNormal = i;
        }
    }
    
    // Grab face's vertices
    Vector2 v1 = vertexData.vertices[faceNormal];
    int ii = (((faceNormal + 1) < vertexData.vertexCount) ? (faceNormal + 1) : 0);
    Vector2 v2 = vertexData.vertices[ii];
    
    // Check to see if center is within polygon
    if (separation < MATH_EPSILON)
    {
        manifold->contactsCount = 1;
        Vector2 normal = Mat2MultiplyVector2(vertexData.transform, vertexData.normals[faceNormal]);
        manifold->normal = (Vector2){ -normal.x, -normal.y };
        manifold->contacts[0] = (Vector2){ manifold->normal.x*A->shape.radius + A->position.x, manifold->normal.y*A->shape.radius + A->position.y };
        manifold->penetration = A->shape.radius;
        return;
    }
    
    // Determine which voronoi region of the edge center of circle lies within
    float dot1 = MathDot((Vector2){ center.x - v1.x, center.y - v1.y }, (Vector2){ v2.x - v1.x, v2.y - v1.y });
    float dot2 = MathDot((Vector2){ center.x - v2.x, center.y - v2.y }, (Vector2){ v1.x - v2.x, v1.y - v2.y });
    manifold->penetration = A->shape.radius - separation;
  
    if (dot1 <= 0) // Closest to v1
    {
        Vector2 c = Vector2Subtract(center, v1);
        float distSqr = MathDot(c, c);
        
        if (distSqr > A->shape.radius*A->shape.radius) return;
        
        manifold->contactsCount = 1;
        Vector2 normal = Vector2Subtract(v1, center);
        normal = Mat2MultiplyVector2(vertexData.transform, normal);
        MathNormalize(&normal);
        manifold->normal = normal;
        v1 = Mat2MultiplyVector2(vertexData.transform, v1);
        v1 = Vector2Add(v1, B->position);
        manifold->contacts[0] = v1;
    }
    else if (dot2 <= 0) // Closest to v2
    {
        Vector2 c = { center.x - v2.x, center.y - v2.y };
        float distSqr = MathDot(c, c);
        
        if (distSqr > A->shape.radius*A->shape.radius) return;
        
        manifold->contactsCount = 1;
        Vector2 normal = Vector2Subtract(v2, center);
        v2 = Mat2MultiplyVector2(vertexData.transform, v2);
        v2 = Vector2Add(v2, B->position);
        manifold->contacts[0] = v2;
        normal = Mat2MultiplyVector2(vertexData.transform, normal);
        MathNormalize(&normal);
        manifold->normal = normal;
    }
    else // Closest to face
    {
        Vector2 normal = vertexData.normals[faceNormal];
        if (MathDot(Vector2Subtract(center, v1), normal) > A->shape.radius) return;
        
        normal = Mat2MultiplyVector2(vertexData.transform, normal);
        manifold->normal = (Vector2){ -normal.x, -normal.y };
        manifold->contacts[0] = (Vector2){ manifold->normal.x*A->shape.radius + A->position.x, manifold->normal.y*A->shape.radius + A->position.y };
        manifold->contactsCount = 1;
    }
}

// Solves collision between a polygon to a circle shape physics bodies
static void SolvePhysicsPolygonToCircle(PhysicsManifold manifold)
{
    PhysicsBody A = manifold->bodyA;
    PhysicsBody B = manifold->bodyB;
    
    manifold->bodyA = B;
    manifold->bodyB = A;
    SolvePhysicsCircleToPolygon(manifold);
    
    manifold->normal.x *= -1;
    manifold->normal.y *= -1;
}

// Solves collision between two polygons shape physics bodies
static void SolvePhysicsPolygonToPolygon(PhysicsManifold manifold)
{
    // TODO: implement it
}

// Integrates physics forces into velocity
static void IntegratePhysicsForces(PhysicsBody body)
{
    if (body->inverseMass != 0 && body->enabled)
    {
        body->velocity.x += (body->force.x*body->inverseMass)*(deltaTime/2);
        body->velocity.y += (body->force.y*body->inverseMass)*(deltaTime/2);
        
        if (body->useGravity)
        {
            body->velocity.x += gravityForce.x*(deltaTime/2);
            body->velocity.y += gravityForce.y*(deltaTime/2);
        }
        
        body->angularVelocity += body->torque*body->inverseInertia*(deltaTime/2);
    }
}

// Initializes physics manifolds to solve collisions
static void InitializePhysicsManifolds(PhysicsManifold manifold)
{
    PhysicsBody A = manifold->bodyA;
    PhysicsBody B = manifold->bodyB;
    
    // Calculate average restitution
    manifold->e = min(A->restitution, B->restitution);
    
    // Calculate static and dynamic friction
    manifold->sf = sqrt(A->staticFriction*B->staticFriction);
    manifold->df = sqrt(A->dynamicFriction*B->dynamicFriction);
    
    for (int i = 0; i < 2; i++)
    {
        // Caculate radius from center of mass to contact
        Vector2 ra = Vector2Subtract(manifold->contacts[i], A->position);
        Vector2 rb = Vector2Subtract(manifold->contacts[i], B->position);
        
        Vector2 crossB = MathCross(B->angularVelocity, rb);
        Vector2 crossA = MathCross(A->angularVelocity, ra);
        
        Vector2 rv = { 0 };
        rv.x = B->velocity.x + crossB.x - A->velocity.x - crossA.x;
        rv.y = B->velocity.y + crossB.y - A->velocity.y - crossA.y;
        
        // Determine if we should perform a resting collision or not;
        // The idea is if the only thing moving this object is gravity, then the collision should be performed without any restitution
        if (MathLenSqr(rv) < (MathLenSqr((Vector2){ gravityForce.x*deltaTime, gravityForce.y*deltaTime }) + MATH_EPSILON)) manifold->e = 0;
    }
}

// Integrates physics collisions impulses to solve collisions
static void IntegratePhysicsImpulses(PhysicsManifold manifold)
{
    PhysicsBody A = manifold->bodyA;
    PhysicsBody B = manifold->bodyB;
    
    // Early out and positional correct if both objects have infinite mass
    if (fabs(manifold->bodyA->inverseMass + manifold->bodyB->inverseMass) <= MATH_EPSILON)
    {
        A->velocity = (Vector2){ 0 };        
        B->velocity = (Vector2){ 0 };   
    }
    else
    {
        for (int i = 0; i < manifold->contactsCount; i++)
        {
            // Calculate radius from center of mass to contact
            Vector2 ra = Vector2Subtract(manifold->contacts[i], A->position);
            Vector2 rb = Vector2Subtract(manifold->contacts[i], B->position);
            
            // Calculate relative velocity
            Vector2 rv = { 0 };
            rv.x = B->velocity.x + MathCross(B->angularVelocity, rb).x - A->velocity.x - MathCross(A->angularVelocity, ra).x;
            rv.y = B->velocity.y + MathCross(B->angularVelocity, rb).y - A->velocity.y - MathCross(A->angularVelocity, ra).y;
            
            // Relative velocity along the normal
            float contactVelocity = MathDot(rv, manifold->normal);
            
            // Do not resolve if velocities are separating
            if (contactVelocity <= 0)
            {
                float raCrossN = MathCrossVector2(ra, manifold->normal);
                float rbCrossN = MathCrossVector2(rb, manifold->normal);
                
                float inverseMassSum = A->inverseMass + B->inverseMass + (raCrossN*raCrossN)*A->inverseInertia + (rbCrossN*rbCrossN)*B->inverseInertia;
                
                // Calculate impulse scalar value
                float j = -(1.0f + manifold->e)*contactVelocity;
                j /= inverseMassSum;                
                j /= (float)manifold->contactsCount;
                
                // Apply impulse to each physics body
                Vector2 impulse = { manifold->normal.x*j, manifold->normal.y*j };
                A->velocity.x += A->inverseMass*(-impulse.x);
                A->velocity.y += A->inverseMass*(-impulse.y);
                A->angularVelocity += A->inverseInertia*MathCrossVector2(ra, (Vector2){ -impulse.x, -impulse.y });
                
                B->velocity.x += B->inverseMass*(impulse.x);
                B->velocity.y += B->inverseMass*(impulse.y);
                B->angularVelocity += B->inverseInertia*MathCrossVector2(rb, impulse);
                
                // Apply friction impulse to each physics body
                rv.x = B->velocity.x + MathCross(B->angularVelocity, rb).x - A->velocity.x - MathCross(A->angularVelocity, ra).x;
                rv.y = B->velocity.y + MathCross(B->angularVelocity, rb).y - A->velocity.y - MathCross(A->angularVelocity, ra).y;
                
                Vector2 t = { rv.x - (manifold->normal.x*MathDot(rv, manifold->normal)), rv.y - (manifold->normal.y*MathDot(rv, manifold->normal)) };
                MathNormalize(&t);
                
                // Calculate 'j' tangent magnitude
                float jt = -MathDot(rv, t);
                jt /= inverseMassSum;
                jt /= (float)manifold->contactsCount;
                
                float absJt = fabs(jt);
                
                // Don't apply tiny friction impulses
                if (absJt > MATH_EPSILON)
                {
                    // Apply coulumb's law
                    Vector2 tangentImpulse;
                    if (absJt < j*manifold->sf) tangentImpulse = (Vector2){ t.x*jt, t.y*jt };
                    else tangentImpulse = (Vector2){ t.x*-j*manifold->df, t.y*-j*manifold->df };
                    
                    // Apply friction impulse
                    A->velocity.x += A->inverseMass*(-tangentImpulse.x);
                    A->velocity.y += A->inverseMass*(-tangentImpulse.y);
                    A->angularVelocity += A->inverseInertia*MathCrossVector2(ra, (Vector2){ -tangentImpulse.x, -tangentImpulse.y });
                    
                    B->velocity.x += B->inverseMass*(tangentImpulse.x);
                    B->velocity.y += B->inverseMass*(tangentImpulse.y);
                    B->angularVelocity += B->inverseInertia*MathCrossVector2(rb, tangentImpulse);
                }
            }
        }
    }
}

// Integrates physics velocity into position and forces
static void IntegratePhysicsVelocity(PhysicsBody body)
{
    if (body->inverseMass != 0 && body->enabled)
    {
        body->position.x += body->velocity.x*deltaTime;
        body->position.y += body->velocity.y*deltaTime;
        
        body->orient += body->angularVelocity*deltaTime;
        Mat2Set(&body->shape.vertexData.transform, body->orient);
        
        IntegratePhysicsForces(body);
    }
}

// Corrects physics bodies positions based on manifolds collision information
static void CorrectPhysicsPositions(PhysicsManifold manifold)
{
    Vector2 correction;
    correction.x = (max(manifold->penetration - PENETRATION_ALLOWANCE, 0)/(manifold->bodyA->inverseMass + manifold->bodyB->inverseMass))*manifold->normal.x*PENETRATION_CORRECTION;
    correction.y = (max(manifold->penetration - PENETRATION_ALLOWANCE, 0)/(manifold->bodyA->inverseMass + manifold->bodyB->inverseMass))*manifold->normal.y*PENETRATION_CORRECTION;
    
    if (manifold->bodyA->enabled)
    {
        manifold->bodyA->position.x -= correction.x*manifold->bodyA->inverseMass;
        manifold->bodyA->position.y -= correction.y*manifold->bodyA->inverseMass;
    }
    
    if (manifold->bodyB->enabled)
    {
        manifold->bodyB->position.x += correction.x*manifold->bodyB->inverseMass;
        manifold->bodyB->position.y += correction.y*manifold->bodyB->inverseMass;
    }
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

// Returns the cross product of a vector and a value
static Vector2 MathCross(float a, Vector2 v)
{
    return (Vector2){ -a*v.y, a*v.x };
}

// Returns the cross product of two vectors
static float MathCrossVector2(Vector2 a, Vector2 b)
{
    return (a.x*b.y - a.y*b.x);
}

// Returns the len square root of a vector
static float MathLenSqr(Vector2 v)
{
    return (v.x*v.x + v.y*v.y);
}

// Returns the dot product of two vectors
static float MathDot(Vector2 a, Vector2 b)
{
    return (a.x*b.x + a.y*b.y);
}

// Returns the normalized values of a vector
static void MathNormalize(Vector2 *v)
{
    float length, ilength;

    Vector2 vector = *v;
    length = sqrt(vector.x*vector.x + vector.y*vector.y);

    if (length == 0) length = 1.0f;

    ilength = 1.0f/length;

    v->x *= ilength;
    v->y *= ilength;
}

// Returns the sum of two given vectors
static Vector2 Vector2Add(Vector2 a, Vector2 b)
{
    return (Vector2){ a.x + b.x, a.y + b.y };
}

// Returns the subtract of two given vectors
static Vector2 Vector2Subtract(Vector2 a, Vector2 b)
{
    return (Vector2){ a.x - b.x, a.y - b.y };
}

// Creates a matrix 2x2 from a given radians value
static Mat2 Mat2Radians(float radians)
{
    Mat2 matrix = { 0 };
    
    float c = cos(radians);
    float s = sin(radians);
    
    matrix.m00 = c;
    matrix.m01 = -s;
    matrix.m10 = s;
    matrix.m11 = c;
    
    return matrix;
}

// Creates a matrix 2x2 from specific values
static Mat2 CreateMat2(float a, float b, float c, float d)
{
    Mat2 matrix = { 0 };
    
    matrix.m00 = a;
    matrix.m01 = b;
    matrix.m10 = c;
    matrix.m11 = d;
    
    return matrix;
}

// Set values from radians to a created matrix 2x2
static void Mat2Set(Mat2 *matrix, float radians)
{
    float c = cos(radians);
    float s = sin(radians);
    
    matrix->m00 = c;
    matrix->m01 = -s;
    matrix->m10 = s;
    matrix->m11 = c;
}

// Converts to absolute value every matrix values
static void Mat2Abs(Mat2 *matrix, float radians)
{
    matrix->m00 = fabs(matrix->m00);
    matrix->m01 = fabs(matrix->m01);
    matrix->m10 = fabs(matrix->m10);
    matrix->m11 = fabs(matrix->m11);
}

// Returns m00 and m10 as a Vector2 struct of a matrix 2x2
static Vector2 Mat2AxisX(Mat2 matrix)
{
    return (Vector2){ matrix.m00, matrix.m10 };
}

// Returns m01 and m11 as a Vector2 struct of a matrix 2x2
static Vector2 Mat2AxisY(Mat2 matrix)
{
    return (Vector2){ matrix.m01, matrix.m11 };
}

// Returns the transpose of a given matrix 2x2
static Mat2 Mat2Transpose(Mat2 m)
{
    Mat2 matrix = m;
    
    matrix.m01 = m.m10;
    matrix.m10 = m.m01;
    
    return matrix;
}

// Multiplies a vector by a matrix 2x2
static Vector2 Mat2MultiplyVector2(Mat2 m, Vector2 v)
{
    Vector2 vector = { 0 };
    
    vector.x = m.m00*v.x + m.m01*v.y;
    vector.y = m.m10*v.x + m.m11*v.y;
    
    return vector;
}

// Multiplies two given matrices 2x2
static Mat2 Mat2Multiply(Mat2 m1, Mat2 m2)
{
    Mat2 matrix = { 0 };
    
    matrix.m00 = m1.m00*m2.m00 + m1.m01*m2.m10;
    matrix.m00 = m1.m00*m2.m01 + m1.m01*m2.m11;
    matrix.m00 = m1.m10*m2.m00 + m1.m11*m2.m10;
    matrix.m00 = m1.m10*m2.m01 + m1.m11*m2.m11;
    
    return matrix;
}

#endif  // PHYSAC_IMPLEMENTATION
