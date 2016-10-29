# Physac
_Created by Víctor Fisac [www.victorfisac.com]_

Physac is a small 2D physics engine written in pure C. The engine uses a fixed time-step thread loop to simluate physics.
A physics step contains the following phases: get collision information, apply dynamics, collision solving and position correction. It uses
a very simple struct for physic bodies with a position vector to be used in any 3D rendering API.

The header file includes some tweakable define values to fit the results that the user wants with a minimal bad results. Most of those values are commented with a little explanation about their uses.

_Note: The example code uses raylib programming library to create the program window and rendering framework._

Physac API
-----

The PhysicsBody struct contains all dynamics information and collision shape. It has the following structure:
```c
typedef struct *PhysicsBody {
    unsigned int id;
    bool enabled;                   // Enabled dynamics state (collisions are calculated anyway)
    Vector2 position;               // Physics body shape pivot
    Vector2 velocity;               // Current linear velocity applied to position
    Vector2 force;                  // Current linear force (reset to 0 every step)
    float angularVelocity;          // Current angular velocity applied to orient
    float torque;                   // Current angular force (reset to 0 every step)
    float orient;                   // Rotation in radians
    float inertia;                  // Moment of inertia
    float inverseInertia;           // Inverse value of inertia
    float mass;                     // Physics body mass
    float inverseMass;              // Inverse value of mass
    float staticFriction;           // Friction when the body has not movement (0 to 1)
    float dynamicFriction;          // Friction when the body has movement (0 to 1)
    float restitution;              // Restitution coefficient of the body (0 to 1)
    bool useGravity;                // Apply gravity force to dynamics
    PhysicsShape shape;             // Physics body shape information (type, radius, vertices, normals)
} *PhysicsBody;
```
By the way, the header contains a few customizable define values. I set the values that gived me the best results.

```c
#define     DESIRED_DELTATIME               1.0/60.0
#define     MAX_TIMESTEP                    0.02
#define     COLLISION_ITERATIONS            100
#define     PENETRATION_ALLOWANCE           0.05f
#define     PENETRATION_CORRECTION          0.4f
```

These are the Physac API functions usables by the user in its program:

```c
// Initializes physics values, pointers and creates physics loop thread
void InitPhysics(Vector2 gravity);

// Creates a new circle physics body with generic parameters
PhysicsBody CreatePhysicsBodyCircle(Vector2 pos, float density, float radius);

// Creates a new rectangle physics body with generic parameters
PhysicsBody CreatePhysicsBodyRectangle(Vector2 pos, Vector2 min, Vector2 max, float density);

// Creates a new polygon physics body with generic parameters
PhysicsBody CreatePhysicsBodyPolygon(int vertex, Vector2 pos, float density);

// Adds a force to a physics body
void PhysicsAddForce(PhysicsBody body, Vector2 f);

// Adds a angular force to a physics body
void PhysicsAddTorque(PhysicsBody body, float amount);

// Draws all created physics bodies shapes
void DrawPhysicsBodies(void);

// Draws all calculated physics contacts points and its normals
void DrawPhysicsContacts(void);

// Unitializes and destroy a physics body
void DestroyPhysicsBody(PhysicsBody body);

// Unitializes physics pointers and closes physics loop thread
void ClosePhysics(void);
```
_Note: InitPhysics() needs to be called at program start and ClosePhysics() before the program ends. Closing and initializing Physac during the program flow doesn't affect or produces any error (useful as a 'reset' to destroy any created body by user in runtime)._

Dependencies
-----

The current main dependency of Physac is [raylib](http://www.raylib.com). This videogames programming library is used to handle inputs, window management and graphics drawing (using OpenGL API). I would like to remove this dependency soon and implement my own functions to make Physac more independent.

Besides, Physac uses the following C libraries:

   *  stdlib.h - Memory allocation [malloc(), free()].
   *  stdio.h  - Message logging (only if PHYSAC_DEBUG is defined) [printf()].
   *  math.h   - Math operations functions [cos(), sin(), fabs(), sqrt()].
