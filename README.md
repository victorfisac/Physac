<img src="https://github.com/victorfisac/Physac/blob/master/icon/physac_256x256.png">

# Physac

Physac is a small 2D physics engine written in pure C. The engine uses a fixed time-step thread loop to simluate physics.
A physics step contains the following phases: get collision information, apply dynamics, collision solving and position correction. It uses a very simple struct for physic bodies with a position vector to be used in any 3D rendering API.

The header file includes some tweakable define values to fit the results that the user wants with a minimal bad results. Most of those values are commented with a little explanation about their uses.

_Note: The example code uses raylib programming library to create the program window and rendering framework._

Installation
-----

Physac requires raylib. To get it, follow the next steps:

    * Go to [raylib](https://www.github.com/raysan5/raylib) and clone the repository.
    * Ensure to pull the last changes of 'master' branch.
    * Use code inside examples header comments to compile and execute.

Physac API
-----

The PhysicsBody struct contains all dynamics information and collision shape. The user should use the following structure components:
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
    float staticFriction;           // Friction when the body has not movement (0 to 1)
    float dynamicFriction;          // Friction when the body has movement (0 to 1)
    float restitution;              // Restitution coefficient of the body (0 to 1)
    bool useGravity;                // Apply gravity force to dynamics
    bool isGrounded;                // Physics grounded on other body state
    bool freezeOrient;              // Physics rotation constraint
    PhysicsShape shape;             // Physics body shape information (type, radius, vertices, normals)
} *PhysicsBody;
```
The header contains a few customizable define values. I set the values that gived me the best results.

```c
#define     PHYSAC_MAX_BODIES               64
#define     PHYSAC_MAX_MANIFOLDS            4096
#define     PHYSAC_MAX_VERTICES             24
#define     PHYSAC_CIRCLE_VERTICES          24

#define     PHYSAC_COLLISION_ITERATIONS     100
#define     PHYSAC_PENETRATION_ALLOWANCE    0.05f
#define     PHYSAC_PENETRATION_CORRECTION   0.4f
```

Physac contains defines for memory management functions (malloc, free) to bring the user the opportunity to implement its own memory functions:

```c
#define     PHYSAC_MALLOC(size)             malloc(size)
#define     PHYSAC_FREE(ptr)                free(ptr)
```

The Physac API functions availables for the user are the following:

```c
// Initializes physics values, pointers and creates physics loop thread
void InitPhysics(void);

// Returns true if physics thread is currently enabled
bool IsPhysicsEnabled(void);

// Sets physics global gravity force
void SetPhysicsGravity(float x, float y);

// Creates a new circle physics body with generic parameters
PhysicsBody CreatePhysicsBodyCircle(Vector2 pos, float radius, float density);

// Creates a new rectangle physics body with generic parameters
PhysicsBody CreatePhysicsBodyRectangle(Vector2 pos, float width, float height, float density);

// Creates a new polygon physics body with generic parameters
PhysicsBody CreatePhysicsBodyPolygon(Vector2 pos, float radius, int sides, float density);

// Adds a force to a physics body
void PhysicsAddForce(PhysicsBody body, Vector2 force);

// Adds a angular force to a physics body
void PhysicsAddTorque(PhysicsBody body, float amount);

// Shatters a polygon shape physics body to little physics bodies with explosion force
void PhysicsShatter(PhysicsBody body, Vector2 position, float force);

// Returns the current amount of created physics bodies
int GetPhysicsBodiesCount(void);

// Returns a physics body of the bodies pool at a specific index
PhysicsBody GetPhysicsBody(int index);

// Returns the physics body shape type (PHYSICS_CIRCLE or PHYSICS_POLYGON)
int GetPhysicsShapeType(int index);

// Returns the amount of vertices of a physics body shape
int GetPhysicsShapeVerticesCount(int index);

// Returns transformed position of a body shape (body position + vertex transformed position)
Vector2 GetPhysicsShapeVertex(PhysicsBody body, int vertex);

// Sets physics body shape transform based on radians parameter
void SetPhysicsBodyRotation(PhysicsBody body, float radians);

// Unitializes and destroy a physics body
void DestroyPhysicsBody(PhysicsBody body);

// Unitializes physics pointers and closes physics loop thread
void ClosePhysics(void);
```
_Note: InitPhysics() needs to be called at program start and ClosePhysics() before the program ends. Closing and initializing Physac during the program flow doesn't affect or produces any error (useful as a 'reset' to destroy any created body by user in runtime)._

Dependencies
-----

Physac uses the following C libraries for memory management, math operations and some debug features:

   *  stdlib.h - Memory allocation [malloc(), free(), srand(), rand()].
   *  stdio.h  - Message logging (only if PHYSAC_DEBUG is defined) [printf()].
   *  math.h   - Math operations functions [cos(), sin(), fabs(), sqrtf()].

**It is independent to any graphics engine** and prepared to use any graphics API and use the vertices information (look at examples Drawing logic) to draw lines or shapes in screen. For example, this vertices information can be use in OpenGL API glVertex2f().

By the way, I use [raylib](http://www.raylib.com) to create the examples. This videogames programming library is used to handle inputs, window management and graphics drawing (using OpenGL API).
