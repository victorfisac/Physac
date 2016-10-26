/*******************************************************************************************
*
*   Physac - Hello World
*
*   NOTE: Physac requires multi-threading, when InitPhysics() a second thread is created to manage physics calculations.
*   The file pthreadGC2.dll is required to run the program; you can find it in 'src\external'
*
*   Copyright (c) 2016 Victor Fisac
*
********************************************************************************************/

#define PHYSAC_IMPLEMENTATION
#include "physac.h"
#include "stdio.h"

#define     GRAVITY     (Vector2){ 0, 0 }
#define     FORCE       10000
#define     TORQUE      10000

int main()
{
    // Initialization
    //--------------------------------------------------------------------------------------
    int screenWidth = 800;
    int screenHeight = 450;

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "Physac [raylib] - demo");
    SetTargetFPS(60);

    InitPhysics(GRAVITY);

    PhysicsBody A = CreatePhysicsBodyCircle((Vector2){ screenWidth/2, screenHeight/2 }, 2, 30);
    PhysicsBody B = CreatePhysicsBodyCircle((Vector2){ screenWidth/2 + 100, screenHeight/2 - 70 }, 2, 30);
    PhysicsBody C = CreatePhysicsBodyPolygon((Vector2){ screenWidth/2 + 250, screenHeight/2 - 100 }, 10);
    PhysicsBody D = CreatePhysicsBodyPolygon((Vector2){ screenWidth/2 + 250, screenHeight/2 + 100 }, 10);
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------        
        if (IsKeyPressed('F')) frameStepping = !frameStepping;
        if (IsKeyDown(' ')) canStep = true;
        if (IsKeyPressed('R'))
        {
            ClosePhysics();
            InitPhysics(GRAVITY);

            C = CreatePhysicsBodyPolygon((Vector2){ screenWidth/2 + 250, screenHeight/2 - 100 }, 10);
            D = CreatePhysicsBodyPolygon((Vector2){ screenWidth/2 + 250, screenHeight/2 + 100 }, 10);
        }

        if (IsKeyDown('D')) C->force.x += FORCE;
        else if (IsKeyDown('A')) C->force.x -= FORCE;

        if (IsKeyDown('S')) C->force.y += FORCE;
        else if (IsKeyDown('W')) C->force.y -= FORCE;

        if (IsKeyDown('T')) C->torque += TORQUE;
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(BLACK);

            DrawPhysicsBodies();
            DrawPhysicsContacts();

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------   
    ClosePhysics();       // Unitialize physics
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}