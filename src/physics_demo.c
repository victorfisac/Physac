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

#define     GRAVITY     (Vector2){ 0, 9.81f/100 }
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

    PhysicsBody A = CreatePhysicsBodyCircle((Vector2){ screenWidth/2, screenHeight/2 }, 2, 45);
    A->enabled = false;

    PhysicsBody C = CreatePhysicsBodyRectangle((Vector2){ screenWidth/2, screenHeight - 50 }, (Vector2){ -500, -50 }, (Vector2){ 500, 50 }, 10);
    C->enabled = false;
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------        
        if (IsMouseButtonPressed(0)) CreatePhysicsBodyPolygon(5, GetMousePosition(), 10);
        else if (IsMouseButtonPressed(1)) CreatePhysicsBodyCircle(GetMousePosition(), 2, GetRandomValue(20, 60));

        if (IsKeyPressed('F')) frameStepping = !frameStepping;
        if (IsKeyDown(' ')) canStep = true;
        if (IsKeyPressed('R'))
        {
            ClosePhysics();
            InitPhysics(GRAVITY);

            A = CreatePhysicsBodyCircle((Vector2){ screenWidth/2, screenHeight/2 }, 2, 45);
            A->enabled = false;

            C = CreatePhysicsBodyRectangle((Vector2){ screenWidth/2, screenHeight - 50 }, (Vector2){ -500, -50 }, (Vector2){ 500, 50 }, 10);
            C->enabled = false;
        }
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