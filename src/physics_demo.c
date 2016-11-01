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

#include "raylib.h"

#define PHYSAC_IMPLEMENTATION
#include "physac.h"

int main()
{
    // Initialization
    //--------------------------------------------------------------------------------------
    int screenWidth = 800;
    int screenHeight = 450;

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "Physac [raylib] - demo");
    SetTargetFPS(60);

    bool debug = false;
    bool closedPhysics = false;     // Used for manual physics closing and reinitialization

    // Physac logo drawing position
    int logoX = screenWidth - MeasureText("Physac", 30) - 10;
    int logoY = 15;

    // Initialize physics and default physics bodies
    InitPhysics((Vector2){ 0, 9.81f/1000 });

    // Create obstacle circle physics body
    PhysicsBody A = CreatePhysicsBodyCircle((Vector2){ screenWidth/2, screenHeight/2 }, 2, 45);
    A->enabled = false; // Disable body state to convert it to static (no dynamics, but collisions)

    // Create floor rectangle physics body
    PhysicsBody C = CreatePhysicsBodyRectangle((Vector2){ screenWidth/2, screenHeight }, (Vector2){ -500, -50 }, (Vector2){ 500, 75 }, 10);
    C->enabled = false; // Disable body state to convert it to static (no dynamics, but collisions)
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        if (!physicsThreadEnabled && closedPhysics)   // Wait for physics thread end to reinitialize it again to avoid exceptions
        {
            InitPhysics((Vector2){ 0, 9.81f/1000 });

            // Create obstacle circle physics body
            PhysicsBody A = CreatePhysicsBodyCircle((Vector2){ screenWidth/2, screenHeight/2 }, 2, 45);
            A->enabled = false;

            PhysicsBody C = CreatePhysicsBodyRectangle((Vector2){ screenWidth/2, screenHeight }, (Vector2){ -500, -50 }, (Vector2){ 500, 75 }, 10);
            C->enabled = false;

            closedPhysics = false;
        }

        // Destroy falling physics bodies
        for (int i = physicsBodiesCount - 1; i >= 0; i--)
        {
            PhysicsBody body = bodies[i];
            if (body->position.y > screenHeight*2) DestroyPhysicsBody(body);
        }

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) CreatePhysicsBodyPolygon(GetRandomValue(3, PHYSAC_MAX_VERTICES), GetRandomValue(20, 80), GetMousePosition(), 10);
        else if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) CreatePhysicsBodyCircle(GetMousePosition(), 2, GetRandomValue(10, 45));

        if (IsKeyPressed('R'))  // Reset physics input
        {
            ClosePhysics();          
            closedPhysics = true;
        }
        else if (IsKeyPressed('P')) debug = !debug;     // Debug state switch input
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(BLACK);

            DrawPhysicsBodies();    // Draws all created physics bodies shapes
            if (debug) DrawPhysicsContacts();

            DrawText("Left mouse button to create a polygon", 10, 10, 10, WHITE);
            DrawText("Right mouse button to create a circle", 10, 25, 10, WHITE);
            DrawText("Press 'P' to switch debug state\nPress 'R' to reset example", 10, 40, 10, WHITE);

            DrawText("Physac", logoX, logoY, 30, WHITE);
            DrawText("Powered by", logoX + 50, logoY - 7, 10, WHITE);

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