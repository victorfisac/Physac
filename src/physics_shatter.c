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

    bool closedPhysics = false;     // Used for manual physics closing and reinitialization

    // Physac logo drawing position
    int logoX = screenWidth - MeasureText("Physac", 30) - 10;
    int logoY = 15;

    // Initialize physics and default physics bodies
    InitPhysics((Vector2){ 0, 0 });

    PhysicsBody A = CreatePhysicsBodyPolygon(GetRandomValue(3, PHYSAC_MAX_VERTICES), GetRandomValue(80, 200), (Vector2){ screenWidth/2, screenHeight/2 }, 10);
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        if (!physicsThreadEnabled && closedPhysics)   // Wait for physics thread end to reinitialize it again to avoid exceptions
        {
            InitPhysics((Vector2){ 0, 0 });

            // Create obstacle circle physics body
            A = CreatePhysicsBodyPolygon(GetRandomValue(3, PHYSAC_MAX_VERTICES), GetRandomValue(80, 200), (Vector2){ screenWidth/2, screenHeight/2 }, 10);

            closedPhysics = false;
        }

        if (IsKeyPressed('R'))  // Reset physics input
        {
            ClosePhysics();          
            closedPhysics = true;
        }

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON))
        {
            for (int i = physicsBodiesCount; i >= 0; i--) PhysicsShatter(bodies[i], GetMousePosition(), 1000000);
        }
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(BLACK);

            DrawPhysicsBodies();    // Draws all created physics bodies shapes
            
            DrawText("Left mouse button in polygon area to shatter body\nPress 'R' to reset example", 10, 10, 10, WHITE);

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