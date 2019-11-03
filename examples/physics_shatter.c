/*******************************************************************************************
*
*   Physac - Body shatter
*
*   NOTE 1: Physac requires multi-threading, when InitPhysics() a second thread 
*           is created to manage physics calculations.
*   NOTE 2: Physac requires static C library linkage to avoid dependency 
*           on MinGW DLL (-static -lpthread)
*
*   Compile this program using:
*       gcc -o $(NAME_PART).exe $(FILE_NAME) -s ..\icon\physac_icon -I. -I../src 
*           -I../src/external/raylib/src -static -lraylib -lopengl32 -lgdi32 -pthread -std=c99
*   
*   Copyright (c) 2016-2020 Victor Fisac (github: @victorfisac)
*
********************************************************************************************/

#include "raylib.h"

#define PHYSAC_IMPLEMENTATION
#include "physac.h"

#define SHATTER_FORCE 200.0f

int main()
{
    // Initialization
    //--------------------------------------------------------------------------------------
    int screenWidth = 800;
    int screenHeight = 450;

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "[physac] - Shatter demo");

    // Physac logo drawing position
    int logoX = screenWidth - MeasureText("Physac", 30) - 10;
    int logoY = 15;
    bool shatter = false;

    // Initialize physics and default physics bodies
    InitPhysics();
    SetPhysicsGravity(0, 0);

    // Create random polygon physics body to shatter
    CreatePhysicsBodyPolygon((Vector2){ screenWidth/2, screenHeight/2 }, GetRandomValue(80, 200), GetRandomValue(3, 8), 10);
    
    SetTargetFPS(60);
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        if (!shatter && IsMouseButtonPressed(MOUSE_LEFT_BUTTON))
        {
            shatter = true;
            
            // Note: some values need to be stored in variables due to asynchronous changes during main thread
            int count = GetPhysicsBodiesCount();
            for (int i = count - 1; i >= 0; i--)
            {
                PhysicsBody currentBody = GetPhysicsBody(i);
                
                if (currentBody != NULL)
                    PhysicsShatter(currentBody, GetMousePosition(), SHATTER_FORCE);
            }
        }
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(BLACK);

            // Draw created physics bodies
            int bodiesCount = GetPhysicsBodiesCount();
            for (int i = 0; i < bodiesCount; i++)
            {
                PhysicsBody currentBody = GetPhysicsBody(i);

                int vertexCount = GetPhysicsShapeVerticesCount(i);
                for (int j = 0; j < vertexCount; j++)
                {
                    // Get physics bodies shape vertices to draw lines
                    // Note: GetPhysicsShapeVertex() already calculates rotation transformations
                    Vector2 vertexA = GetPhysicsShapeVertex(currentBody, j);

                    int jj = (((j + 1) < vertexCount) ? (j + 1) : 0);   // Get next vertex or first to close the shape
                    Vector2 vertexB = GetPhysicsShapeVertex(currentBody, jj);

                    DrawLineV(vertexA, vertexB, GREEN);     // Draw a line between two vertex positions
                }
            }

            DrawText("Left mouse button in polygon area to shatter body", 10, 10, 10, WHITE);

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