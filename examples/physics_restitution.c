/*******************************************************************************************
*
*   Physac - Physics restitution
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

int main()
{
    // Initialization
    //--------------------------------------------------------------------------------------
    int screenWidth = 800;
    int screenHeight = 450;

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "[physac] - Restitution demo");

    // Physac logo drawing position
    int logoX = screenWidth - MeasureText("Physac", 30) - 10;
    int logoY = 15;

    // Initialize physics and default physics bodies
    InitPhysics();

    // Create floor rectangle physics body
    PhysicsBody floor = CreatePhysicsBodyRectangle((Vector2){ screenWidth/2, screenHeight }, screenWidth, 100, 10);
    floor->enabled = false; // Disable body state to convert it to static (no dynamics, but collisions)
    floor->restitution = 1;

    // Create circles physics body
    PhysicsBody circleA = CreatePhysicsBodyCircle((Vector2){ screenWidth*0.25f, screenHeight/2 }, 30, 10);
    circleA->restitution = 0.0f;
    PhysicsBody circleB = CreatePhysicsBodyCircle((Vector2){ screenWidth*0.5f, screenHeight/2 }, 30, 10);
    circleB->restitution = 0.5f;
    PhysicsBody circleC = CreatePhysicsBodyCircle((Vector2){ screenWidth*0.75f, screenHeight/2 }, 30, 10);
    circleC->restitution = 0.9f;
    
    SetTargetFPS(60);
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        // ...
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(BLACK);

            DrawFPS(screenWidth - 90, screenHeight - 30);

            // Draw created physics bodies
            int bodiesCount = GetPhysicsBodiesCount();
            for (int i = 0; i < bodiesCount; i++)
            {
                PhysicsBody body = GetPhysicsBody(i);

                int vertexCount = GetPhysicsShapeVerticesCount(i);
                for (int j = 0; j < vertexCount; j++)
                {
                    // Get physics bodies shape vertices to draw lines
                    // Note: GetPhysicsShapeVertex() already calculates rotation transformations
                    Vector2 vertexA = GetPhysicsShapeVertex(body, j);

                    int jj = (((j + 1) < vertexCount) ? (j + 1) : 0);   // Get next vertex or first to close the shape
                    Vector2 vertexB = GetPhysicsShapeVertex(body, jj);

                    DrawLineV(vertexA, vertexB, GREEN);     // Draw a line between two vertex positions
                }
            }

            DrawText("Restitution amount", (screenWidth - MeasureText("Restitution amount", 30))/2, 75, 30, WHITE);
            DrawText("0%", circleA->position.x - MeasureText("0%", 20)/2, circleA->position.y - 7, 20, WHITE);
            DrawText("50%", circleB->position.x - MeasureText("50%", 20)/2, circleB->position.y - 7, 20, WHITE);
            DrawText("90%", circleC->position.x - MeasureText("90%", 20)/2, circleC->position.y - 7, 20, WHITE);

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