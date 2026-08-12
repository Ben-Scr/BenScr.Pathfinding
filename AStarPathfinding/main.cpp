#include <raylib.h>
#include "AstarAlgorithm.hpp"




int main() {
	InitWindow(1000, 1000, "A* Algorithm");
    SetTargetFPS(60);

    std::unique_ptr<AStarAlgorithm> astar = std::make_unique<AStarAlgorithm>();

    while (!WindowShouldClose())
    {
        if (GetKeyPressed() == KEY_R) {
            astar = std::make_unique<AStarAlgorithm>();
        }
           
        astar->Update();
        BeginDrawing();
        ClearBackground(Palette::Background);
        astar->Draw();
        EndDrawing();
    }

	CloseWindow();
	return 0;
}
