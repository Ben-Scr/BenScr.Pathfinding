#include <benscr/pathfinding/grid.hpp>

int main()
{
    BenScr::Pathfinding::Grid grid(1, 1);
    return BenScr::Pathfinding::FindPath(grid, {0, 0}, {0, 0}).Path.size() == 1 ? 0 : 1;
}
