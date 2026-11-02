#include <benscr/pathfinding/grid.hpp>

#include <iostream>

namespace Pathfinding = BenScr::Pathfinding;

int main()
{
    Pathfinding::Grid grid(5, 3);
    grid.SetCellCost({1, 1}, 9);
    grid.SetCellCost({2, 1}, 9);
    grid.SetCellCost({3, 1}, 9);
    const auto result = Pathfinding::FindPath(grid, {0, 1}, {4, 1});
    const auto reference =
        Pathfinding::FindPath(grid, {0, 1}, {4, 1}, {Pathfinding::SearchAlgorithm::Dijkstra});
    if (result.Status != Pathfinding::SearchStatus::Found || result.Cost != 60 ||
        result.Cost != reference.Cost || result.Path.front() != Pathfinding::Point{0, 1} ||
        result.Path.back() != Pathfinding::Point{4, 1})
    {
        return 1;
    }
    const auto diagonal =
        Pathfinding::FindPath(Pathfinding::Grid(4, 3), {0, 0}, {3, 2},
                              {.Movement = Pathfinding::MovementPolicy::DiagonalNoCornerCutting});
    if (diagonal.Cost != 38)
    {
        return 2;
    }
    std::cout << "Shortest path verified.\n";
}
