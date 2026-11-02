#include <benscr/pathfinding/grid.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <future>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace Pathfinding = BenScr::Pathfinding;
using Pathfinding::FindPath;
using Pathfinding::Grid;
using Pathfinding::MovementPolicy;
using Pathfinding::Point;
using Pathfinding::SearchAlgorithm;
using Pathfinding::SearchOptions;
using Pathfinding::SearchStatus;

void Check(bool condition, const char *message = "Contract failed.")
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

template <typename Exception, typename Action> void Throws(Action action)
{
    try
    {
        action();
    }
    catch (const Exception &)
    {
        return;
    }
    throw std::runtime_error("Expected exception was not thrown.");
}

// Independent dense Bellman-Ford reference: enumerate every possible directed edge,
// with no priority queue, heuristic, grid-neighbour helper or production search state.
std::uint64_t Reference(const Grid &grid, Point start, Point goal, MovementPolicy movement)
{
    constexpr auto kInfinity = std::numeric_limits<std::uint64_t>::max();
    if (grid.IsBlocked(start) || grid.IsBlocked(goal))
    {
        return kInfinity;
    }
    const auto count = grid.GetWidth() * grid.GetHeight();
    std::vector<std::uint64_t> distance(count, kInfinity);
    distance[start.Y * grid.GetWidth() + start.X] = 0;
    for (std::size_t pass = 1; pass < count; ++pass)
    {
        bool changed = false;
        for (std::size_t i = 0; i < count; ++i)
        {
            if (distance[i] == kInfinity)
            {
                continue;
            }
            const Point from{i % grid.GetWidth(), i / grid.GetWidth()};
            for (std::size_t j = 0; j < count; ++j)
            {
                const Point to{j % grid.GetWidth(), j / grid.GetWidth()};
                const auto dx = from.X > to.X ? from.X - to.X : to.X - from.X;
                const auto dy = from.Y > to.Y ? from.Y - to.Y : to.Y - from.Y;
                const bool diagonal = dx == 1 && dy == 1;
                const bool adjacent =
                    dx + dy == 1 || (diagonal && movement != MovementPolicy::Orthogonal);
                if (!adjacent || grid.IsBlocked(to))
                {
                    continue;
                }
                if (diagonal && movement == MovementPolicy::DiagonalNoCornerCutting &&
                    (grid.IsBlocked({from.X, to.Y}) || grid.IsBlocked({to.X, from.Y})))
                {
                    continue;
                }
                const auto candidate =
                    distance[i] +
                    static_cast<std::uint64_t>(grid.GetCellCost(to)) * (diagonal ? 14 : 10);
                if (candidate < distance[j])
                {
                    distance[j] = candidate;
                    changed = true;
                }
            }
        }
        if (!changed)
        {
            break;
        }
    }
    return distance[goal.Y * grid.GetWidth() + goal.X];
}

void CheckPath(const Grid &grid, Point start, Point goal, const Pathfinding::SearchResult &result,
               MovementPolicy movement)
{
    Check(result.Status == SearchStatus::Found);
    Check(!result.Path.empty() && result.Path.front() == start && result.Path.back() == goal);
    std::uint64_t total = 0;
    for (std::size_t i = 1; i < result.Path.size(); ++i)
    {
        const Point from = result.Path[i - 1];
        const Point to = result.Path[i];
        Check(grid.Contains(to) && !grid.IsBlocked(to));
        const auto dx = from.X > to.X ? from.X - to.X : to.X - from.X;
        const auto dy = from.Y > to.Y ? from.Y - to.Y : to.Y - from.Y;
        const bool diagonal = dx == 1 && dy == 1;
        Check(dx + dy == 1 || (diagonal && movement != MovementPolicy::Orthogonal));
        if (diagonal && movement == MovementPolicy::DiagonalNoCornerCutting)
        {
            Check(!grid.IsBlocked({from.X, to.Y}) && !grid.IsBlocked({to.X, from.Y}));
        }
        total += static_cast<std::uint64_t>(grid.GetCellCost(to)) * (diagonal ? 14 : 10);
    }
    Check(total == result.Cost, "Path cost differs from its traversed cells.");
    auto unique = result.Path;
    std::sort(unique.begin(), unique.end(),
              [](Point a, Point b) { return a.Y < b.Y || (a.Y == b.Y && a.X < b.X); });
    Check(std::adjacent_find(unique.begin(), unique.end()) == unique.end(),
          "Path contains a cycle.");
}

int main()
{
    const std::vector<std::pair<const char *, std::function<void()>>> cases{
        {"dimensions and initial costs",
         []
         {
             Grid grid(4, 3, 7);
             Check(grid.GetWidth() == 4 && grid.GetHeight() == 3 && !grid.IsEmpty());
             Check(grid.GetCellCost({3, 2}) == 7 && !grid.IsBlocked({3, 2}));
         }},
        {"zero width rejected", [] { Throws<std::length_error>([] { Grid grid(0, 1); }); }},
        {"zero height rejected", [] { Throws<std::length_error>([] { Grid grid(1, 0); }); }},
        {"multiplication overflow rejected",
         []
         {
             Throws<std::length_error>([]
                                       { Grid grid(std::numeric_limits<std::size_t>::max(), 2); });
             Throws<std::length_error>([]
                                       { Grid grid(2, std::numeric_limits<std::size_t>::max()); });
         }},
        {"maximum cell count enforced",
         []
         {
             Throws<std::length_error>([] { Grid grid(1024, 1025); });
             Grid grid(Pathfinding::kMaxCellCount, 1, 0);
             Check(grid.Contains({Pathfinding::kMaxCellCount - 1, 0}));
         }},
        {"contains boundaries",
         []
         {
             Grid grid(2, 3);
             Check(grid.Contains({0, 0}) && grid.Contains({1, 2}));
             Check(!grid.Contains({2, 2}) && !grid.Contains({1, 3}));
             Check(!grid.Contains({std::numeric_limits<std::size_t>::max(), 0}));
         }},
        {"invalid read",
         []
         {
             Grid grid(2, 2);
             Throws<std::out_of_range>([&] { (void)grid.GetCellCost({2, 0}); });
             Throws<std::out_of_range>([&] { (void)grid.IsBlocked({0, 2}); });
         }},
        {"invalid write preserves grid",
         []
         {
             Grid grid(2, 2, 8);
             Throws<std::out_of_range>([&] { grid.SetCellCost({2, 0}, 99); });
             Check(grid.GetCellCost({0, 1}) == 8);
         }},
        {"fill including blocked grid",
         []
         {
             Grid grid(3, 4, 0);
             Check(grid.IsBlocked({2, 3}));
             grid.Fill(9);
             Check(grid.GetCellCost({2, 3}) == 9);
             grid.SetCellCost({1, 2}, 0);
             Check(grid.IsBlocked({1, 2}));
         }},
        {"copy owns independent data",
         []
         {
             Grid original(3, 2, 5);
             Grid copy = original;
             copy.SetCellCost({1, 1}, 0);
             Check(original.GetCellCost({1, 1}) == 5);
             Grid assigned(1, 1);
             assigned = original;
             assigned.Fill(3);
             Check(original.GetCellCost({0, 0}) == 5);
         }},
        {"move and moved-from contracts",
         []
         {
             Grid original(3, 2, 5);
             Grid moved(std::move(original));
             Check(original.IsEmpty() && original.GetWidth() == 0 && original.GetHeight() == 0);
             original.Fill(1);
             Throws<std::out_of_range>([&] { (void)original.GetCellCost({0, 0}); });
             Grid target(1, 1);
             target = std::move(moved);
             Check(moved.IsEmpty() && target.GetCellCost({2, 1}) == 5);
             original = target;
             Check(!original.IsEmpty());
         }},
        {"straight path order and cost",
         []
         {
             Grid grid(5, 1);
             auto result = FindPath(grid, {0, 0}, {4, 0});
             Check(result.Cost == 40 && result.Path.size() == 5);
             CheckPath(grid, {0, 0}, {4, 0}, result, MovementPolicy::Orthogonal);
         }},
        {"start equals goal",
         []
         {
             auto result = FindPath(Grid(1, 1), {0, 0}, {0, 0}, {.MaxExpandedNodes = 1});
             Check(result.Status == SearchStatus::Found && result.Cost == 0);
             Check(result.Path == std::vector<Point>{{0, 0}} && result.ExpandedNodes == 1);
         }},
        {"blocked start",
         []
         {
             Grid grid(2, 2);
             grid.SetCellCost({0, 0}, 0);
             auto result = FindPath(grid, {0, 0}, {1, 1});
             Check(result.Status == SearchStatus::NoPath && result.ExpandedNodes == 0);
             Check(result.Path.empty() && result.Cost == 0);
         }},
        {"blocked goal and same blocked cell",
         []
         {
             Grid grid(2, 2, 0);
             grid.SetCellCost({0, 0}, 1);
             Check(FindPath(grid, {0, 0}, {1, 1}).Status == SearchStatus::NoPath);
             Check(FindPath(grid, {1, 1}, {1, 1}).Status == SearchStatus::NoPath);
         }},
        {"unreachable barrier",
         []
         {
             Grid grid(3, 3);
             for (std::size_t y = 0; y < 3; ++y)
             {
                 grid.SetCellCost({1, y}, 0);
             }
             auto result = FindPath(grid, {0, 1}, {2, 1});
             Check(result.Status == SearchStatus::NoPath && result.ExpandedNodes == 3);
             Check(result.Path.empty());
         }},
        {"weighted detour regression",
         []
         {
             Grid grid(5, 3);
             for (std::size_t x = 1; x < 4; ++x)
             {
                 grid.SetCellCost({x, 1}, 9);
             }
             auto result = FindPath(grid, {0, 1}, {4, 1});
             Check(result.Cost == 60);
             CheckPath(grid, {0, 1}, {4, 1}, result, MovementPolicy::Orthogonal);
             Check(FindPath(grid, {0, 1}, {4, 1}, {SearchAlgorithm::Dijkstra}).Cost == 60);
         }},
        {"diagonal octile reference",
         []
         {
             Grid grid(5, 4);
             SearchOptions options{.Movement = MovementPolicy::DiagonalNoCornerCutting};
             auto result = FindPath(grid, {0, 0}, {4, 3}, options);
             Check(result.Cost == 52 && result.Path.size() == 5);
             CheckPath(grid, {0, 0}, {4, 3}, result, options.Movement);
         }},
        {"corner crossing forbidden",
         []
         {
             Grid grid(2, 2);
             grid.SetCellCost({1, 0}, 0);
             grid.SetCellCost({0, 1}, 0);
             auto result = FindPath(grid, {0, 0}, {1, 1},
                                    {.Movement = MovementPolicy::DiagonalNoCornerCutting});
             Check(result.Status == SearchStatus::NoPath);
         }},
        {"corner crossing explicitly allowed",
         []
         {
             Grid grid(2, 2);
             grid.SetCellCost({1, 0}, 0);
             grid.SetCellCost({0, 1}, 0);
             auto result = FindPath(grid, {0, 0}, {1, 1},
                                    {.Movement = MovementPolicy::DiagonalAllowCornerCutting});
             Check(result.Status == SearchStatus::Found && result.Cost == 14);
         }},
        {"one blocked side prevents diagonal",
         []
         {
             Grid grid(2, 2);
             grid.SetCellCost({1, 0}, 0);
             auto result = FindPath(grid, {0, 0}, {1, 1},
                                    {.Movement = MovementPolicy::DiagonalNoCornerCutting});
             Check(result.Cost == 20 && result.Path.size() == 3);
         }},
        {"orthogonal default", [] { Check(FindPath(Grid(3, 3), {0, 0}, {2, 2}).Cost == 40); }},
        {"entered-cell costs are directed",
         []
         {
             Grid grid(2, 1);
             grid.SetCellCost({0, 0}, 100);
             Check(FindPath(grid, {0, 0}, {1, 0}).Cost == 10);
             Check(FindPath(grid, {1, 0}, {0, 0}).Cost == 1000);
         }},
        {"maximum weights fit uint64",
         []
         {
             constexpr auto kMaximum = std::numeric_limits<std::uint32_t>::max();
             Grid grid(4, 1, kMaximum);
             Check(FindPath(grid, {0, 0}, {3, 0}).Cost == 30ULL * kMaximum);
             Grid diagonal(2, 2, kMaximum);
             Check(FindPath(diagonal, {0, 0}, {1, 1},
                            {.Movement = MovementPolicy::DiagonalNoCornerCutting})
                       .Cost == 14ULL * kMaximum);
         }},
        {"invalid algorithm",
         []
         {
             Throws<std::invalid_argument>(
                 []
                 {
                     (void)FindPath(Grid(1, 1), {0, 0}, {0, 0}, {static_cast<SearchAlgorithm>(99)});
                 });
         }},
        {"invalid movement",
         []
         {
             Throws<std::invalid_argument>(
                 []
                 {
                     (void)FindPath(Grid(1, 1), {0, 0}, {0, 0},
                                    {.Movement = static_cast<MovementPolicy>(99)});
                 });
         }},
        {"invalid search limits",
         []
         {
             Throws<std::invalid_argument>(
                 [] { (void)FindPath(Grid(1, 1), {0, 0}, {0, 0}, {.MaxExpandedNodes = 0}); });
             Throws<std::invalid_argument>(
                 []
                 {
                     (void)FindPath(Grid(1, 1), {0, 0}, {0, 0},
                                    {.MaxExpandedNodes = std::numeric_limits<std::size_t>::max()});
                 });
         }},
        {"invalid endpoints and empty grid",
         []
         {
             Grid grid(2, 2);
             Throws<std::out_of_range>([&] { (void)FindPath(grid, {2, 0}, {0, 0}); });
             Throws<std::out_of_range>([&] { (void)FindPath(grid, {0, 0}, {0, 2}); });
             Grid other(std::move(grid));
             Throws<std::out_of_range>([&] { (void)FindPath(grid, {0, 0}, {0, 0}); });
         }},
        {"search budget boundary and reuse",
         []
         {
             Grid grid(5, 1);
             auto limited = FindPath(grid, {0, 0}, {4, 0}, {.MaxExpandedNodes = 4});
             Check(limited.Status == SearchStatus::SearchLimitReached &&
                   limited.ExpandedNodes == 4);
             Check(limited.Path.empty() && limited.Cost == 0);
             auto complete = FindPath(grid, {0, 0}, {4, 0}, {.MaxExpandedNodes = 5});
             Check(complete.Status == SearchStatus::Found && complete.ExpandedNodes == 5);
         }},
        {"mutation affects next search only",
         []
         {
             Grid grid(3, 1);
             const auto first = FindPath(grid, {0, 0}, {2, 0});
             grid.SetCellCost({1, 0}, 0);
             Check(FindPath(grid, {0, 0}, {2, 0}).Status == SearchStatus::NoPath);
             Check(first.Cost == 20 && first.Path.size() == 3);
         }},
        {"720 searches agree with independent reference",
         []
         {
             std::uint32_t state = 0x917AC12U;
             auto next = [&]
             {
                 state = state * 1664525U + 1013904223U;
                 return state;
             };
             for (int scenario = 0; scenario < 120; ++scenario)
             {
                 const std::size_t width = 1 + next() % 7;
                 const std::size_t height = 1 + next() % 7;
                 Grid grid(width, height);
                 for (std::size_t y = 0; y < height; ++y)
                 {
                     for (std::size_t x = 0; x < width; ++x)
                     {
                         grid.SetCellCost({x, y}, next() % 8);
                     }
                 }
                 const Point start{next() % width, next() % height};
                 const Point goal{next() % width, next() % height};
                 for (auto movement :
                      {MovementPolicy::Orthogonal, MovementPolicy::DiagonalNoCornerCutting,
                       MovementPolicy::DiagonalAllowCornerCutting})
                 {
                     const auto expected = Reference(grid, start, goal, movement);
                     for (auto algorithm : {SearchAlgorithm::AStar, SearchAlgorithm::Dijkstra})
                     {
                         const auto result = FindPath(grid, start, goal, {algorithm, movement});
                         if (expected == std::numeric_limits<std::uint64_t>::max())
                         {
                             Check(result.Status == SearchStatus::NoPath && result.Path.empty());
                         }
                         else
                         {
                             Check(result.Cost == expected,
                                   "Search cost differs from Bellman-Ford.");
                             CheckPath(grid, start, goal, result, movement);
                         }
                     }
                 }
             }
         }},
        {"concurrent independent searches", []
         {
             const Grid grid(16, 12);
             std::vector<std::future<Pathfinding::SearchResult>> tasks;
             for (int i = 0; i < 8; ++i)
             {
                 tasks.push_back(std::async(
                     std::launch::async,
                     [&]
                     {
                         return FindPath(grid, {0, 0}, {15, 11},
                                         {.Movement = MovementPolicy::DiagonalNoCornerCutting});
                     }));
             }
             for (auto &task : tasks)
             {
                 const auto result = task.get();
                 Check(result.Cost == 194);
                 CheckPath(grid, {0, 0}, {15, 11}, result, MovementPolicy::DiagonalNoCornerCutting);
             }
         }}};
    std::size_t passed = 0;
    for (const auto &[name, test] : cases)
    {
        try
        {
            test();
            ++passed;
        }
        catch (const std::exception &error)
        {
            std::cerr << "Failed: " << name << ": " << error.what() << '\n';
            return 1;
        }
    }
    std::cout << passed << " contract cases passed.\n";
}
