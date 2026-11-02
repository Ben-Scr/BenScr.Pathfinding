#include <benscr/pathfinding/grid.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace BenScr::Pathfinding
{
namespace Detail
{
std::size_t CheckSize(std::size_t width, std::size_t height)
{
    if (width == 0 || height == 0 || width > kMaxCellCount / height)
    {
        throw std::length_error(
            "Grid dimensions must be positive and contain at most 1048576 cells.");
    }
    return width * height;
}

void CheckOptions(const SearchOptions &options)
{
    if (options.Algorithm != SearchAlgorithm::AStar &&
        options.Algorithm != SearchAlgorithm::Dijkstra)
    {
        throw std::invalid_argument("Unknown search algorithm.");
    }
    if (options.Movement != MovementPolicy::Orthogonal &&
        options.Movement != MovementPolicy::DiagonalNoCornerCutting &&
        options.Movement != MovementPolicy::DiagonalAllowCornerCutting)
    {
        throw std::invalid_argument("Unknown movement policy.");
    }
    if (options.MaxExpandedNodes == 0 || options.MaxExpandedNodes > kMaxCellCount)
    {
        throw std::invalid_argument("The expanded-node limit must be in [1,1048576].");
    }
}

std::uint64_t Distance(Point from, Point to, bool diagonal, std::uint32_t minimumCost)
{
    const auto dx = from.X > to.X ? from.X - to.X : to.X - from.X;
    const auto dy = from.Y > to.Y ? from.Y - to.Y : to.Y - from.Y;
    const auto shared = diagonal ? std::min(dx, dy) : 0;
    return (static_cast<std::uint64_t>(dx + dy - 2 * shared) * kOrthogonalCost +
            static_cast<std::uint64_t>(shared) * kDiagonalCost) *
           minimumCost;
}

struct QueueNode
{
    std::uint64_t Estimate;
    std::uint64_t Cost;
    std::size_t Index;
};

struct QueueOrder
{
    bool operator()(const QueueNode &left, const QueueNode &right) const noexcept
    {
        if (left.Estimate != right.Estimate)
        {
            return left.Estimate > right.Estimate;
        }
        if (left.Cost != right.Cost)
        {
            return left.Cost > right.Cost;
        }
        return left.Index > right.Index;
    }
};
} // namespace Detail

Grid::Grid(std::size_t width, std::size_t height, std::uint32_t initialCost)
    : width(width), height(height), costs(Detail::CheckSize(width, height), initialCost)
{
}

Grid::Grid(Grid &&other) noexcept
    : width(std::exchange(other.width, 0)), height(std::exchange(other.height, 0)),
      costs(std::move(other.costs))
{
}

Grid &Grid::operator=(const Grid &other)
{
    if (this != &other)
    {
        Grid copy(other);
        *this = std::move(copy);
    }
    return *this;
}

Grid &Grid::operator=(Grid &&other) noexcept
{
    if (this != &other)
    {
        costs = std::move(other.costs);
        width = std::exchange(other.width, 0);
        height = std::exchange(other.height, 0);
    }
    return *this;
}

std::size_t Grid::GetWidth() const noexcept
{
    return width;
}
std::size_t Grid::GetHeight() const noexcept
{
    return height;
}
bool Grid::IsEmpty() const noexcept
{
    return width == 0;
}
bool Grid::Contains(Point point) const noexcept
{
    return point.X < width && point.Y < height;
}

std::uint32_t Grid::GetCellCost(Point point) const
{
    if (!Contains(point))
    {
        throw std::out_of_range("Cell coordinates are outside the grid.");
    }
    return costs[point.Y * width + point.X];
}

bool Grid::IsBlocked(Point point) const
{
    return GetCellCost(point) == 0;
}

void Grid::SetCellCost(Point point, std::uint32_t cost)
{
    if (!Contains(point))
    {
        throw std::out_of_range("Cell coordinates are outside the grid.");
    }
    costs[point.Y * width + point.X] = cost;
}

void Grid::Fill(std::uint32_t cost) noexcept
{
    if (!IsEmpty())
    {
        std::fill(costs.begin(), costs.end(), cost);
    }
}

SearchResult FindPath(const Grid &grid, Point start, Point goal, const SearchOptions &options)
{
    Detail::CheckOptions(options);
    if (!grid.Contains(start) || !grid.Contains(goal))
    {
        throw std::out_of_range("Search endpoints must be inside the grid.");
    }
    SearchResult result;
    if (grid.IsBlocked(start) || grid.IsBlocked(goal))
    {
        return result;
    }
    const auto width = grid.GetWidth();
    const auto height = grid.GetHeight();
    const auto count = width * height;
    const auto startIndex = start.Y * width + start.X;
    const auto goalIndex = goal.Y * width + goal.X;
    const bool diagonal = options.Movement != MovementPolicy::Orthogonal;
    std::uint32_t minimumCost = 0;
    if (options.Algorithm == SearchAlgorithm::AStar)
    {
        minimumCost = std::numeric_limits<std::uint32_t>::max();
        for (std::size_t y = 0; y < height; ++y)
        {
            for (std::size_t x = 0; x < width; ++x)
            {
                const auto cost = grid.GetCellCost({x, y});
                if (cost != 0)
                {
                    minimumCost = std::min(minimumCost, cost);
                }
            }
        }
    }
    constexpr auto kUnreached = std::numeric_limits<std::uint64_t>::max();
    std::vector<std::uint64_t> distances(count, kUnreached);
    std::vector<std::size_t> parents(count, count);
    std::vector<bool> closed(count, false);
    std::priority_queue<Detail::QueueNode, std::vector<Detail::QueueNode>, Detail::QueueOrder> open;
    distances[startIndex] = 0;
    open.push({Detail::Distance(start, goal, diagonal, minimumCost), 0, startIndex});
    constexpr std::array<std::array<int, 2>, 8> kDirections{
        {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}}};
    while (!open.empty())
    {
        const auto current = open.top();
        open.pop();
        if (closed[current.Index] || current.Cost != distances[current.Index])
        {
            continue;
        }
        if (result.ExpandedNodes == options.MaxExpandedNodes)
        {
            result.Status = SearchStatus::SearchLimitReached;
            return result;
        }
        closed[current.Index] = true;
        ++result.ExpandedNodes;
        if (current.Index == goalIndex)
        {
            result.Status = SearchStatus::Found;
            result.Cost = current.Cost;
            for (auto index = goalIndex; index != count; index = parents[index])
            {
                result.Path.push_back({index % width, index / width});
            }
            std::reverse(result.Path.begin(), result.Path.end());
            return result;
        }
        const Point point{current.Index % width, current.Index / width};
        const std::size_t directionCount = diagonal ? 8 : 4;
        for (std::size_t direction = 0; direction < directionCount; ++direction)
        {
            const auto dx = kDirections[direction][0];
            const auto dy = kDirections[direction][1];
            if ((dx < 0 && point.X == 0) || (dx > 0 && point.X + 1 == width) ||
                (dy < 0 && point.Y == 0) || (dy > 0 && point.Y + 1 == height))
            {
                continue;
            }
            const Point next{dx < 0 ? point.X - 1 : point.X + static_cast<std::size_t>(dx),
                             dy < 0 ? point.Y - 1 : point.Y + static_cast<std::size_t>(dy)};
            const auto weight = grid.GetCellCost(next);
            const auto nextIndex = next.Y * width + next.X;
            if (weight == 0 || closed[nextIndex])
            {
                continue;
            }
            const bool isDiagonal = dx != 0 && dy != 0;
            if (isDiagonal && options.Movement == MovementPolicy::DiagonalNoCornerCutting &&
                (grid.IsBlocked({next.X, point.Y}) || grid.IsBlocked({point.X, next.Y})))
            {
                continue;
            }
            // With at most kMaxCellCount cells and uint32 weights, all candidate costs
            // and heuristic estimates are below 2^57. No integer overflow is possible.
            const auto candidate =
                current.Cost + weight * (isDiagonal ? kDiagonalCost : kOrthogonalCost);
            if (candidate < distances[nextIndex])
            {
                distances[nextIndex] = candidate;
                parents[nextIndex] = current.Index;
                open.push({candidate + Detail::Distance(next, goal, diagonal, minimumCost),
                           candidate, nextIndex});
            }
        }
    }
    return result;
}
} // namespace BenScr::Pathfinding
