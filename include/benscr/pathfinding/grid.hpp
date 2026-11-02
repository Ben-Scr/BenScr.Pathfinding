#ifndef BENSCR_PATHFINDING_GRID_HPP
#define BENSCR_PATHFINDING_GRID_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace BenScr::Pathfinding
{
inline constexpr std::size_t kMaxCellCount = 1'048'576;
inline constexpr std::uint64_t kOrthogonalCost = 10;
inline constexpr std::uint64_t kDiagonalCost = 14;

/// Zero-based grid coordinates. Negative coordinates are outside the unsigned domain.
struct Point
{
    std::size_t X = 0;
    std::size_t Y = 0;
    bool operator==(const Point &) const = default;
};

/// Owns a rectangular row-major grid. A zero cell cost blocks the cell; otherwise the
/// cost is a multiplier in [1, UINT32_MAX]. Copies are independent. Concurrent const
/// calls are safe unless the grid is modified or assigned. Mutation needs synchronization.
class Grid
{
  public:
    /// Dimensions must be positive and their product <=kMaxCellCount, otherwise
    /// length_error is thrown before allocation. The initial cost may be zero.
    Grid(std::size_t width, std::size_t height, std::uint32_t initialCost = 1);
    Grid(const Grid &) = default;
    /// Copy assignment preserves the destination if allocation fails.
    Grid &operator=(const Grid &other);
    /// A moved-from grid is empty and may be assigned a new grid.
    Grid(Grid &&other) noexcept;
    Grid &operator=(Grid &&other) noexcept;

    [[nodiscard]] std::size_t GetWidth() const noexcept;
    [[nodiscard]] std::size_t GetHeight() const noexcept;
    [[nodiscard]] bool IsEmpty() const noexcept;
    [[nodiscard]] bool Contains(Point point) const noexcept;
    /// Throws out_of_range if the point is outside this grid, including a moved-from grid.
    [[nodiscard]] std::uint32_t GetCellCost(Point point) const;
    [[nodiscard]] bool IsBlocked(Point point) const;
    /// Sets one cell after validation. An invalid point leaves the grid unchanged.
    void SetCellCost(Point point, std::uint32_t cost);
    /// Sets all cells; on a moved-from grid this is a no-op.
    void Fill(std::uint32_t cost) noexcept;

  private:
    std::size_t width;
    std::size_t height;
    std::vector<std::uint32_t> costs;
};

enum class SearchAlgorithm
{
    AStar,
    Dijkstra
};

enum class MovementPolicy
{
    Orthogonal,
    /// A diagonal requires both adjacent orthogonal cells to be unblocked.
    DiagonalNoCornerCutting,
    /// A diagonal only requires its destination to be unblocked.
    DiagonalAllowCornerCutting
};

struct SearchOptions
{
    SearchAlgorithm Algorithm = SearchAlgorithm::AStar;
    MovementPolicy Movement = MovementPolicy::Orthogonal;
    /// Inclusive limit on settled nodes, including start and goal: [1,kMaxCellCount].
    std::size_t MaxExpandedNodes = kMaxCellCount;
};

enum class SearchStatus
{
    Found,
    NoPath,
    SearchLimitReached
};

struct SearchResult
{
    SearchStatus Status = SearchStatus::NoPath;
    /// Start-to-goal inclusive when Found; empty for other statuses. Owns its points.
    std::vector<Point> Path;
    /// Sum of entered-cell cost * 10 (orthogonal) or * 14 (diagonal). Start costs zero.
    /// Zero for non-Found results; Found start==goal has one point and cost zero.
    std::uint64_t Cost = 0;
    std::size_t ExpandedNodes = 0;
};

/// Finds an optimal path for the specified integer cost model. A* uses Manhattan/octile
/// distance times the cheapest unblocked cell; Dijkstra uses zero heuristic.
/// Out-of-grid endpoints throw out_of_range. Invalid enum values/limits throw
/// invalid_argument. Blocked endpoints produce NoPath with no expanded nodes.
/// Allocates O(cell count) working storage plus a priority queue; bad_alloc propagates.
/// Does not change the grid. Independent calls on an unchanged grid can run concurrently.
/// Equal-cost path shape is deterministic for this implementation, not a versioned promise.
[[nodiscard]] SearchResult FindPath(const Grid &grid, Point start, Point goal,
                                    const SearchOptions &options = {});
} // namespace BenScr::Pathfinding
#endif
