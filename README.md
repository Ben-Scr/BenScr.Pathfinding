# BenScr.Pathfinding

C++20 weighted grid A* and Dijkstra with exact integer path costs, configurable diagonal movement,
owned path results and bounded search. There are no runtime library dependencies.

Created by Ben Schneider — https://github.com/Ben-Scr

```cpp
#include <benscr/pathfinding/grid.hpp>
namespace Pathfinding = BenScr::Pathfinding;

Pathfinding::Grid grid(20, 12); // traversable cells with weight 1
grid.SetCellCost({5, 5}, 0);    // obstacle
grid.SetCellCost({6, 5}, 4);    // more expensive terrain
const auto result = Pathfinding::FindPath(grid, {0, 0}, {19, 11},
    {.Movement = Pathfinding::MovementPolicy::DiagonalNoCornerCutting});
if (result.Status == Pathfinding::SearchStatus::Found)
{
    // result.Path owns the points in start-to-goal order; result.Cost is exact.
}
```

## Cost and search contract

- Grid dimensions are positive; their product is at most 1,048,576 cells. Sizes are checked before
  multiplication/allocation. An invalid size throws `std::length_error`.
- Cell weights use `uint32_t`: `0` is blocked, `1..UINT32_MAX` is traversable. Convert and validate
  signed or floating-point application data before calling these unsigned APIs.
- A step costs the **destination cell's weight** times `10` orthogonally or `14` diagonally. The
  starting cell costs nothing. `14/10` is this library's explicit diagonal cost model, not an exact
  square root of two. Reverse searches may have different costs because weights are paid on entry.
- A* uses Manhattan distance (orthogonal) or octile distance (diagonal), multiplied by the minimum
  traversable cell weight. These heuristics are consistent with the defined movement costs.
  Dijkstra uses zero heuristic. Both return an optimal path for this cost model when `Found`.
- `Orthogonal` is the default. `DiagonalNoCornerCutting` requires both adjacent orthogonal cells
  to be traversable; `DiagonalAllowCornerCutting` only checks the diagonal destination.
- Endpoints outside the grid throw `std::out_of_range`. Blocked endpoints return `NoPath` and zero
  expanded nodes. Unblocked `start == goal` returns one point, zero cost and one expanded node.
- Search limits count settled nodes, including start and goal. The valid inclusive limit is
  `1..kMaxCellCount`. Exhaustion returns `SearchLimitReached`, empty path and zero cost; this does
  not prove that no path exists. `NoPath` means the frontier was exhausted or an endpoint blocked.
- Unknown enum values and invalid search limits throw `std::invalid_argument`. No result contains
  a partial path. Equal-cost path shape is repeatable for this implementation but is not a
  cross-version compatibility guarantee.
- `Grid` owns its storage; copies are independent. A moved-from grid is empty, can be assigned a
  new grid, and rejects all cell coordinates. Copy assignment has the strong exception guarantee.
  `SearchResult` owns its path and survives later grid changes.
- Searches do not modify the grid. Concurrent const calls on an unchanged grid are supported.
  Mutation/assignment requires external synchronization. The library starts no threads.
- Each search allocates distance, parent and visited arrays plus a priority queue. A* additionally
  scans the grid for its minimum cell weight. This is bounded memory/work input, not a timing or
  allocation-free promise. Allocation failures propagate as `std::bad_alloc`.

All intermediate path costs and heuristic estimates are below `2^57` under these bounds, so
`uint64_t` arithmetic is exact. No `float`/`double` weights, custom heuristics, graph callbacks,
incremental visualization or navigation meshes are part of this API.

## Installation

In a Visual Studio C++ project using `packages.config`:

```powershell
Install-Package BenScr.Pathfinding -Version 1.0.0
```

Enable C++20. The package supplies `build/native/BenScr.Pathfinding.props` and
`BenScr.Pathfinding.targets`, adds the include directory and compiles `grid.cpp` in the consumer's
project. Your architecture, CRT and Debug/Release settings therefore apply to the source. Do not
compile the packaged `.cpp` a second time. This is a native source package, not a managed assembly;
there is no precompiled `.lib` or managed `.snupkg`.

CMake consumers build and install the source checkout:

```sh
cmake -S . -B out/build -DCMAKE_BUILD_TYPE=Release
cmake --build out/build --config Release
cmake --install out/build --config Release --prefix /your/prefix
```

Then use the installed, relocatable package:

```cmake
find_package(BenScrPathfinding 1 CONFIG REQUIRED)
target_link_libraries(MyApp PRIVATE BenScr::Pathfinding)
```

## Verification and support

Requires CMake 3.28+, C++20, PowerShell 7 and .NET SDK 10.0.303 for packaging. Windows verification
requires Visual Studio v143 C++ tools and a Windows SDK. The library itself does not require .NET.
Verification obtains checksum-pinned NuGet.CommandLine 7.9.0.

```powershell
pwsh -File tools/Verify.ps1 -Generator 'Visual Studio 17 2022' -Toolset v143
# After NuGet indexing, test the public native package:
pwsh -File tools/VerifyNuGetConsumer.ps1 -Published
```

The suite contains 32 contract cases, including 720 searches checked against an independent dense
Bellman-Ford reference, path legality/cost checks, boundaries, ownership and concurrent reads.
Verification also runs the standalone-header test and example, moves a CMake installation before
building a consumer, audits exact package bytes, and runs native NuGet consumers in Debug/Release
on Windows x64. CI covers MSVC v143, GCC 14 and Clang 18; Linux uses ASan/UBSan. macOS, ARM, x86,
ThreadSanitizer and MSVC sanitizers are not verified support claims. Tests/examples default off
when used as a CMake subproject.

See [migration](https://github.com/Ben-Scr/BenScr.Pathfinding/blob/master/MIGRATION.md),
[algorithm](https://github.com/Ben-Scr/BenScr.Pathfinding/blob/master/docs/ALGORITHM.md),
[provenance](https://github.com/Ben-Scr/BenScr.Pathfinding/blob/master/docs/PROVENANCE.md) and
[release procedure](https://github.com/Ben-Scr/BenScr.Pathfinding/blob/master/docs/RELEASE.md).
The original MIT license is preserved in `LICENSE` and its byte-identical package copy `LICENSE.txt`.
