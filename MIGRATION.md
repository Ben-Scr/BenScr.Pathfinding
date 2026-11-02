# Migration from AStarPathfinding

Source: [Ben-Scr/AStarPathfinding](https://github.com/Ben-Scr/AStarPathfinding), master commit
`ca7504d4ae0ef6f75c9f0c0a9b86061b87dc8430`. The original repository is a Raylib visualization
application, not an existing NuGet library. Its public Git history and MIT license are retained.
No existing NuGet version is replaced by `BenScr.Pathfinding` 1.0.0.

The library separates the search model from the renderer. The original demo and its historical
project/build files remain in the checkout for provenance, but are not part of CMake, the native
package or library verification. They are not a supported application entry point for this release.
The active library uses only `include/`, `src/`, `tests/` and `examples/`.

| Original public surface | Replacement / disposition |
| --- | --- |
| `AStarAlgorithm()` | `Grid(width,height,cost)` plus explicit cell data and `FindPath`. Random obstacles and fixed endpoints are application responsibilities. |
| `AStarAlgorithm::NODE_ROW_AMOUNT` | Runtime `Grid::GetWidth/GetHeight`; maximum cell count is documented. |
| `AStarAlgorithm::NODE_PX_SIZE` | Renderer-only pixel sizing; intentionally omitted from the search library. |
| `AStarAlgorithm::OBSTACLES_AMOUNT` | Callers set zero-cost cells explicitly; no random obstacle generation. |
| `AStarAlgorithm::Update()` | Synchronous `FindPath` with an explicit expansion budget. The old per-frame stepping interface is intentionally not retained. |
| `AStarAlgorithm::Draw()` | Application-owned rendering of `Grid` and `SearchResult::Path`; no Raylib dependency. |
| `Palette::Background/Grid/Empty/Obstacle/Open/Closed/Path/Start/End` | Demo colors are presentation data and omitted from the library. |
| `Node::X/Y` | Value coordinates `Point::X/Y`. |
| `Node::IsObstacle` | `Grid::GetCellCost(point)==0` or `IsBlocked(point)`. |
| `Node::HCost/GCost/FCost` | Search-internal exact integer state; final cost is `SearchResult::Cost`. Mutable per-node diagnostic values are not exposed. |
| `Node::IsOpen/IsClosed` | Search-internal state; `ExpandedNodes` reports completed work. Visualization callbacks are not supplied. |
| `Node::Parent` | Internal indices; the result owns a forward ordered path instead of borrowing node pointers. |
| `OpenNode::m_Node` and `operator<` | Internal immutable queue records. The old public wrapper is removed. |
| Raylib `main()` and R-key reset | Application behavior outside the package. See the console consumer example for the new library. |

Changes are source-breaking: namespaces, types, construction and return values are explicit.
Old unit-cost steps correspond to new weight `1` with path cost scaled by `10`.
The old path was assembled goal-to-start; the new result is start-to-goal inclusive.

The old expansion wrote `FCost = GCost + FCost` instead of using `HCost`; its queue comparator
also read mutable node costs after insertion. The new queue stores immutable cost snapshots and
skips stale entries. Tests compare both algorithms with an independent Bellman-Ford implementation
and validate every returned path edge. Pointer lifetime and copy hazards from stored node pointers
are replaced by owned vectors and indices.
