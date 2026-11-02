# Search and numeric model

The graph contains every unblocked grid cell. Edges connect orthogonal neighbours and, when
enabled, diagonal neighbours. The no-corner-cutting policy requires both side cells of each
diagonal edge to be unblocked. Edges are directed by their entered-cell cost.

For an orthogonal edge `u -> v`, `w(u,v) = 10 * weight(v)`; for a diagonal edge it is
`14 * weight(v)`. Positive integer weights ensure that an optimal path contains no cycles.
With at most 2^20 cells and weights below 2^32, a simple path costs less than
`2^20 * 14 * 2^32 < 2^56`. Manhattan/octile lower bounds and candidate queue estimates fit
below `2^57`. Invalid dimensions are rejected before multiplication. No distance sentinel can
be confused with a legal finite result.

Let `m` be the least positive grid weight. Manhattan distance times `10*m` is a consistent
orthogonal heuristic. For diagonal movement, use
`m * (14 * min(dx,dy) + 10 * (max(dx,dy)-min(dx,dy)))`.
Each permitted edge costs at least its geometric step factor times `m`. Restricting edges
because of obstacles/corner policy cannot invalidate this lower bound. Dijkstra sets it to zero.
These fixed consistent heuristics permit settling a cell once.

Queue entries store the distance and estimated total when inserted. A better relaxation inserts
a fresh record; old records are discarded if settled or their distance no longer matches the
best stored distance. The queue orders estimated total, then travelled cost, then cell index.
Parent indices are assigned only on strict improvement and the final chain is reversed to
produce a start-to-goal result. Ties are deterministic here, but their path shape is not part of
the package's compatibility contract.

The public expansion budget counts settled nodes, not stale queue entries, scanning or heap
operations. It bounds graph exploration rather than elapsed time. Returning a budget status
does not assert that the destination is unreachable.

For general A* terminology see the
[Boost.Graph A* documentation](https://www.boost.org/latest/libs/graph/doc/html/graph/algorithms/shortest_paths/astar_search.html).
This implementation does not copy or depend on Boost. Tests use a separately implemented dense
Bellman-Ford reference without a heap or heuristic.
