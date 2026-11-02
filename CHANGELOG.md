# Changelog

## 1.0.0

- Introduce standalone C++20 weighted-grid A* and Dijkstra with owned results.
- Define exact integer costs, diagonal/corner policies, endpoint validation and search budgets.
- Replace the visualizer's mutable heap keys and incorrect estimated-cost update.
- Add reference, boundary, lifetime and concurrent-read tests plus isolated package consumers.
- Distribute native source with MSBuild integration and relocatable CMake installation.
