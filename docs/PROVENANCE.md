# Provenance

Created by Ben Schneider — https://github.com/Ben-Scr

`BenScr.Pathfinding` continues the pathfinding domain of the public Raylib demonstration
[AStarPathfinding](https://github.com/Ben-Scr/AStarPathfinding) at master commit
`ca7504d4ae0ef6f75c9f0c0a9b86061b87dc8430`. The source is owned by Ben-Scr and was publicly
available when inventoried on 2026-11-02. Its Git history is retained; the upstream repository,
branch and existing releases are unchanged. No pre-existing NuGet package was identified in
the source project, which used the vcpkg project version `0.1.0` and Raylib.

The original MIT license, Copyright (c) 2026 Ben Schneider, remains unchanged in `LICENSE`.
`LICENSE.txt` is its byte-identical package copy. The library's grid ownership, explicit integer
cost model, bounded search and independent tests replace the demo's coupled renderer/search state.
The migration table records every original public symbol. The active library has no Raylib,
vcpkg, Boost or other runtime package dependency. Old Raylib/vcpkg files are historical source
context outside the new build and distribution.

The original shared brand asset is embedded unchanged as `icon.png`. Repository path:
`branding/BenScr_Icon_Master.png`; PNG RGBA, 1254x1254 pixels, 795819 bytes;
SHA-256 `a5dcabceb766e13c1840a95130ee98a458e12e3713fd9b07f3ceae75a1f5b993`.
No raster transformation or alternative logo is used. The package audit verifies the icon bytes.

Release build tooling is maintained with the BenScr family. .NET SDK 10.0.303 is used only to
pack native files; checksum-pinned NuGet.CommandLine 7.9.0 is used only for the Windows consumer.
The C++ implementation and tests require the standard library only.
