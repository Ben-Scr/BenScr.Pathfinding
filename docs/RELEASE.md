# Release procedure

Run `pwsh -File tools/Verify.ps1` from a clean checkout. On Windows select the installed Visual
Studio generator and v143 toolset; on supported Linux run with `-Generator Ninja -Sanitizers`.
The verifier checks the reviewed header baseline, contract tests, independent-header test,
example, relocated CMake installation, native package payloads and Windows NuGet consumers.

`artifacts/latest-verification.txt` points to the exact verified output. Its release manifest
contains source SHA, package SHA-256 and executed checks. Require successful Windows, GCC and
Clang CI for the release commit, then check the exact downloaded CI artifact with the package
audit. Match the nuspec, manifest and immutable source tag
`packages/BenScr.Pathfinding/1.0.0`. Publish that specific `BenScr.Pathfinding.1.0.0.nupkg`.
Do not replace a published version or move an existing tag. A rebuilt archive needs verification.

After NuGet indexing:

```powershell
pwsh -File tools/VerifyNuGetConsumer.ps1 -Published -ExpectedPackage '/absolute/path/BenScr.Pathfinding.1.0.0.nupkg'
```

This consumer restores from nuget.org into a new directory, builds/runs Debug and Release x64
and compares payload bytes with the exact pre-publication artifact. NuGet signing may change
the enclosing ZIP hash; check content and repository signatures separately. Inspect the gallery
icon after indexing. The source package does not contain managed portable PDBs or a `.snupkg`.

Create a GitHub release at the same tag with the exact original package and final manifest.
Include package and CI URLs, actual verification states and platform limitations. Do not count
queued indexing, configured workflows or skipped platforms as successful publication checks.
