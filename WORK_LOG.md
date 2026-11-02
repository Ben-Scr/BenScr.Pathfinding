# Verification record

The public AStarPathfinding source, master commit
`ca7504d4ae0ef6f75c9f0c0a9b86061b87dc8430`, was inventoried before migration. Its MIT license
and history are retained. The new package has no runtime dependencies. The original demo
and existing project outputs are excluded from the new build and NuGet package.

Implementation and test contracts are described in README, MIGRATION.md and docs/ALGORITHM.md.
The reviewed public header fingerprint is in docs/public-header-baseline.json. The shared icon
is unchanged. Verification results are recorded in each exact artifact's release-manifest.json;
remote CI and public-feed checks are tracked separately. Packaging success alone does not imply
that a release has been published.

Local Windows x64 MSVC 19.44/v143 Release checks passed: 32 contract cases, including 720
independent reference comparisons, standalone header and executable example. CMake was
4.3.1-msvc1. The final package and external consumers are checked by tools/Verify.ps1 after
a clean source commit. Linux sanitizer results require the corresponding remote CI evidence.

