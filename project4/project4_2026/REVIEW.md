# Accuracy reviews

Scope: example paths for RRT, KPIECE1 and RG-RRT. All twelve example paths reach the
goal regions; they are illustrative runs, not repeated-trial benchmarks.

1. Implementation review: equations, signs, state ordering, control bounds,
   angle topology, velocity bounds, obstacle contact, value ownership in
   callbacks, goal metrics and explicit KPIECE projections checked against
   the handout. C++ analytical model tests and OMPL control-path checks passed.
2. Independent results review: SciPy RK45 replay with rtol 1e-11 and atol
   1e-12, samples at most 0.002 s apart, verified local endpoint agreement,
   cumulative replay drift, original states, dense plotted states, controls,
   durations, bounds, collisions and final goals. All twelve paths passed.
   Local errors are below 4e-11; maximum cumulative drift is below 5e-8.
   Car minimum sampled obstacle clearances are 0.06715 m (RRT), 0.03338 m
   (KPIECE1) and 0.05945 m (RG-RRT). See results/validation.json for full precision.

Report reviews: numeric statements compared with summary.csv and saved JSON;
Checkpoint PDF confirmed one letter-size page and visually inspected for legibility,
labels, paths, captions, clipping and the equal contribution statement.

Package reviews: clean source archive checked for complete expected contents,
CRC integrity and matching PDF; extracted source rebuilt and model tests run
separately. No executables, build directories or Python caches are submitted.
Native validation uses macOS ARM64 / AppleClang 17 / OMPL 2.0.2.

## Final submission audit — November 1, 2026

- Rebuilt both executables and ran CTest successfully (OMPL 2.0.2).
- Fixed the reachable-control cache leak in RG-RRT's destructor. Regression
  checks count control allocations across setup, clear, repeated setup and
  destruction, and solve a pendulum query again after clearing the planner.
- Independently replayed all twelve bundled paths again; all passed. The
  original benchmark runs remain unchanged by the memory-management fix.
- Reimported the raw top-level benchmark logs into a separate temporary SQLite
  database. All 685 run records exactly match the bundled database for seed,
  planner, solution status, solve time, duration and tree-node count.
- Corrected the unsupported minimum swing-count argument, qualified causal
  explanations and CPU-contention comparisons, and retained the team-confirmed
  hours and contributions. Rebuilt the five-page final report and ten-slide
  presentation; checked rendering, captions, tables and plots.
- The benchmark runner now refuses existing output directories and checks every
  child process before database import; shell syntax and refusal were checked.
- Recreated the final archive from clean source, verified its CRCs and contents,
  and rebuilt and tested the extracted submission separately.

Submission status: final code/report and slides are prepared locally. Canvas
submission and the team's 5–7 minute presentation recording remain manual steps.
