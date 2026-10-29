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
PDF confirmed one letter-size page and visually inspected for legibility,
labels, paths, captions, clipping and the equal contribution statement.

Package reviews: clean source archive checked for complete expected contents,
CRC integrity and matching PDF; extracted source rebuilt and model tests run
separately. No executables, build directories or Python caches are submitted.
Native validation uses macOS ARM64 / AppleClang 17 / OMPL 2.0.2.
