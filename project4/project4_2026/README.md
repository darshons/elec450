# Project 4 — Out of Control Planning

Winson Lin (wl85) and Darshon Singh (ds161)

C++17 implementations of the pendulum and point-car dynamics, validity checks,
OMPL control RRT and KPIECE1 (with explicit projections), our Reachability-Guided
RRT (`src/RG-RRT.h`, `src/RG-RRT.cpp`) and OMPL Benchmark runs. The one-page
checkpoint report is `pdf/project4_part1_writeup.pdf`; the final report is
`pdf/project4_final_report.pdf` and the presentation `pdf/project4_slides.pdf`.

## Build and run

Use the same `ompl_env` as Project 3 (OMPL, C++ compiler, Python, CMake).
Built and tested with OMPL 2.0.2 on macOS ARM64 (AppleClang 17); the code uses
only standard C++17 and OMPL APIs.

```bash
micromamba activate ompl_env
make
./Project4Pendulum        # template menu: Plan/Benchmark, torque, planner
./Project4Car             # template menu: Plan/Benchmark, planner
```

With no arguments both programs show the starter's menus (planner 1 RRT,
2 KPIECE1, 3 RG-RRT; torque 1/2/3 = 3/5/10). Interactive planning writes
`pendulum_path.json` or `car_path.json`; interactive benchmarking writes
`pendulum.log` or `car.log`. For scripted runs:

```bash
./Project4Pendulum --planner RG-RRT --torque 3 --seed 450 --time 30 --output pendulum.json
./Project4Car --planner KPIECE1 --seed 451 --time 120 --output car.json
./Project4Pendulum --benchmark --torque 3 --runs 30 --time 30 --log pendulum.log
```

`--help` shows all options. Output parents must already exist. Exit code
0 means an exact solution within the specified goal region; 2 means no exact
solution before the time limit; 1 means invalid input or another error.
Approximate paths are never exported as successful examples. Choose a new
output filename: a successful run replaces that file. A failed run does not
remove a previously existing output file.

```bash
python src/run_examples.py --output results_new
```

This creates a new directory, runs the twelve examples sequentially in fresh
processes, and saves logs and original state/control/duration data. Seeds fix
the random sequence within this environment; timing and results can vary across
platforms. The bundled examples are illustrative single runs, not a benchmark
or evidence of aggregate planner performance. The first car KPIECE1 attempt
(seed 450, 60 s) was approximate; its log is retained separately.

## Models and parameters

Pendulum: `SO2 × R`, state `(theta, omega)`, angle zero horizontal,
`theta_dot=omega`, `omega_dot=-9.81*cos(theta)+tau`. Speed bound ±10 rad/s;
torque bounds ±3, ±5, or ±10. Start `(-pi/2,0)`, goal `(pi/2,0)`.
The goal distance is wrapped absolute angle difference plus absolute speed
difference, with tolerance 0.15. No obstacles. KPIECE projects to
`(theta,omega)` with cells `(0.25,0.5)`; the angular grid has a seam at ±pi,
while state-space distance correctly wraps across that seam.

Car: `SE2 × R`, state `(x,y,theta,v)`, controls `(angular_velocity,acceleration)`;
`x_dot=v*cos(theta)`, `y_dot=v*sin(theta)`, `theta_dot=u0`, `v_dot=u1`.
Workspace `[0,12] × [0,10]` m, speed ±2 m/s, both controls ±1 in their
respective units (rad/s and m/s²). Reverse is allowed. Start `(1,1,pi/2,0)`;
goal `(11,9,pi/2,0)`. Goal distance is Euclidean position distance plus
0.5 times wrapped absolute heading difference plus absolute speed difference,
with tolerance 0.5. This is OMPL's compound metric, not a 0.5 m position-only
tolerance. The point model is permitted by the assignment; the plotted line
represents the point, not a finite car footprint. Buildings are closed rectangles
`(x,y,width,height)`: `(3,0,1,6.5)`, `(6,3.5,1,6.5)`, `(9,0,1,6.5)`.
KPIECE projects to `(x,y,v)` with cell sizes `(0.5,0.5,0.5)`, retaining velocity
information. The obstacles are captured by value in the validity callback.

Both systems: control RRT or control KPIECE1, goal bias 0.1, ODE RK4 integration
step 0.001 s, propagation/validity step 0.02 s, sampled durations of 1–25
propagation steps (0.02–0.50 s). Post-propagation callbacks wrap only the
angle; speeds and positions are never clamped to manufacture valid states.
All bounds and, for the car, obstacle collisions are checked by the validator.
CollisionChecking.cpp provides our Project 2 closed-contact point, circle and
rotated-square checks; the car uses the point checker.

## Two review passes

After implementation, analytical model tests check gravity signs, equilibria,
reverse velocity, constant-acceleration propagation, projections, bounds and
collision/contact cases. After generating paths, OMPL's `PathControl::check()`
and goal satisfaction are checked in C++; then Python independently integrates
every saved control using SciPy RK45 at tighter tolerance, checks each segment's
endpoint, and samples at ≤0.002 s for bounds, obstacles, and final goal distance.
The Python checker also performs cumulative replay without resetting at nodes
and records its drift. Cumulative divergence can grow for the unstable pendulum;
local error is the relevant numerical propagation consistency check.

```bash
python -m cmake --build build --target check_models
ctest --test-dir build --output-on-failure
python src/analyze_paths.py  # requires numpy, scipy, matplotlib
```

The analysis audits the twelve bundled JSON paths, writes `results/summary.csv`
and `results/validation.json`, and regenerates `fig/checkpoint_paths.*` (RRT,
KPIECE1) and `fig/all_paths.*` (all three planners). `dense` states come from `PathControl::asGeometric()` (dynamic propagation,
not linear interpolation). Original controls/durations are retained. Angles are
wrapped for display and lines break at the ±pi seam. Green circles are starts,
red stars are goals. Car samples also have obstacle clearance exceeding the
0.002 m nearest-sample travel bound at speed ≤2 m/s; discrete checks still
have numerical integration error. The planner itself uses 0.02 s checking.

## RG-RRT

`ompl::control::RGRRT` extends OMPL's control RRT. Every tree node stores an
approximate reachable set R(q): the valid end states of `k` controls applied
for `s` propagation steps (`propagateWhileValid`; a state is kept only if the
whole segment is valid). The `k` controls are evenly spaced over the bounds of
control dimension 0 (pendulum torque, car steering rate); other dimensions are
held at the centre of their bounds (car acceleration 0). A random sample is
discarded unless some state of R(q_near) is closer to it than q_near; otherwise
q_near is extended as in RRT. Defaults `k=11`, `s=5` (0.1 s), goal bias 0.1;
set with `--rg-controls`/`--rg-steps` or the planner parameters
`reachable_controls`/`reachable_steps`. Assumes a bounded
`RealVectorControlSpace`.

## Example paths

```bash
python src/run_examples.py --output results_new   # 12 runs: 3 planners x (3 torques + car)
python src/analyze_paths.py                       # audits results/*.json, draws fig/all_paths.*
```

The audit integrates every saved control independently with SciPy and checks
bounds, collisions and the goal. The car RG-RRT example (seed 450) solved in 172 s
while the benchmarks were sharing the CPU; an earlier 120 s attempt with the same seed was
approximate and is kept as `results/car_RG-RRT_120s.log`.

## Benchmarks

```bash
bash src/run_benchmarks.sh results/benchmark   # parallel OMPL Benchmark runs, then benchmark.db
python src/analyze_benchmarks.py                # summary.csv, fig/bench_*.pdf, report tables
```

Experiments (seed in parentheses): pendulum torque 3, RRT/KPIECE1/RG-RRT,
30 runs, 30 s (1001); car, same planners, 25 runs each, 60 s (2001); RG-RRT
reachable-set study on the pendulum, 30 runs (3001) and car, 20 runs (4001);
pendulum torque 5 and 10, 30 runs (5005, 5010). The raw OMPL logs and
`benchmark.db` are in `results/benchmark/`; the database can also be loaded in
Planner Arena. Path length in OMPL's control benchmarks is the trajectory
duration in seconds. Processes ran concurrently on an 8-core laptop, so absolute
times include some contention; all planners within an experiment ran under the
same conditions.

## Reports and archive

```bash
pdflatex -output-directory=pdf report/project4_part1_writeup.tex      # checkpoint
pdflatex -output-directory=pdf report/project4_final_report.tex       # final (run twice)
pdflatex -output-directory=pdf report/project4_slides.tex             # presentation (run twice)
make clean
cd ..
zip -r Project4_wl85_ds161.zip project4_2026 -x '*/__pycache__/*' '*/.DS_Store' '*/build/*'
```

## Contributions (equal, 50% each)

Darshon Singh: pendulum state/control model, validity checker, RRT and KPIECE
setup, pendulum torque examples, and pendulum analysis.
Winson Lin: car state/control model, street environment and collision checking,
car RRT and KPIECE setup, car examples, and visualization.
Both: integration, the two accuracy review passes, interpretation, report
preparation, and submission packaging. The overall work was split evenly.
