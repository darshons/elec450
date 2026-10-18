# COMP/ELEC/MECH 450/550 - Project 3 (2026)

Implement the Random Tree Planner (RTP) in C++ and plan motions for the seven-joint Panda arm from Project 2. Compare RTP with OMPL's RRT, RRT-Connect, and PRM, and visualize the paths in Viser.

The assignment is in [`pdf/project3_2026.pdf`](pdf/project3_2026.pdf), with editable LaTeX in [`src/project3_2026.tex`](src/project3_2026.tex). Part 1 (Exercises 1 and 2) is due **October 15, 2026**. Part 2 (Exercise 3) is due **October 20, 2026**.

## 1. Activate and build

Use your Project 2 environment:

```bash
micromamba activate ompl_env
cd project3_2026
make
```

Run `make` after each C++ edit, then restart the Python program. The reviewed implementation uses OMPL 2.0.2 and Python 3.12. On Linux, create an environment if you do not already have the Project 2 environment:

```bash
micromamba create -n ompl_env -c conda-forge python=3.12 ompl=2.0.2 cxx-compiler cmake ninja
micromamba activate ompl_env
python -m pip install cmake nanobind matplotlib -e ./visualization/viser-motion-planning
make
```

Install the bundled visualization package even if you have a different Project 2 checkout installed. Its dependencies include NumPy, SciPy, PyYAML, Viser, trimesh and yourdfpy. The benchmark converter `ompl_benchmark_statistics.py` must be on `PATH` (provided by the OMPL environment). Check it with `command -v ompl_benchmark_statistics.py`.

The Docker scripts in `../docker/` are historical helpers tied to the original author's Linux paths and `/tmp/p3` setup. They are not required by the commands here and are not portable as written. The October 14 review built isolated copies on macOS and Ubuntu 24.04 ARM64. An Apple-only CMake visibility fix restored goal sampling through the Python extension; RTP and collision-checking C++ sources are unchanged. A fresh Ubuntu 24.04 ARM64 build with OMPL 2.0.2 also passed the support checks, benchmark-conversion checks, RTP lifecycle checks, and all four custom-scene binding smoke cases.

## 2. Run the supplied planner

RTP and the three OMPL planners are implemented in this checkout. For a single RRT-Connect trial:

```bash
python plan_arm.py --planner RRTConnect --output results/path.json --visualize
```

Each **query** specifies a different **start/goal pair** in the same scene. For example, `--query query2` uses the start and goal stored under `query2` in the scene YAML; the obstacles stay the same. The supplied scenes contain `query1` through `query5`; omitting `--query` selects `query1`.

Use **`--seed`** to change the planner's random samples. The default seed is `1`, and the seed is recorded in the saved result. For example:

```bash
python plan_arm.py --planner RRTConnect --query query2 --seed 42 \
  --output results/path.json --visualize
```

Open **http://localhost:8080**. Use the playback slider or play control to inspect the arm's motion. The result file records the scene, start and goal, joint order, planner settings, random seed, solution status, and path.

Replay the saved result later:

```bash
python visualize_path.py results/path.json
```

Replay uses the supplied Panda assets and Python visualization package. It does not need the compiled planner extension.

## 3. RTP implementation

The student implementation is in [`src/RTP.h`](src/RTP.h) and [`src/RTP.cpp`](src/RTP.cpp), including `solve()`, `clear()`, `getPlannerData()`, and state ownership. The setter exposes the goal-bias parameter.

RTP selects an existing tree vertex uniformly at random, samples a target with goal bias, checks the full connecting motion, and adds the target when that motion is valid. Recover a solution using parent pointers. If time expires, return the path to the closest tree vertex as an approximate solution.

Use OMPL's state-space, validity, and goal interfaces. The planner should work independently of the robot's geometry. The original starter RTP returned `timeout`; that placeholder has been replaced.

```bash
make
python plan_arm.py --planner RTP --goal-bias 0.1 --time-limit 5 \
  --output results/rtp_path.json --visualize
```

The binding and scene support are provided. Students implement the planning algorithm in C++.

## 4. Create arm scenes

The seven-joint Panda arm is used in all three exercises. Its configuration is a vector of seven joint angles in radians. The configuration space is the product of the seven bounded joint intervals from the URDF, represented by OMPL's `RealVectorStateSpace(7)`. The base and gripper are fixed.

For Exercise 2, create **two custom 3D scenes** with sphere and oriented-box obstacles. Copy a scene from [`resources/scenes/`](resources/scenes/) and edit its obstacles and queries. Choose valid start and goal arm configurations whose direct joint-space connection collides with an obstacle.

Run your C++ RTP planner in both scenes and visualize the arm's motion in **Viser**. Include images of the scenes, start and goal configurations, and solution paths in your report. Submit the scene files and saved paths.

```bash
python plan_arm.py --scene resources/scenes/shelf.yaml --query query1 \
  --planner RRTConnect --output results/shelf_path.json --visualize
```

Scene conventions match Project 2:

- Positions, radii, and box half-extents are in meters.
- Obstacle quaternions use `[x, y, z, w]` order.
- Configurations contain seven angles in radians, from `panda_joint1` through `panda_joint7`.
- Joint limits come from the Panda URDF; the base and gripper are fixed.
- Collision checking uses all 59 robot spheres against environment spheres and oriented boxes. Self-collision is outside this assignment's model.

The supplied C++ forward kinematics and Project 2 collision checker run inside each planning call. OMPL checks edges at resolution `0.005` times the configuration space's maximum extent. Keep that resolution fixed in your comparison. Viser playback interpolates the geometric joint path for display; it does not assign robot velocity or acceleration limits.

## 5. Run experiments

The existing [`configs/planners.json`](configs/planners.json) contains 20 settings and already exceeds the three-per-planner minimum. The four supplied databases each have 500 trials; no rerun is needed to fill the required experiment matrix.

| Planner | Parameters |
| --- | --- |
| RTP | `goal_bias`, `time_limit` |
| RRT | `goal_bias`, `planner_range`, `time_limit` |
| RRTConnect | `planner_range`, `time_limit` |
| PRM | `max_neighbors`, `time_limit` |

For a deliberate new experiment, run five trials on each of the five queries into a new directory:

```bash
python benchmark.py --scene resources/scenes/clutter.yaml \
  --repetitions 5 --output review_runs/benchmark.db
```

Upload **`results/benchmark.db`** on the **Change database** page of [Planner Arena](https://planner-arena.kavrakilab.rice.edu/). Use its plots to compare planners and download figures for your report.

The C++ code uses `ompl::tools::Benchmark`; OMPL runs the repetitions and calls each planner's `clear()` between trials. Its `ompl_benchmark_statistics.py` tool converts the native logs into the database. New native logs are written beside the output database in a `<database_stem>_logs/` folder. The committed experiments' original native logs are missing from this checkout; only their converted per-run databases are available. Running the command again replaces the destination database. Use a new output directory to preserve existing results. Compare equal time limits first.

| Quantity | Definition |
| --- | --- |
| Planning time | Seconds spent in `solve()`, including collision checking |
| Path length | Sum of Euclidean joint-space edge lengths, in radians |
| Smoothness | OMPL `PathGeometric::smoothness()` on the raw waypoints; lower is smoother under this metric |
| Minimum clearance | Minimum robot-sphere surface distance to an obstacle over sampled path edges, in meters |

The custom `minimum clearance` benchmark property uses the same edge resolution as planning and is a sampled estimate. OMPL's separate `solution clearance` property is the mean clearance at path waypoints. Both clearances are in meters. No shortcutting or smoothing is applied to benchmark paths. See [OMPL's path metrics](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1PathGeometric.html).

Inspect solution status alongside path quality: an approximate solution ends short of the goal. Use `plan_arm.py --visualize` to view representative paths with the planner settings you selected.

## Files

| Location | Purpose |
| --- | --- |
| `src/RTP.h`, `src/RTP.cpp` | Student planner implementation |
| `provided/` | C++ arm model, collision checker, OMPL setup, and binding |
| `arm_scene.py` | URDF and scene loading |
| `plan_arm.py` | One planning trial and JSON export |
| `visualize_path.py` | Viser playback of a saved result |
| `benchmark.py`, `configs/` | OMPL benchmarks, Planner Arena database, and planner settings |
| `resources/` | Panda model, meshes, and example scenes |
| `visualization/viser-motion-planning/` | Project 2 visualization toolkit |
| `tests/` | Checks of the supplied planning infrastructure |

## Submit

Include your sources, custom scenes, saved example paths, experiment settings, raw results, and report. Run `make clean` to remove the compiled extension and build directory, then zip the project as `Project3_<your NetID>_<partner NetID>.zip`. The handout gives the report requirements and grading.

## Check the supplied infrastructure

```bash
python tests/check_support.py
```

These checks cover forward kinematics against Project 2 reference data, collision geometry, scene queries, built-in planners, and exact versus approximate results. They do not test your RTP implementation. Do not use `--check-starter` with the completed RTP: that optional check expects the old placeholder.

To rebuild the handout with a LaTeX installation:

```bash
pdflatex -output-directory=pdf src/project3_2026.tex
```

## Part 1 (our submission)

RTP is in `src/RTP.h` and `src/RTP.cpp`. Our two custom scenes are `resources/scenes/wall.yaml` and `resources/scenes/window.yaml`, each with two queries. Saved RTP paths (goal bias 0.05, 10 s limit) are in `results/rtp_<scene>_<query>.json`, plus one approximate example, `results/rtp_window_query2_approx_seed2.json`. Viser screenshots are in `fig/`, and the write-up is `pdf/project3_part1_writeup.pdf` (source in `report/`).

```bash
make
python plan_arm.py --scene resources/scenes/wall.yaml --query query1 --planner RTP \
  --goal-bias 0.05 --time-limit 10 --seed 1 --output review_runs/rtp_wall_query1.json --visualize
python visualize_path.py results/rtp_window_query2.json
```

Build the write-up from this directory with two passes so figure references resolve:

```bash
pdflatex -halt-on-error -output-directory=pdf report/project3_part1_writeup.tex
pdflatex -halt-on-error -output-directory=pdf report/project3_part1_writeup.tex
```

It uses `comp450.cls`, which needs the `algorithms` LaTeX package (texlive-science). The final Part 1 report uses wider `fig/wall_query1_path_part1.png` and `fig/wall_query2_path_part1.png` views and an unclipped `fig/window_query2_path_part1.png` view rendered from the unchanged saved JSON paths; original screenshots remain available. Its focused archive includes the Part 1 report, scenes, paths, figures, planner/build sources and supplied robot/visualization assets. Part 2 report and experiment outputs stay in this checkout and are excluded from that Part 1 archive.

## Part 2 (our submission)

Our 20 planner settings are in `configs/planners.json` (5 RTP goal biases, 6 RRT goal bias/range pairs, 5 RRT-Connect ranges, 4 PRM neighbor counts, 5 s limit). Raw results are in `results/benchmark.db` (main run, clutter, 5 trials per query) plus `results/benchmark_t1.db`, `results/benchmark_t20.db` (same with 1 s and 20 s limits) and `results/benchmark_shelf.db` (shelf scene, 5 s). The matching original native logs were not committed and are unavailable here. Recover them from the machine that ran the experiments before considering any rerun. The database rows retain the raw per-trial metrics. `analyze_benchmark.py` makes the summary CSVs in `results/` and the plots in `fig/`. Representative paths for our selected settings are `results/clutter_query1_<planner>.json`. The full report is `pdf/project3_report.pdf` (source `report/project3_report.tex`). Existing figures were generated locally by `analyze_benchmark.py`; there is no retained evidence of a Planner Arena upload or download. Complete that handout requirement separately without replacing these databases.

```bash
python benchmark.py --scene resources/scenes/clutter.yaml --repetitions 5 --output review_runs/benchmark.db
python benchmark.py --scene resources/scenes/clutter.yaml --repetitions 5 --time-limit 1 --output review_runs/benchmark_t1.db
python benchmark.py --scene resources/scenes/clutter.yaml --repetitions 5 --time-limit 20 --output review_runs/benchmark_t20.db
python benchmark.py --scene resources/scenes/shelf.yaml --repetitions 5 --output review_runs/benchmark_shelf.db
python analyze_benchmark.py --db results/benchmark.db --tag review_main
pdflatex -output-directory=pdf report/project3_report.tex
```

## October 14 review and verification

No RTP, collision-checking, scene, saved-path, screenshot, planner-configuration, or benchmark database files were changed in the review. CMake adds an Apple-only default symbol-visibility setting: nanobind's hidden visibility prevented the extension from recognizing OMPL sampleable goals in the reviewed macOS build. An obstacle-free goal-bias-1.0 case was approximate before the fix and exact afterward. Linux flags are unchanged. See [nanobind's visibility documentation](https://nanobind.readthedocs.io/en/latest/api_cmake.html#nanobind-set-visibility). The two reports distinguish original Linux measurements from local smoke checks, and use saved custom examples rather than unsupported aggregate trial claims. The reports retain the draft difficulty/hour estimates; both partners should review their contribution statements before submission.

Read-only verification of saved artifacts and every number in the full benchmark table:

```bash
python -B tests/check_review.py > /tmp/project3_artifact_audit.json
```

This checks independent FK against the Pinocchio fixture, scene/query consistency, joint limits, obstructed direct paths, sampled path clearance, RTP parent relationships, database integrity, all 500 setting/query/trial combinations per database, recorded parameters, summary CSVs, and per-planner metric winners. It does not regenerate results. Add `--native-dir /path/to/built/project3_2026 --smoke` for exact and zero-time approximate binding checks plus four fresh-process RTP trials in the custom scenes with the saved examples' seeds and 10 s limits. Exact and approximate statuses are checked separately; these are smoke checks, not a success-rate experiment. A nominal seed is not a guarantee of the same path across platforms/builds.

The standalone lifecycle check compiles the unchanged RTP source:

```bash
python -m cmake -S tests/rtp_lifecycle -B /tmp/project3_lifecycle_build -DCMAKE_PREFIX_PATH="$CONDA_PREFIX"
python -m cmake --build /tmp/project3_lifecycle_build
/tmp/project3_lifecycle_build/check_rtp_lifecycle
```

It checks exact/approximate paths, continuation, 25 clear/reuse cycles, start/goal labels and tree edges, invalid starts, independent solution-path ownership, and balanced state allocations/frees. It does not replace a general leak detector. `../docker/verify_rtp.py` is an older diagnostic that reseeds inside one process and only prints failures; use the review checks for documented pass/fail verification.

Planning and benchmark collision resolution stays at 0.005 (about 0.067 rad maximum joint-space sampling interval). The denser audit uses at most 0.002 rad. All four saved custom RTP paths pass that denser check. The saved clutter RRT-Connect example has approximately 0.84 mm penetration between validator samples despite 0.27 mm recorded positive clearance. Neither a positive sampled minimum nor an exact solution proves continuous collision freedom. No benchmark paths were altered or smoothed.

For isolated build verification, copy the project into a new working directory and run `make` there: CMake intentionally writes the extension into its source directory, even with an external build directory.

Before packaging, stage a copy of the project outside the working checkout, run `make clean` in that staged copy, and zip it as `Project3_wl85_ds161.zip`. Keep Part 1 and Part 2 archives in separate directories. Part 1 needs the short report, RTP and support sources/assets, custom scenes, paths and figures; Part 2 additionally needs the complete report, configurations and raw experiment databases. Do not include build outputs or caches. The expected Part 2 database is still `results/benchmark.db`, not `review_runs/benchmark.db`.

The full benchmark table is retained; `analyze_review.py` creates the additional readable representative-settings figure from the existing database without touching original plots:

```bash
python analyze_review.py --output fig/bench_review_selected_metrics.pdf
```

Verification summaries are retained in `review/validation.json` and explained in `review/README.md`. Remaining checks: partners' report/contribution review, recovery of native logs if available, and Planner Arena upload/download verification. Nothing was uploaded or submitted during the review.
