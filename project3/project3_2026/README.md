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

Run `make` after each C++ edit, then restart the Python program.

## 2. Run the supplied planner

RRT-Connect works before you implement RTP:

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

## 3. Implement RTP

Edit [`src/RTP.h`](src/RTP.h) and [`src/RTP.cpp`](src/RTP.cpp). Complete `solve()`, `clear()`, `getPlannerData()`, and the planner's state ownership. The supplied setter exposes the goal-bias parameter.

RTP selects an existing tree vertex uniformly at random, samples a target with goal bias, checks the full connecting motion, and adds the target when that motion is valid. Recover a solution using parent pointers. If time expires, return the path to the closest tree vertex as an approximate solution.

Use OMPL's state-space, validity, and goal interfaces. The planner should work independently of the robot's geometry. The starter compiles, but its unfinished RTP returns `timeout`.

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

Edit [`configs/planners.json`](configs/planners.json) to add parameter settings. Use at least three settings per planner for the assignment.

| Planner | Parameters |
| --- | --- |
| RTP | `goal_bias`, `time_limit` |
| RRT | `goal_bias`, `planner_range`, `time_limit` |
| RRTConnect | `planner_range`, `time_limit` |
| PRM | `max_neighbors`, `time_limit` |

Run five trials on each of the five queries:

```bash
python benchmark.py --scene resources/scenes/clutter.yaml \
  --repetitions 5 --output results/benchmark.db
```

Upload **`results/benchmark.db`** on the **Change database** page of [Planner Arena](https://planner-arena.kavrakilab.rice.edu/). Use its plots to compare planners and download figures for your report.

The C++ code uses `ompl::tools::Benchmark`; OMPL runs the repetitions and calls each planner's `clear()` between trials. Its `ompl_benchmark_statistics.py` tool converts the native logs into the database. Logs are kept in `results/benchmark_logs/`. Running the command again replaces the database. Compare equal time limits first.

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

These checks cover forward kinematics against Project 2 reference data, collision geometry, scene queries, built-in planners, and exact versus approximate results. They do not test your RTP implementation. Before implementing RTP, add `--check-starter` to also verify the placeholder's timeout behavior.

To rebuild the handout with a LaTeX installation:

```bash
pdflatex -output-directory=pdf src/project3_2026.tex
```

## Part 1 (our submission)

RTP is in `src/RTP.h` and `src/RTP.cpp`. Our two custom scenes are `resources/scenes/wall.yaml` and `resources/scenes/window.yaml`, each with two queries. Saved RTP paths (goal bias 0.05, 10 s limit) are in `results/rtp_<scene>_<query>.json`, plus one approximate example, `results/rtp_window_query2_approx_seed2.json`. Viser screenshots are in `fig/`, and the write-up is `pdf/project3_part1_writeup.pdf` (source in `report/`).

```bash
make
python plan_arm.py --scene resources/scenes/wall.yaml --query query1 --planner RTP \
  --goal-bias 0.05 --time-limit 10 --seed 1 --output results/rtp_wall_query1.json --visualize
python visualize_path.py results/rtp_window_query2.json
```

Build the write-up from this directory with `pdflatex -output-directory=pdf report/project3_part1_writeup.tex`. It uses `comp450.cls`, which needs the `algorithms` LaTeX package (texlive-science).

## Part 2 (our submission)

Our 20 planner settings are in `configs/planners.json` (5 RTP goal biases, 6 RRT goal bias/range pairs, 5 RRT-Connect ranges, 4 PRM neighbor counts, 5 s limit). Raw results are in `results/benchmark.db` (main run, clutter, 5 trials per query) plus `results/benchmark_t1.db`, `results/benchmark_t20.db` (same with 1 s and 20 s limits) and `results/benchmark_shelf.db` (shelf scene, 5 s). Native logs are in the matching `results/*_logs/` folders. `analyze_benchmark.py` makes the summary CSVs in `results/` and the plots in `fig/`. Representative paths for our selected settings are `results/clutter_query1_<planner>.json`. The full report is `pdf/project3_report.pdf` (source `report/project3_report.tex`).

```bash
python benchmark.py --scene resources/scenes/clutter.yaml --repetitions 5 --output results/benchmark.db
python benchmark.py --scene resources/scenes/clutter.yaml --repetitions 5 --time-limit 1 --output results/benchmark_t1.db
python benchmark.py --scene resources/scenes/clutter.yaml --repetitions 5 --time-limit 20 --output results/benchmark_t20.db
python benchmark.py --scene resources/scenes/shelf.yaml --repetitions 5 --output results/benchmark_shelf.db
python analyze_benchmark.py --db results/benchmark.db --tag main
pdflatex -output-directory=pdf report/project3_report.tex
```
