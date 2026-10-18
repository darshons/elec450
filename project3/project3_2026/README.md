# COMP/ELEC/MECH 450/550 — Project 3

Darshon Singh (`ds161`) and Winson Lin (`wl85`). The assignment is
[`pdf/project3_2026.pdf`](pdf/project3_2026.pdf).
The complete 1–5 page report is [`pdf/project3_report.pdf`](pdf/project3_report.pdf),
with source in `report/project3_report.tex`. Part 1's report, custom scenes,
paths, and images are preserved.

## Work split and target dates

Darshon handles RTP implementation and analysis; Winson handles custom environments,
visualization and benchmark experiments. Testing and report/submission preparation are shared.

| Task | Responsibility | Target |
| --- | --- | --- |
| RTP implementation | Darshon: C++ tree construction, path extraction, debugging and memory management | Oct 13 |
| Custom environments | Winson: two scenes, valid start/goal queries, Viser screenshots and saved paths | Oct 13 |
| Part 1 submission | Both: test RTP in both scenes, finish the short write-up and package code | Oct 13 |
| Benchmark experiments | Winson: configure the four planners with at least three settings each, run trials and collect results | Oct 17 |
| Results analysis | Darshon: compare all metrics and success rates, and explain algorithm differences | Oct 18 |
| Final report and submission | Both: combine results, figures and explanations; finalize code and submission | Oct 18 |

These are internal targets in 2026. Official deadlines are October 15 for
Part 1 and October 20 for Part 2. The final report follows this confirmed split;
the original Part 1 submission remains preserved.

## Build and check

Use the course environment and the bundled visualization package:

```bash
micromamba activate ompl_env
cd project3_2026
make
python tests/check_support.py
python tests/check_benchmark.py
python tests/check_part2.py
python -m cmake -S tests/rtp_lifecycle -B /tmp/project3_lifecycle \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX"
python -m cmake --build /tmp/project3_lifecycle
/tmp/project3_lifecycle/check_rtp_lifecycle
```

The October 18 build used macOS ARM64, Python 3.12.12, OMPL 2.0.2,
AppleClang 17 and Release mode. Full versions and source hashes are in
`results/part2/environment.json`. The existing Apple-only symbol-visibility
fix in CMake is retained; Linux flags and RTP sources are unchanged.
Earlier Linux ARM64 checks are documented in `review/README.md`.

If recreating the course environment on Linux:

```bash
micromamba create -n ompl_env -c conda-forge python=3.12 ompl=2.0.2 cxx-compiler cmake ninja
micromamba activate ompl_env
python -m pip install cmake nanobind matplotlib -e ./visualization/viser-motion-planning
make
command -v ompl_benchmark_statistics.py
```

Rebuild after C++ edits and restart Python. The lifecycle check covers 25
clear/reuse cycles, exact/approximate solutions, continued solving, trivial
and invalid starts, exported tree labels/edges, independent solution-path
ownership, and balanced state allocations. RTP uniformly selects a stored
vertex, uses goal-biased sampling and checks the full connection. It has no
nearest-neighbor selection, range-limited extension, rewiring, or intermediate
edge states stored as vertices. No RTP fix was needed for Part 2.

## Part 2 experiments

`configs/planners.json` contains 20 settings, all with a 5 s limit and
resolution 0.005:

| Planner | Settings |
| --- | --- |
| RTP | goal bias .01, .05, .1, .2, .5 |
| RRT | bias .05 at ranges .2, .4, 1, 2.5; biases .01, .2 at range 1 |
| RRT-Connect | ranges .2, .4, 1, 2.5, 5 |
| PRM | maximum neighbors 5, 10, 20, 40 |

Bias is a probability; range is Euclidean joint-space distance in radians;
maximum neighbors is a count. The RRT design isolates range at fixed bias
and bias at fixed range. Ranges span fine extensions to a substantial fraction
of the 13.4165 rad joint-box diagonal. PRM settings span sparse to denser local
connectivity. These are exploratory settings, not claimed optima.

A separate 100-run pilot (one trial per setting/query, 1 s, seed 18001) gave
86 exact and 14 approximate solutions. We chose 5 s for the final sequential
study to give harder settings longer while keeping the experiment practical.
The final study has **500 runs**: 20 settings × 5 clutter queries × 5 trials,
seed 18002. Pilot and historical runs are excluded from its summaries.

Reproduce into new directories, with no other timed jobs running:

```bash
python benchmark.py --scene resources/scenes/clutter.yaml --repetitions 1 \
  --time-limit 1 --seed 18001 --output results/reproduction_pilot/benchmark.db
python benchmark.py --scene resources/scenes/clutter.yaml --repetitions 5 \
  --seed 18002 --output results/reproduction/benchmark.db
python analyze_part2.py --db results/reproduction/benchmark.db --out results/reproduction \
  --fig-dir results/reproduction/fig
```

The supplied runner replaces an existing output database and matching log
files. Use new paths for new experiments. The final canonical database is
`results/benchmark.db`; an identical copy and the original native logs are in
`results/part2/`. The OMPL converter produces the database directly from those
logs. To reconstruct it without rerunning planning:

```bash
ompl_benchmark_statistics.py -d /tmp/project3_reconverted.db \
  results/part2/benchmark_logs/*.log
```

The canonical analysis command is:

```bash
python analyze_part2.py
```

It audits the database, query endpoints, actual planner parameters, resolution,
status enums, and all 100 setting/query groups; exports `runs.csv`,
`summary.csv`, `per_query.csv`, `schema.sql`, `experiments.json`, and
`winners.json` into `results/part2/`; and makes `fig/part2_*.pdf` and the
report table. Generated figures refer to the database supplied to this command.
The supplemental `fig/part2_parameter_effects.pdf` shows medians and interquartile
ranges for all settings and metrics. Legacy `bench_main_*` and
`bench_review_*` figures describe the archived October 14 data.
See `results/part2/README.md` for the data dictionary.

| Quantity | Definition and analysis |
| --- | --- |
| Solve time | OMPL `runs.time`, seconds including collision checking; all runs, actual measured timeout overhead retained |
| Length | Raw path's summed Euclidean joint-space edge lengths, radians; exact solutions only |
| Smoothness | Raw `PathGeometric::smoothness()`, lower is smoother under this geometric metric; exact only |
| Minimum clearance | Custom `minimum_clearance`, minimum sampled robot-sphere surface separation along edges, meters; exact only |
| Success | Exact count and percentage of 25 runs per setting, and of 5 per query |

The database enum table identifies exact and approximate statuses. In this
OMPL workflow `solved` means exact success; approximate endpoints are recorded
separately and do not enter path-quality summaries. `solution_clearance` is a
mean over waypoints, not the required minimum. No simplification or smoothing
is applied (`Benchmark::Request::simplify=false`). Collision geometry, URDF
bounds, resolution 0.005 (about 0.067 rad maximum interval), and the supplied
edge-clearance sampler are fixed. Positive sampled clearance does not prove
continuous collision freedom. All 478 exact paths passed the prescribed check,
but 14 failed OMPL's post-run check with four times the segment count: 4 RRT
and 10 RRT-Connect (6 at range .2). Exact means goal reaching under the
prescribed validator. These diagnostics are exported as `exact_strict_passed`
and `exact_strict_failed`; they do not change the primary exact-only summaries.

OMPL is initialized once per benchmark invocation. Its random generators then
advance across settings, repetitions and queries. The recorded experiment
seed is not an independently reset per-trial seed. Machine speed, scheduling
and timeout termination can affect reproducibility. Timed trials ran
sequentially; normal OS background activity was uncontrolled.

## Visualize paths

```bash
python plan_arm.py --scene resources/scenes/clutter.yaml --query query1 \
  --planner RRTConnect --range 1 --time-limit 5 --seed 18101 \
  --output results/new_example.json --visualize
```

Open http://localhost:8080 and use `Playing` or the timestep slider. Select a
query with `--query query1` through `query5`; obstacles stay fixed. Replay:

```bash
python visualize_path.py results/part2/paths/RRTConnect.json
```

Representative paths, playback captures and `selection.json` are in
`results/part2/paths/`. For each of RTP (bias .2), RRT (bias .05, range 1),
RRT-Connect (range 1), and PRM (k=10), we generate seeds 18101–18105 on query1
and select the exact path closest to that setting's benchmark query1 median length. Candidate successes and all five lengths
are retained. Examples are illustrations, not extra benchmark observations.
The length-based selection does not imply typical timing, clearance or smoothness.
The same selected seed is run with `plan_arm.py --visualize`, and a Viser
overview of its unsmoothed path is saved in `fig/part2_<planner>.png`.

To repeat automated browser inspection after installing Playwright:

```bash
python -m pip install playwright
python -m playwright install chromium
python inspect_part2_paths.py
# Or: python inspect_part2_paths.py --browser-executable /path/to/chrome
```

## Planner Arena

Open https://planner-arena.kavrakilab.rice.edu/, choose **Change database**,
and upload `results/benchmark.db`. Choose **Overall performance**, a clutter
query (or aggregate when offered), and planners/settings to compare. Select
`time` for solve time and `minimum clearance` for the custom metric. Keep
simplified results disabled. Download plots with **Download plot as PDF**.
Check the per-plot sample count: native approximate-path length/smoothness
properties may exist, so use our exact-only local summaries for the report's
path-quality comparison. Arena's time and solved plots use all trials.
The October 18 upload succeeded. Downloaded query1 time, success and minimum-clearance
PDFs, screenshots and upload provenance are in `results/part2/arena/`. The time
plot is included in the report; local plots pool all five queries.

## Preserved Part 1 and historical artifacts

- RTP: `src/RTP.h`, `src/RTP.cpp`; supplied binding and collision model: `provided/`.
- Custom scenes: `resources/scenes/wall.yaml`, `window.yaml`, two queries each.
- Original custom RTP paths: `results/rtp_<scene>_<query>.json`, plus the window
  query2 seed-2 approximate example. Images in `fig/` are retained unchanged.
- Part 1 report: `pdf/project3_part1_writeup.pdf`, source in `report/`.
- Old main database, summary, full report and README: `results/legacy_2026-10-14/`.
- Other older databases: `results/benchmark_t1.db`, `benchmark_t20.db`,
  `benchmark_shelf.db`; original native logs are unavailable. They are historical
  data, not measurements from this fresh session.
- `tests/check_review.py` now audits the archived October 14 main table and
  historical artifacts. `analyze_part2.py` audits the new study.

Original difficulty ratings and hour estimates are retained. The final contribution
statement follows the confirmed work split above. Partners should review their
personal time estimates before submitting.

## Build report and package

With a full LaTeX installation:

```bash
pdflatex -halt-on-error -output-directory=pdf report/project3_report.tex
pdflatex -halt-on-error -output-directory=pdf report/project3_report.tex
```

Alternatively use `tectonic -Z search-path="$PWD" -o pdf report/project3_report.tex` from this directory.
The class requires the algorithms package (texlive-science). The completed
report must remain 1–5 pages. Sources, configurations, scenes, example paths,
raw logs, CSV exports, databases, images and both reports belong in the archive.
Remove build outputs before packaging:

```bash
make clean
cd ..
mkdir -p submission_part2
zip -r submission_part2/Project3_wl85_ds161.zip project3_2026 \
  -x '*/__pycache__/*' '*/.DS_Store' '*/.git/*'
```

The prepared Part 2 archive is stored separately under `../submission_part2/`
to preserve the existing Part 1 archive in the parent directory. Upload the
Part 2 archive and complete report to Canvas yourself.
