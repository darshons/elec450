# Part 2 results (October 18, 2026)

These are the 500 runs used in the report. `../benchmark.db` is an identical copy of `benchmark.db`.

- `environment.json`: commands, package/build versions, configuration, source hashes and seeds.
- `benchmark.db`, `benchmark_logs/*.log`, `console.txt`: native OMPL results, the converter output and the console record. There are five logs, one per query, each with 20 settings × 5 trials.
- `runs.csv`: every database run with its setting, query and decoded outcome. Trial numbers give the order within a setting/query, not separate seeds.
- `summary.csv`: 25 runs per setting: exact counts and percentages, approximate/failure counts, solve-time summaries over all runs, and path-quality summaries and quartiles over exact runs.
- `per_query.csv`: the same summaries for each of the 100 setting/query groups (5 runs each).
- `schema.sql`, `status_enums.json`: database schema and status codes. `status=6` is exact, `5` is approximate, anything else is a failure. These match the `solved` and `approximate_solution` columns.
- `table.tex`, `winners.json`, `analysis.txt`: the report table, the best setting for each metric, and the analysis output.
- `strict_sensitivity.csv`: path-quality medians after dropping the 14 exact paths that failed OMPL's finer post-run check.
- `paths/`: the example paths in the report, with logs, Viser captures and `selection.json`. These are separate runs and are not part of the benchmark statistics. For each planner we ran query1 with seeds 18101–18105 and kept the exact path closest to that setting's benchmark query1 median length.
- `arena/`: Planner Arena query1 plots (time, solved, minimum clearance), a screenshot and upload details.

`time` is OMPL's solve time including collision checks. Approximate and failed runs keep their actual elapsed times, which can go slightly past 5 s while a collision check finishes. Paths are not simplified or smoothed. Path quality uses exact results only. `minimum_clearance` is the minimum sampled robot-sphere surface distance along the path, in meters; `solution_clearance` is OMPL's waypoint average and is not used.

All 478 exact paths passed the check at the required resolution. 14 failed OMPL's post-run check with four times as many segments (4 RRT, 10 RRT-Connect, 6 of them at range .2). `summary.csv` and `per_query.csv` list these as `exact_strict_passed` and `exact_strict_failed`.

OMPL is seeded once per benchmark run (seed 18002), so trials do not have independent seeds. Normal background OS activity was not controlled.

To reproduce from the project directory with the course environment active:

```bash
python benchmark.py --scene resources/scenes/clutter.yaml --repetitions 5 \
  --seed 18002 --output results/reproduction/benchmark.db
python analyze_part2.py --db results/reproduction/benchmark.db --out results/reproduction \
  --fig-dir results/reproduction/fig
```

Use a new output path, since the benchmark runner replaces an existing database and its logs.
