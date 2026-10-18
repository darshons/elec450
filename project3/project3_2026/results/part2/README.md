# Fresh Part 2 study — October 18, 2026

This directory is the new study. Do not pool it with `../pilot` or the older databases.
The canonical submission database is also copied to `../benchmark.db` after completion.

- `environment.json`: commands, package/build versions, configuration, source hashes, seeds, fixed model, and pilot decision.
- `benchmark.db`, `benchmark_logs/*.log`, `console.txt`: native OMPL results, converter output and console record. Five logs, one per query; 20 configurations × 5 trials in each log.
- `runs.csv`: all raw database run fields, settings and available experiment seeds, plus decoded outcome and query. Trial numbers describe order within a setting/query, not independent seed values.
- `summary.csv`: 25 runs per setting, exact counts and percentages, approximate/failure counts, all-run solve-time summaries, exact-only quality summaries and quartiles.
- `per_query.csv`: corresponding summaries for each of the 100 setting/query groups (5 runs each).
- `schema.sql`, `status_enums.json`: inspected database schema and actual status mapping. `status=6` is exact; `5` is approximate; all other statuses are failures. The exporter discovers these codes by description and checks them against `solved` and `approximate_solution`.
- `table.tex`, `winners.json`: generated report table and metric winners, each with supporting exact counts.
- `paths/`: independent illustrative paths, logs, Viser captures, candidate-selection records. These runs are excluded from the study. The displayed example is the exact candidate closest to the benchmark query1 median length among seeds 18101–18105, separately for each planner; this is not claimed to reproduce a benchmark trial.

`time` measures OMPL's solve call including collision checks. Keep approximate and failed runs at their actual elapsed times; do not clip to the nominal timeout. A timeout can exceed the budget while a collision check completes. No paths are simplified or smoothed. Path quality uses exact results only. `minimum_clearance` is the supplied minimum sampled robot-sphere surface distance along edges, in meters; `solution_clearance` is a separate waypoint average and is not used as the minimum.

The 100-run pilot used one run per setting/query, 1 second, seed 18001: 86 exact, 14 approximate, no failures. The final study uses 5 seconds and seed 18002. Both initialize OMPL once per invocation, then advance generator state. They are not paired-seed experiments. Timing and platform changes can alter paths. Ordinary background OS activity was not controlled.

Reproduce from the project directory with the course environment active:

```bash
python benchmark.py --scene resources/scenes/clutter.yaml --repetitions 5 \
  --seed 18002 --output results/reproduction/benchmark.db
python analyze_part2.py --db results/reproduction/benchmark.db --out results/reproduction \
  --fig-dir results/reproduction/fig
```

Use a new output path: the supplied benchmark runner replaces an existing destination database and native logs.

Historical artifacts remain in `../legacy_2026-10-14/` and the original `benchmark_t1.db`, `benchmark_t20.db`, `benchmark_shelf.db`. Their original native logs are unavailable. The final report uses only the fresh main study for quantitative comparisons.

Final validation is recorded in `validation.json`. The native logs reconstruct all five database tables exactly. All four selected example paths also have positive independently sampled clearance at .002 rad intervals (`paths/dense_check.json`); this does not prove continuous collision freedom. The Planner Arena upload and three query1 PDF downloads completed; see `arena/provenance.json`. Partners still need to review personal time estimates and submit to Canvas.

The accuracy audit found that all 478 exact paths passed the prescribed check, while 14 failed OMPL's additional post-run check with four times the segment count (4 RRT, 10 RRT-Connect, including 6 at range .2). Exact status denotes goal reaching; it is not a continuous collision-freedom certificate. Summary and per-query exports now include the separate strict-check counts.

`strict_sensitivity.csv` provides a secondary quality comparison excluding the 14 finer-check failures. Its shortest pooled median is RRT bias .05, range 1 (4.11743 rad, 23 retained exact paths); the smoothness and clearance winners remain PRM k=10 and RTP bias .05. This check does not replace the prescribed-resolution comparison or prove continuous collision freedom.
