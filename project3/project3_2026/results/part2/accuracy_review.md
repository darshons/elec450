# Accuracy and completeness review

The 500-run study and primary numerical conclusions are verified against the native OMPL results. The report and archive were corrected during this review; the raw experiments, RTP, collision model, configurations and Part 1 assets were not changed.

## Important correction

All 478 exact paths passed OMPL's prescribed-resolution `correct_solution` check. **14 failed `correct_solution_strict`**, which the installed OMPL 2.0.2 Benchmark source implements by multiplying the valid-segment count factor by four, checking the raw path, and restoring the factor. This happens after solve-time measurement; it does not change the planning or custom clearance-sampling resolution.

The 14 failures comprise four RRT and ten RRT-Connect paths. Six are RRT-Connect at range .2. They are still exact goal-reaching solutions under the prescribed validator, so the primary counts and exact-only metrics remain as required. The report now explicitly separates exact goal reaching from collision validity at finer sampling. Positive sampled minimum clearance is not a continuous collision certificate.

`summary.csv` and `per_query.csv` now include separate exact/strict-pass and exact/strict-fail counts. `strict_sensitivity.csv` shows secondary quality medians after excluding the 14 finer-check failures. That subset changes the shortest pooled median to RRT bias .05, range 1: 4.11743 rad from 23 retained paths. The fastest setting (RRT-Connect range 2.5, all 25 strict-pass), smoothest setting (PRM k=10), and greatest sampled-clearance setting (RTP bias .05) remain unchanged. Neither check establishes statistical significance or continuous collision freedom.

## Checks completed

- Independent standard-library recalculation of all means, medians, quartiles and 90th percentiles in the 20-setting and 100-query-group exports, with all-run time and exact-only path quality.
- All 500 run-level CSV rows checked against native database columns; exact and approximate enum/flag semantics checked separately.
- All 20 report-table rows, success counts/percentages, and every within-planner/global metric winner checked against the database.
- Complete 20 settings × 5 queries × 5 repetitions; final outcomes 478 exact and 22 approximate. Separate pilot: 86 exact and 14 approximate over 100 runs.
- Native logs reconverted into a fresh database; every row of all five data tables matches the canonical database exactly.
- Source hashes, planner parameters and query endpoints checked; prescribed resolution .005 and raw unsimplified-path workflow retained.
- Representative candidate selection checked. Length, smoothness and prescribed sampled minimum clearance independently recomputed from each saved JSON. All four have positive sampled clearance at .002 rad spacing; this remains a sampled check.
- The submission archive extracted to a new temporary directory and built successfully in the course environment. Its native support checks passed (7 passed, one intentionally skipped placeholder check); benchmark-conversion checks passed (4); lifecycle check passed, including 25 clear/reuse cycles and balanced state allocation.
- The updated report remains five pages. Revised pages were rendered and visually inspected. Planner Arena PDFs, screenshots and database upload hash are retained. The underlying benchmark database is unchanged.
- Part 1 sources, custom scenes, saved RTP paths, images and short report remain preserved. The original root-level Part 1 zip was not replaced.

## Reproducibility corrections

The analyzer now accepts `--fig-dir`, so reproduction plots can be written separately from the canonical report figures. The README commands use that option. Plot/table trial-count labels and query-heatmap scaling now derive from `--expected-repetitions`; a separate pilot-analysis run verified five total runs per setting rather than incorrectly labeling them as 25.

Run the independent audit without rebuilding the native extension:

```bash
micromamba activate ompl_env
python tests/check_part2.py
```

The five audit tests passed; detailed logs are `reverify_data.txt`, `reverify_support.txt`, `reverify_benchmark.txt` and `reverify_lifecycle.txt`. No additional full benchmark study was needed or performed during this review.

Winson confirmed the work split: Darshon handles RTP implementation/debugging and performance analysis; Winson handles custom scenes, visualization, benchmark configuration and experiments; the write-ups, testing and submissions are shared. The final report and README now reflect that split. Existing hours and difficulty ratings remain estimates for the partners to review. Fresh package compilation was checked on macOS ARM64 in the installed course environment; earlier Linux build evidence remains historical, not a new Linux verification from this review.
