#!/bin/bash
# Run all final benchmarks in parallel (one process per experiment) and build benchmark.db.
# Usage: bash src/run_benchmarks.sh [output_dir]   (run from project4_2026 after make)
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=${1:-results/benchmark}
if [[ -e "$OUT" ]]; then
    echo "Choose a new output directory; refusing to overwrite benchmark evidence: $OUT" >&2
    exit 1
fi
mkdir -p "$OUT"
pids=()
P=./Project4Pendulum; C=./Project4Car
# Exercise 4: RRT vs KPIECE1 vs RG-RRT (11 controls, 5 steps = 0.1 s), torque 3, >= 20 runs each.
$P --benchmark --torque 3 --runs 30 --time 30 --seed 1001 --log "$OUT/pendulum.log" > "$OUT/pendulum.out" 2>&1 &
pids+=("$!")
for planner in RRT KPIECE1 RG-RRT; do
    $C --benchmark --planners $planner --runs 25 --time 60 --seed 2001 --log "$OUT/car_$planner.log" > "$OUT/car_$planner.out" 2>&1 &
    pids+=("$!")
done
# RG-RRT trade-off study: number of reachable-set controls and length of the reachable-set time period.
$P --benchmark --torque 3 --runs 30 --time 30 --seed 3001 --log "$OUT/pendulum_rg_study.log" \
   --planners RRT,RG-RRT:3:5,RG-RRT:5:5,RG-RRT:21:5,RG-RRT:11:1,RG-RRT:11:2,RG-RRT:11:10,RG-RRT:11:20 \
   > "$OUT/pendulum_rg_study.out" 2>&1 &
pids+=("$!")
# Car study: one process per variant; CPU load can change as processes finish.
for planner in RRT RG-RRT:11:5 RG-RRT:3:5 RG-RRT:11:2 RG-RRT:11:10; do
    $C --benchmark --planners $planner --runs 20 --time 60 --seed 4001 --log "$OUT/car_study_${planner//:/_}.log" \
       > "$OUT/car_study_${planner//:/_}.out" 2>&1 &
    pids+=("$!")
done
# Torque comparison (report section on torque 3/5/10): the same three planners at torques 5 and 10.
for torque in 5 10; do
    $P --benchmark --torque $torque --runs 30 --time 30 --seed 50$(printf %02d $torque) --log "$OUT/pendulum_torque$torque.log" \
       > "$OUT/pendulum_torque$torque.out" 2>&1 &
    pids+=("$!")
done
failed=0
for pid in "${pids[@]}"; do
    if ! wait "$pid"; then failed=1; fi
done
if (( failed )); then
    echo "A benchmark process failed; inspect the .out files before building the database." >&2
    exit 1
fi
python "$(command -v ompl_benchmark_statistics.py)" -d "$OUT/benchmark.db" "$OUT"/*.log
