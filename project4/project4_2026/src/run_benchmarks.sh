#!/bin/bash
# Run all final benchmarks in parallel (one process per experiment) and build benchmark.db.
# Usage: bash src/run_benchmarks.sh [output_dir]   (run from project4_2026 after make)
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=${1:-results/benchmark}
mkdir -p "$OUT"
P=./Project4Pendulum; C=./Project4Car
# Exercise 4: RRT vs KPIECE1 vs RG-RRT (11 controls, 5 steps = 0.1 s), torque 3, >= 20 runs each.
$P --benchmark --torque 3 --runs 30 --time 30 --seed 1001 --log "$OUT/pendulum.log" > "$OUT/pendulum.out" 2>&1 &
for planner in RRT KPIECE1 RG-RRT; do
    $C --benchmark --planners $planner --runs 25 --time 60 --seed 2001 --log "$OUT/car_$planner.log" > "$OUT/car_$planner.out" 2>&1 &
done
# RG-RRT trade-off study: number of reachable-set controls and length of the reachable-set time period.
$P --benchmark --torque 3 --runs 30 --time 30 --seed 3001 --log "$OUT/pendulum_rg_study.log" \
   --planners RRT,RG-RRT:3:5,RG-RRT:5:5,RG-RRT:21:5,RG-RRT:11:1,RG-RRT:11:2,RG-RRT:11:10,RG-RRT:11:20 \
   > "$OUT/pendulum_rg_study.out" 2>&1 &
# Car study: one process per variant, all started together so every variant sees the same CPU load.
for planner in RRT RG-RRT:11:5 RG-RRT:3:5 RG-RRT:11:2 RG-RRT:11:10; do
    $C --benchmark --planners $planner --runs 20 --time 60 --seed 4001 --log "$OUT/car_study_${planner//:/_}.log" \
       > "$OUT/car_study_${planner//:/_}.out" 2>&1 &
done
# Torque comparison (report section on torque 3/5/10): the same three planners at torques 5 and 10.
for torque in 5 10; do
    $P --benchmark --torque $torque --runs 30 --time 30 --seed 50$(printf %02d $torque) --log "$OUT/pendulum_torque$torque.log" \
       > "$OUT/pendulum_torque$torque.out" 2>&1 &
done
wait
python "$(command -v ompl_benchmark_statistics.py)" -d "$OUT/benchmark.db" "$OUT"/*.log
