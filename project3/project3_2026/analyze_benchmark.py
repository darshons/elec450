"""Summarize results/benchmark.db and make the report plots.

python analyze_benchmark.py --db results/benchmark.db --tag main
Writes results/summary_<tag>.csv and fig/bench_<tag>_*.pdf.
Path quality (length, smoothness, clearance) only uses exact solutions.
"""
import argparse
import csv
from pathlib import Path
import sqlite3

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

COLORS = {"RTP": "#2a78d6", "RRT": "#eb6834", "RRTConnect": "#1baf7a", "PRM": "#eda100"}
METRICS = [
    ("time", "planning time (s)", False),
    ("solution_length", "path length (rad)", True),
    ("solution_smoothness", "smoothness (lower is smoother)", True),
    ("minimum_clearance", "minimum clearance (m)", True),
]


def family(name):
    return name.split("_")[0]


def label(name):
    # RRT_gb0.05_r0.4 -> "gb0.05 r0.4", RTP_gb0.1 -> "gb0.1"
    return " ".join(name.split("_")[1:])


def load(db):
    con = sqlite3.connect(db)
    rows = con.execute("""
        SELECT plannerConfigs.name, experiments.name, experiments.timelimit, runs.time,
               runs.solved, runs.approximate_solution, runs.solution_length,
               runs.solution_smoothness, runs.minimum_clearance, runs.graph_states
        FROM runs JOIN plannerConfigs ON runs.plannerid = plannerConfigs.id
                  JOIN experiments ON runs.experimentid = experiments.id""").fetchall()
    data = {}
    for planner, experiment, limit, t, solved, approx, length, smooth, clear, states in rows:
        planner = planner.replace("geometric_", "")
        exact = bool(solved) and not bool(approx)
        data.setdefault(planner, []).append(dict(
            query=experiment.split("_")[1], limit=limit, time=t, exact=exact,
            solution_length=length, solution_smoothness=smooth,
            minimum_clearance=clear, graph_states=states))
    order = ["RTP", "RRT", "RRTConnect", "PRM"]
    def key(n):
        nums = [float(part.lstrip("gbrk")) for part in n.split("_")[1:]]
        return (order.index(family(n)), nums[::-1])
    names = sorted(data, key=key)
    return names, data


def summarize(names, data):
    table = []
    for name in names:
        runs = data[name]
        ex = [r for r in runs if r["exact"]]
        row = {"config": name, "runs": len(runs), "exact": len(ex)}
        for key, _, exact_only in METRICS + [("graph_states", "", True)]:
            vals = [r[key] for r in (ex if exact_only else runs) if r[key] is not None]
            row[key + "_median"] = float(np.median(vals)) if vals else float("nan")
            row[key + "_mean"] = float(np.mean(vals)) if vals else float("nan")
        table.append(row)
    return table


def boxplots(names, data, out, title):
    fig, axes = plt.subplots(2, 2, figsize=(11, 6.6))
    for ax, (key, ylabel, exact_only) in zip(axes.flat, METRICS):
        vals = [[r[key] for r in data[n] if (r["exact"] or not exact_only) and r[key] is not None]
                for n in names]
        bp = ax.boxplot(vals, widths=0.6, patch_artist=True, showfliers=True,
                        medianprops=dict(color="#0b0b0b", linewidth=1.5),
                        flierprops=dict(marker="o", markersize=3, markeredgecolor="#52514e"))
        for patch, n in zip(bp["boxes"], names):
            patch.set_facecolor(COLORS[family(n)])
            patch.set_edgecolor("#52514e")
            patch.set_linewidth(0.8)
        ax.set_xticks(range(1, len(names) + 1))
        ax.set_xticklabels([label(n) for n in names], rotation=70, fontsize=7)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.tick_params(axis="y", labelsize=8)
        ax.grid(axis="y", color="#e4e3df", linewidth=0.6)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        if key in ("time", "solution_smoothness"):
            ax.set_yscale("log")
    handles = [plt.Rectangle((0, 0), 1, 1, facecolor=c, edgecolor="#52514e") for c in COLORS.values()]
    fig.legend(handles, [f"{k}" for k in COLORS], loc="upper center", ncol=4, frameon=False,
               fontsize=9, title=title, title_fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(out)
    plt.close(fig)


def success_plot(names, data, out):
    fig, ax = plt.subplots(figsize=(11, 2.4))
    rates = [100.0 * sum(r["exact"] for r in data[n]) / len(data[n]) for n in names]
    ax.bar(range(len(names)), rates, width=0.7, color=[COLORS[family(n)] for n in names],
           edgecolor="white", linewidth=1)
    for i, v in enumerate(rates):
        ax.text(i, v + 2, f"{v:.0f}", ha="center", fontsize=7, color="#52514e")
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels([label(n) for n in names], rotation=70, fontsize=7)
    ax.set_ylim(0, 112)
    ax.set_ylabel("exact solutions (%)", fontsize=9)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    handles = [plt.Rectangle((0, 0), 1, 1, facecolor=c) for c in COLORS.values()]
    fig.legend(handles, COLORS.keys(), ncol=4, frameon=False, fontsize=8, loc="upper center")
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    fig.savefig(out)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", type=Path, default=Path("results/benchmark.db"))
    parser.add_argument("--tag", default="main")
    parser.add_argument("--title", default="clutter scene, 5 queries x 5 trials per setting")
    args = parser.parse_args()
    names, data = load(args.db)
    table = summarize(names, data)
    out_csv = Path("results") / f"summary_{args.tag}.csv"
    with out_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(table[0]))
        writer.writeheader()
        writer.writerows(table)
    for row in table:
        print(f"{row['config']:22s} exact {row['exact']:2d}/{row['runs']:2d}  "
              f"t {row['time_median']:.3f}/{row['time_mean']:.3f}  "
              f"len {row['solution_length_median']:.2f}  "
              f"sm {row['solution_smoothness_median']:.3f}  "
              f"clr {row['minimum_clearance_median']:.4f}  "
              f"V {row['graph_states_median']:.0f}")
    queries = sorted({r["query"] for runs in data.values() for r in runs})
    print("exact per query:", queries)
    for name in names:
        print(f"  {name:22s}", [sum(r["exact"] for r in data[name] if r["query"] == q) for q in queries])
    Path("fig").mkdir(exist_ok=True)
    boxplots(names, data, f"fig/bench_{args.tag}_metrics.pdf", args.title)
    success_plot(names, data, f"fig/bench_{args.tag}_success.pdf")


if __name__ == "__main__":
    main()
