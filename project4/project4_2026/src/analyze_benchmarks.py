"""Summarize results/benchmark/benchmark.db: CSV table and figures for the final report.

Success = exact solution within the time limit. Path length is OMPL's PathControl
length, i.e. total trajectory duration in seconds. Tree nodes = graph_states.
"""
import argparse
import csv
import sqlite3
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser()
p.add_argument('--db', type=Path, default=ROOT / 'results/benchmark/benchmark.db')
a = p.parse_args()

con = sqlite3.connect(a.db)
# Experiments are identified by the seed given in src/run_benchmarks.sh. The pendulum study reruns RRT,
# so it is kept apart; car study variants have distinct planner names and join the car experiment.
GROUP = {1001: 'pendulum', 2001: 'car', 3001: 'pendulum_rg', 4001: 'car_rg', 5005: 'pendulum_t5', 5010: 'pendulum_t10'}
GROUP = {str(k): v for k, v in GROUP.items()}  # the database stores seeds as text
rows = con.execute("""select e.seed, p.name, r.solved and not r.approximate_solution, r.time,
                      r.solution_length, r.graph_states
                      from runs r join plannerConfigs p on p.id=r.plannerid
                      join experiments e on e.id=r.experimentid""").fetchall()
limits = {GROUP[seed]: t for seed, t in con.execute('select seed, timelimit from experiments')}
data = {}
for seed, planner, ok, t, length, states in rows:
    exp = GROUP[seed]
    planner = planner.removeprefix('control_')
    d = data.setdefault((exp, planner), {'ok': [], 'time': [], 'length': [], 'states': []})
    d['ok'].append(bool(ok)); d['time'].append(t); d['states'].append(states)
    d['length'].append(length if ok else np.nan)

summary = []
for (exp, planner), d in sorted(data.items()):
    ok = np.array(d['ok']); t = np.array(d['time']); L = np.array(d['length']); n = np.array(d['states'])
    summary.append({'system': exp, 'planner': planner, 'runs': len(ok), 'success_rate': ok.mean(),
                    'time_median': np.median(t), 'time_mean': t.mean(),
                    'length_median': np.nanmedian(L) if ok.any() else np.nan,
                    'length_mean': np.nanmean(L) if ok.any() else np.nan,
                    'states_median': np.median(n), 'states_mean': n.mean()})
out = a.db.parent / 'summary.csv'
with out.open('w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=summary[0].keys()); w.writeheader()
    for s in summary:
        w.writerow({k: (f'{v:.4g}' if isinstance(v, float) else v) for k, v in s.items()})
print(f'wrote {out}')
for s in summary:
    print(f"{s['system']:9s} {s['planner']:16s} n={s['runs']:2d} success={s['success_rate']:.2f} "
          f"time~{s['time_median']:.2f}s length~{s['length_median']:.2f}s nodes~{s['states_median']:.0f}")


def panels(axes, system, planners, labels=None):
    labels = labels or planners
    ds = [data[(system, q)] for q in planners]
    x = np.arange(len(planners))
    axes[0].bar(x, [100 * np.mean(d['ok']) for d in ds], color='#4c78a8')
    axes[0].set_ylim(0, 105); axes[0].set_ylabel('success (%)')
    for ax, key, ylabel in [(axes[1], 'time', 'time (s, log)'),
                            (axes[2], 'length', 'path duration (s), solved'),
                            (axes[3], 'states', 'tree nodes, all runs')]:
        vals = [np.array(d[key])[~np.isnan(np.array(d[key], dtype=float))] for d in ds]
        ax.boxplot(vals, positions=x, widths=0.6, showfliers=True,
                   medianprops={'color': '#e45756'}, flierprops={'markersize': 3})
        ax.set_ylabel(ylabel)
        if key == 'time':
            ax.axhline(limits[system], color='gray', ls=':', lw=1); ax.set_yscale('log')
            ax.yaxis.set_major_locator(matplotlib.ticker.FixedLocator([0.1, 0.3, 1, 3, 10, 30, 60]))
            ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f'{v:g}'))
            ax.yaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
    for ax in axes:
        ax.set_xticks(x, labels, rotation=45 if len(planners) > 3 else 0, ha='right' if len(planners) > 3 else 'center')
        ax.grid(axis='y', alpha=0.3)


main = ['RRT', 'KPIECE1', 'RG-RRT']
fig, ax = plt.subplots(2, 4, figsize=(12, 5.2), constrained_layout=True)
for row, system in enumerate(['pendulum', 'car']):
    panels(ax[row], system, main)
    ax[row, 0].set_title(f'{system} ({"torque 3, " if system == "pendulum" else ""}'
                         f'{len(data[(system, "RRT")]["ok"])} runs, {limits[system]:.0f} s limit)', loc='left', fontsize=10)
fig.savefig(ROOT / 'fig/bench_main.pdf'); fig.savefig(ROOT / 'fig/bench_main.png', dpi=150)

study = ['RRT', 'RG-RRT_3x5', 'RG-RRT_5x5', 'RG-RRT', 'RG-RRT_21x5',
         'RG-RRT_11x1', 'RG-RRT_11x2', 'RG-RRT_11x10', 'RG-RRT_11x20']
study_labels = ['RRT', '3×5', '5×5', '11×5', '21×5', '11×1', '11×2', '11×10', '11×20']
car_study = ['RRT', 'RG-RRT_3x5', 'RG-RRT_11x5', 'RG-RRT_11x2', 'RG-RRT_11x10']
car_labels = ['RRT', '3×5', '11×5', '11×2', '11×10']
# The 11-control, 5-step pendulum variant is the main experiment's RG-RRT.
fig, ax = plt.subplots(2, 4, figsize=(13, 5.8), constrained_layout=True)
present = [q for q in study if ('pendulum_rg', q) in data or (q == 'RG-RRT' and ('pendulum', q) in data)]
for q in present:
    if ('pendulum_rg', q) not in data:
        data[('pendulum_rg', q)] = data[('pendulum', q)]
panels(ax[0], 'pendulum_rg', present, [study_labels[study.index(q)] for q in present])
ax[0, 0].set_title('pendulum, torque 3, 30 runs: RG-RRT with (reachable-set controls)×(steps of 0.02 s)', loc='left', fontsize=10)
car_present = [q for q in car_study if ('car_rg', q) in data]
panels(ax[1], 'car_rg', car_present, [car_labels[car_study.index(q)] for q in car_present])
ax[1, 0].set_title('car, 20 runs, all variants run concurrently: (controls)×(steps)', loc='left', fontsize=10)
fig.savefig(ROOT / 'fig/bench_rg_study.pdf'); fig.savefig(ROOT / 'fig/bench_rg_study.png', dpi=150)
print('wrote fig/bench_main.* and fig/bench_rg_study.*')

# LaTeX table for the report: medians over all runs (time, nodes) and over successful runs (length).
by = {(s['system'], s['planner']): s for s in summary}
names = {'RG-RRT': 'RG-RRT (11 ctrl, 5 steps)'}
def tex_rows(keys, label):
    lines = []
    for system, planner in keys:
        # The pendulum 11-control, 5-step variant comes from the main experiment.
        s = by.get((system, planner)) or (by.get(('pendulum', planner)) if system == 'pendulum_rg' else None)
        if s is None:
            continue
        length = '---' if np.isnan(s['length_median']) else f"{s['length_median']:.2f}"
        lines.append(f"{label(system, planner)} & {s['runs']} & {100 * s['success_rate']:.0f}\\% & "
                     f"{s['time_median']:.2f} & {s['time_mean']:.2f} & {length} & "
                     f"{s['states_median']:.0f} \\\\")
    return lines
head = [r'\begin{tabular}{lrrrrrr}\toprule',
        r'System / planner & runs & success & \multicolumn{2}{c}{time (s): median, mean} & duration (s) & tree nodes \\ \midrule']
body = tex_rows([(s, q) for s in ['pendulum', 'car'] for q in main],
                lambda s, q: f"{s.capitalize()} {names.get(q, q)}")
(ROOT / 'report/bench_table.tex').write_text('\n'.join(head + body + [r'\bottomrule\end{tabular}']) + '\n')
lab = lambda s, q: (('Pendulum ' if s == 'pendulum_rg' else 'Car ') +
                    (q if q == 'RRT' else 'RG-RRT ' + (q.split('_')[1].replace('x', ' ctrl, ') + ' steps' if '_' in q
                                                       else '11 ctrl, 5 steps$^*$'))).replace(' 1 steps', ' 1 step')
body = tex_rows([('pendulum_rg', q) for q in present] + [('car_rg', q) for q in car_present], lab)
(ROOT / 'report/rg_table.tex').write_text('\n'.join(head + body + [r'\bottomrule\end{tabular}']) + '\n')
print('wrote report/bench_table.tex and report/rg_table.tex')

# Torque comparison table: 30 runs per planner and torque bound.
lines = [r'\begin{tabular}{lccc}\toprule',
         r'Median over 30 runs & $\tau_{\max}=3$ & $\tau_{\max}=5$ & $\tau_{\max}=10$ \\ \midrule']
for planner in main:
    for key, label in [('length_median', 'path duration (s)'), ('time_median', 'computation time (s)')]:
        cells = [f"{by[(g, planner)][key]:.2f}" for g in ['pendulum', 'pendulum_t5', 'pendulum_t10']]
        lines.append(f"{planner} {label} & " + ' & '.join(cells) + r' \\')
lines.append(r'\bottomrule\end{tabular}')
(ROOT / 'report/torque_table.tex').write_text('\n'.join(lines) + '\n')
print('wrote report/torque_table.tex')
