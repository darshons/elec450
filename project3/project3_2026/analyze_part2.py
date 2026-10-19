"""Check and export the Part 2 benchmark results and make the report figures.

Run from the project directory: python analyze_part2.py
"""
import argparse
import csv
import json
from pathlib import Path
import re
import sqlite3

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

METRICS = ['time', 'solution_length', 'solution_smoothness', 'minimum_clearance']
SELECTED = ['RTP_gb0.2', 'RRT_gb0.05_r1.0', 'RRTConnect_r1.0', 'PRM_k10']
COLORS = ['#2473ad', '#ce5b2b', '#25815b', '#a97800']


def write_csv(path, rows):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--db', type=Path, default=Path('results/benchmark.db'))
    parser.add_argument('--out', type=Path, default=Path('results/part2'))
    parser.add_argument('--fig-dir', type=Path, default=Path('fig'),
                        help='Figure output directory (use a new one for reproductions)')
    parser.add_argument('--expected-repetitions', type=int, default=5)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    args.fig_dir.mkdir(parents=True, exist_ok=True)
    total_runs = 5 * args.expected_repetitions
    con = sqlite3.connect(f'file:{args.db}?mode=ro', uri=True)
    con.row_factory = sqlite3.Row
    assert con.execute('PRAGMA integrity_check').fetchone()[0] == 'ok'
    enums = dict(con.execute("SELECT value,description FROM enums WHERE name='status'"))
    exact_code = next(k for k, v in enums.items() if v == 'Exact solution')
    approx_code = next(k for k, v in enums.items() if v == 'Approximate solution')
    rows = [dict(r) for r in con.execute('''
        SELECT r.*, p.name AS config, p.settings AS recorded_settings,
               e.name AS experiment, e.timelimit AS time_limit, e.seed AS experiment_seed
        FROM runs r JOIN plannerConfigs p ON p.id=r.plannerid
                    JOIN experiments e ON e.id=r.experimentid ORDER BY r.id''')]
    configs = json.loads(Path('configs/planners.json').read_text())['planners']
    names = [c['name'] for c in configs]
    experiments = [dict(r) for r in con.execute('SELECT * FROM experiments')]
    assert len(experiments) == 5
    assert {e['name'] for e in experiments} == {f'clutter_query{q}_group1' for q in range(1, 6)}
    assert all(e['runcount'] == args.expected_repetitions for e in experiments)
    assert len({e['timelimit'] for e in experiments}) == 1
    from arm_scene import load_scene
    queries = {q['name']: q for q in load_scene('resources/scenes/clutter.yaml')['queries']}
    for e in experiments:
        match = re.search(r'Start states:\s*RealVectorState \[([^]]+)\]\s*Goal state.*?'
                          r'state =\s*RealVectorState \[([^]]+)\]', e['setup'], re.S)
        assert match
        q = queries[e['name'].split('_')[1]]
        for values, key in zip(match.groups(), ['start', 'goal']):
            np.testing.assert_allclose([float(v) for v in values.split()], q[key], atol=1e-6, rtol=0)
    for p in con.execute('SELECT name,settings FROM plannerConfigs'):
        cfg = next(c for c in configs if c['name'] == p['name'].removeprefix('geometric_'))
        params = dict(re.findall(r'([\w.]+) = ([^\n;]+)', p['settings']))
        assert float(params['longest_valid_segment_fraction']) == .005
        for k, v in cfg['settings'].items():
            native = {'planner_range': 'range', 'max_neighbors': 'max_nearest_neighbors'}.get(k, k)
            assert float(params[native]) == v
    assert set(r['config'].removeprefix('geometric_') for r in rows) == set(names)
    trial_counts = {}
    for r in rows:
        r['config'] = r['config'].removeprefix('geometric_')
        r['query'] = next(q for q in r['experiment'].split('_') if q.startswith('query'))
        r['status_name'] = enums[r['status']]
        r['exact'] = r['status'] == exact_code
        r['approximate'] = r['status'] == approx_code
        # OMPL Benchmark's solved property denotes EXACT success, unlike
        # PlannerStatus's C++ boolean conversion, which also accepts approximate.
        assert bool(r['solved']) == r['exact']
        assert bool(r['approximate_solution']) == r['approximate']
        if r['exact']:
            assert all(r[k] is not None and np.isfinite(r[k]) for k in METRICS)
        key = (r['config'], r['query'])
        trial_counts[key] = trial_counts.get(key, 0) + 1
        r['trial_in_query'] = trial_counts[key]
    assert len(trial_counts) == len(names) * 5
    assert set(trial_counts.values()) == {args.expected_repetitions}
    write_csv(args.out / 'runs.csv', rows)
    (args.out / 'schema.sql').write_text('\n'.join(r[0] + ';' for r in con.execute(
        "SELECT sql FROM sqlite_master WHERE sql IS NOT NULL")))
    (args.out / 'status_enums.json').write_text(json.dumps(enums, indent=2) + '\n')
    (args.out / 'experiments.json').write_text(json.dumps(experiments, indent=2) + '\n')

    def summary(name, runs, query='all'):
        exact = [r for r in runs if r['exact']]
        d = dict(config=name, query=query, runs=len(runs), exact=len(exact),
                 exact_percent=100 * len(exact) / len(runs),
                 approximate=sum(r['approximate'] for r in runs),
                 failed=sum(not r['exact'] and not r['approximate'] for r in runs),
                 exact_strict_passed=sum(r['correct_solution_strict'] == 1 for r in exact),
                 exact_strict_failed=sum(r['correct_solution_strict'] == 0 for r in exact))
        for metric in METRICS:
            values = [r[metric] for r in (runs if metric == 'time' else exact)]
            for stat, func in [('median', np.median), ('mean', np.mean),
                               ('q25', lambda v: np.percentile(v, 25)),
                               ('q75', lambda v: np.percentile(v, 75)),
                               ('p90', lambda v: np.percentile(v, 90))]:
                d[f'{metric}_{stat}'] = float(func(values)) if values else None
        return d

    groups = {n: [r for r in rows if r['config'] == n] for n in names}
    summaries = [summary(n, groups[n]) for n in names]
    per_query = [summary(n, [r for r in groups[n] if r['query'] == f'query{q}'], f'query{q}')
                 for n in names for q in range(1, 6)]
    write_csv(args.out / 'summary.csv', summaries)
    write_csv(args.out / 'per_query.csv', per_query)
    strict_summaries = []
    for name in names:
        exact = [r for r in groups[name] if r['exact']]
        retained = [r for r in exact if r['correct_solution_strict'] == 1]
        s = dict(config=name, runs=len(groups[name]), exact=len(exact),
                 exact_strict_passed=len(retained), exact_strict_failed=len(exact)-len(retained))
        for metric in METRICS[1:]:
            s[metric+'_strict_pass_median'] = float(np.median([r[metric] for r in retained])) if retained else None
        strict_summaries.append(s)
    write_csv(args.out / 'strict_sensitivity.csv', strict_summaries)
    winners = {}
    for family in ['RTP', 'RRT', 'RRTConnect', 'PRM', 'all']:
        candidates = [s for s in summaries if family == 'all' or s['config'].split('_')[0] == family]
        winners[family] = {}
        for metric in METRICS:
            viable = [s for s in candidates if s[f'{metric}_median'] is not None]
            best = (max if metric == 'minimum_clearance' else min)(viable, key=lambda s: s[f'{metric}_median'])
            winners[family][metric] = {k: best[k] for k in ['config', 'exact', 'runs', f'{metric}_median']}
    (args.out / 'winners.json').write_text(json.dumps(winners, indent=2) + '\n')
    tex = []
    for s in summaries:
        name = s['config'].replace('_', r'\_')
        tex.append(f"{name} & {s['exact']}/{s['runs']} ({s['exact_percent']:.0f}\\%) & "
                   f"{s['approximate']}/{s['failed']} & {s['time_median']:.3f} & "
                   f"{s['time_mean']:.3f} & {s['time_p90']:.3f} & "
                   f"{s['solution_length_median']:.2f} & {s['solution_smoothness_median']:.2f} & "
                   f"{100*s['minimum_clearance_median']:.2f} \\\\")
    header = (r'\begin{tabular}{lrrrrrrrr}\toprule' + '\n' +
              r' & exact (\%) & A/F & \multicolumn{3}{c}{all-run time (s)} & \multicolumn{3}{c}{exact-path medians}\\' + '\n' +
              f'setting & of {total_runs} & & median & mean & p90 & rad & smooth. & cm' + r'\\\midrule' + '\n')
    (args.out / 'table.tex').write_text(header + '\n'.join(tex) + '\n' + r'\bottomrule\end{tabular}' + '\n')

    plt.rcParams.update({'font.size': 9, 'axes.spines.top': False, 'axes.spines.right': False})
    short = ['RTP b=.2', 'RRT b=.05 r=1', 'RRT-C r=1', 'PRM k=10']
    fig, axes = plt.subplots(2, 2, figsize=(8, 4.8), layout='constrained')
    labels = ['Solve time: all runs (s)', 'Exact path length (rad)',
              'Exact raw smoothness', 'Exact sampled minimum clearance (cm)']
    for ax, metric, ylabel in zip(axes.flat, METRICS, labels):
        for i, n in enumerate(SELECTED):
            values = np.array([r[metric] for r in groups[n] if metric == 'time' or r['exact']])
            if metric == 'minimum_clearance':
                values *= 100
            ax.boxplot([values], positions=[i], widths=.5, showfliers=False,
                       medianprops={'color': 'black'})
            ax.scatter(i + np.linspace(-.15, .15, len(values)), values, s=11,
                       color=COLORS[i], alpha=.65, zorder=3)
        ax.set_xticks(range(4), short, fontsize=8)
        ax.set_ylabel(ylabel)
        ax.grid(axis='y', alpha=.2)
        if metric in ['time', 'solution_smoothness']:
            ax.set_yscale('log')
    fig.savefig(args.fig_dir / 'part2_distributions.pdf')
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(7.7, 4.2), layout='constrained')
    counts = np.array([[next(s['exact'] for s in per_query if s['config'] == n and s['query'] == f'query{q}')
                        for q in range(1, 6)] for n in names])
    ax.imshow(counts, vmin=0, vmax=args.expected_repetitions, cmap='YlGn', aspect='auto')
    ax.set_yticks(range(len(names)), [n.replace('_', ' ') for n in names], fontsize=8)
    ax.set_xticks(range(5), [f'query {q}' for q in range(1, 6)])
    ax.xaxis.tick_top()
    for i in range(len(names)):
        for j in range(5):
            ax.text(j, i, str(counts[i, j]), ha='center', va='center', fontsize=8,
                    color='white' if counts[i, j] >= .8 * args.expected_repetitions else 'black')
    fig.savefig(args.fig_dir / 'part2_queries.pdf')
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(7, 2.8), layout='constrained')
    for n, color, lab in zip(SELECTED, COLORS, short):
        times = sorted(r['time'] for r in groups[n] if r['exact'])
        ax.step([.00001] + times + [max(r['time_limit'] for r in rows)],
                [0] + list(np.arange(1, len(times)+1)/total_runs) + [len(times)/total_runs],
                where='post', label=f'{lab}: {len(times)}/{total_runs}', color=color)
    ax.set(xscale='log', xlim=(.001, max(r['time_limit'] for r in rows)*1.05),
           ylim=(0, 1.05), xlabel='OMPL solve time (s)', ylabel=f'Fraction of all {total_runs} runs exact')
    ax.legend(fontsize=8, loc='lower right')
    ax.grid(alpha=.2)
    fig.savefig(args.fig_dir / 'part2_exact_ecdf.pdf')
    plt.close(fig)
    fig, axes = plt.subplots(4, 4, figsize=(13, 10), layout='constrained')
    for i, (family, color) in enumerate(zip(['RTP', 'RRT', 'RRTConnect', 'PRM'], COLORS)):
        ss = [s for s in summaries if s['config'].split('_')[0] == family]
        for j, (metric, ylabel) in enumerate(zip(METRICS, labels)):
            ax = axes[i, j]
            factor = 100 if metric == 'minimum_clearance' else 1
            med = np.array([s[f'{metric}_median'] for s in ss]) * factor
            lo = np.array([s[f'{metric}_q25'] for s in ss]) * factor
            hi = np.array([s[f'{metric}_q75'] for s in ss]) * factor
            ax.errorbar(range(len(ss)), med, yerr=[med-lo, hi-med], fmt='o', capsize=3, color=color)
            ticks = [s['config'].split('_', 1)[1].replace('_', '\n') for s in ss]
            ax.set_xticks(range(len(ss)), ticks, rotation=35, fontsize=7)
            ax.set_title(f'{family}: {ylabel}', fontsize=9)
            ax.grid(axis='y', alpha=.2)
            if metric in ['time', 'solution_smoothness']:
                ax.set_yscale('log')
    fig.suptitle(f'Parameter effects: median and interquartile range; time uses all {total_runs} runs, quality exact only (counts in summary.csv)', fontsize=11)
    fig.savefig(args.fig_dir / 'part2_parameter_effects.pdf')
    plt.close(fig)
    print(json.dumps({'runs': len(rows), 'status_counts': {v: sum(r['status_name'] == v for r in rows)
                      for v in set(r['status_name'] for r in rows)}, 'winners': winners}, indent=2))


if __name__ == '__main__':
    main()
