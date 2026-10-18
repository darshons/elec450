"""Independent, read-only audit of the fresh study, exports and report table.

Run from project3_2026 with the course environment active. No planner extension
is required. Uses standard-library statistics rather than the analysis code.
"""
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import sqlite3
import statistics
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
METRICS = ('time', 'solution_length', 'solution_smoothness', 'minimum_clearance')


def csv_rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def percentile(values, fraction):
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lo, hi = math.floor(position), math.ceil(position)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (position - lo)


class Part2Audit(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.con = sqlite3.connect((ROOT / 'results/benchmark.db').as_uri() + '?mode=ro', uri=True)
        cls.addClassCleanup(cls.con.close)
        cls.con.row_factory = sqlite3.Row
        cls.rows = [dict(r) for r in cls.con.execute('''SELECT r.*, p.name AS config,
            e.name AS experiment FROM runs r JOIN plannerConfigs p ON p.id=r.plannerid
            JOIN experiments e ON e.id=r.experimentid ORDER BY r.id''')]
        for r in cls.rows:
            r['config'] = r['config'].removeprefix('geometric_')
            r['query'] = r['experiment'].split('_')[1]
        cls.summary = csv_rows(ROOT / 'results/part2/summary.csv')
        cls.per_query = csv_rows(ROOT / 'results/part2/per_query.csv')

    def test_matrix_statuses_sources_and_strict_checks(self):
        self.assertEqual(self.con.execute('PRAGMA integrity_check').fetchone()[0], 'ok')
        self.assertEqual(len(self.rows), 500)
        groups = collections.Counter((r['config'], r['query']) for r in self.rows)
        self.assertEqual(len(groups), 100)
        self.assertEqual(set(groups.values()), {5})
        self.assertEqual(collections.Counter(r['status'] for r in self.rows), {6:478, 5:22})
        enums = dict(self.con.execute("SELECT value,description FROM enums WHERE name='status'"))
        self.assertEqual(enums[6], 'Exact solution')
        self.assertEqual(enums[5], 'Approximate solution')
        for r in self.rows:
            self.assertEqual(bool(r['solved']), r['status'] == 6)
            self.assertEqual(bool(r['approximate_solution']), r['status'] == 5)
            self.assertEqual(r['correct_solution'], 1)
        strict = [r for r in self.rows if r['status'] == 6 and not r['correct_solution_strict']]
        self.assertEqual(len(strict), 14)
        self.assertEqual(collections.Counter(r['config'].split('_')[0] for r in strict),
                         {'RRT':4, 'RRTConnect':10})
        self.assertEqual(sum(r['config'] == 'RRTConnect_r0.2' for r in strict), 6)
        for e in self.con.execute('SELECT * FROM experiments'):
            self.assertEqual((e['runcount'], e['timelimit'], e['seed']), (5, 5., '18002'))
        with sqlite3.connect(ROOT / 'results/pilot/benchmark.db') as c:
            self.assertEqual(dict(c.execute('SELECT status,count(*) FROM runs GROUP BY status')), {6:86,5:14})
            self.assertTrue(all(row == (1, 1., '18001') for row in c.execute(
                'SELECT runcount,timelimit,seed FROM experiments')))
        env = json.loads((ROOT / 'results/part2/environment.json').read_text())
        for file, digest in env['file_sha256'].items():
            self.assertEqual(hashlib.sha256((ROOT / file).read_bytes()).hexdigest(), digest, file)

    def test_all_exported_summary_numbers_and_raw_rows(self):
        self.assertEqual(len(self.summary), 20)
        self.assertEqual(len(self.per_query), 100)
        for s in self.summary + self.per_query:
            runs = [r for r in self.rows if r['config'] == s['config'] and
                    (s['query'] == 'all' or r['query'] == s['query'])]
            exact = [r for r in runs if r['status'] == 6]
            counts = dict(runs=len(runs), exact=len(exact),
                          approximate=sum(r['status'] == 5 for r in runs),
                          failed=sum(r['status'] not in (5,6) for r in runs),
                          exact_strict_passed=sum(r['correct_solution_strict'] == 1 for r in exact),
                          exact_strict_failed=sum(r['correct_solution_strict'] == 0 for r in exact))
            for key, value in counts.items():
                self.assertEqual(int(s[key]), value, (s['config'], s['query'], key))
            self.assertAlmostEqual(float(s['exact_percent']), 100 * len(exact) / len(runs))
            for metric in METRICS:
                values = [r[metric] for r in (runs if metric == 'time' else exact)]
                stats = dict(median=statistics.median(values), mean=statistics.mean(values),
                             q25=percentile(values,.25), q75=percentile(values,.75), p90=percentile(values,.9))
                for key, value in stats.items():
                    self.assertTrue(math.isclose(float(s[f'{metric}_{key}']), value, rel_tol=1e-12, abs_tol=1e-12),
                                    (s['config'], s['query'], metric, key))
        sensitivity = csv_rows(ROOT / 'results/part2/strict_sensitivity.csv')
        self.assertEqual(len(sensitivity), 20)
        for s in sensitivity:
            retained = [r for r in self.rows if r['config'] == s['config']
                        and r['status'] == 6 and r['correct_solution_strict'] == 1]
            self.assertEqual(int(s['exact_strict_passed']),len(retained))
            for metric in METRICS[1:]:
                self.assertAlmostEqual(float(s[metric+'_strict_pass_median']),
                                       statistics.median(r[metric] for r in retained), places=10)
        raw = csv_rows(ROOT / 'results/part2/runs.csv')
        self.assertEqual(len(raw), 500)
        columns = [r['name'] for r in self.con.execute('PRAGMA table_info(runs)')]
        for original, exported in zip(self.rows, raw):
            for column in columns:
                expected = '' if original[column] is None else str(original[column])
                self.assertEqual(exported[column], expected, (original['id'], column))

    def test_report_table_and_every_metric_winner(self):
        summaries = {r['config']:r for r in self.summary}
        lines = [line for line in (ROOT / 'results/part2/table.tex').read_text().splitlines()
                 if re.match(r'^(RTP|RRT|PRM)', line)]
        self.assertEqual(len(lines), 20)
        for line in lines:
            cells = [c.strip() for c in line.removesuffix(r'\\').split('&')]
            s = summaries[cells[0].replace(r'\_', '_')]
            self.assertEqual(cells[1], f"{s['exact']}/25 ({float(s['exact_percent']):.0f}\\%)")
            self.assertEqual(cells[2], f"{s['approximate']}/{s['failed']}")
            keys = ['time_median','time_mean','time_p90','solution_length_median',
                    'solution_smoothness_median','minimum_clearance_median']
            for displayed, key, digits, factor in zip(cells[3:],keys,[3,3,3,2,2,2],[1,1,1,1,1,100]):
                self.assertEqual(displayed, f'{float(s[key])*factor:.{digits}f}')
        winners = json.loads((ROOT / 'results/part2/winners.json').read_text())
        for family, metrics in winners.items():
            candidates = [s for s in self.summary if family == 'all' or s['config'].split('_')[0] == family]
            for metric, winner in metrics.items():
                best = (max if metric == 'minimum_clearance' else min)(
                    candidates, key=lambda s: float(s[metric+'_median']))
                self.assertEqual(winner['config'],best['config'])
                self.assertEqual(winner['exact'],int(best['exact']))
                self.assertEqual(winner[metric+'_median'],float(best[metric+'_median']))

    def test_native_log_reconversion_and_canonical_copy(self):
        self.assertEqual((ROOT / 'results/benchmark.db').read_bytes(),
                         (ROOT / 'results/part2/benchmark.db').read_bytes())
        converter = shutil.which('ompl_benchmark_statistics.py')
        self.assertIsNotNone(converter, 'Activate the course environment for the OMPL converter')
        with tempfile.TemporaryDirectory(prefix='part2-audit-') as tmp:
            target = Path(tmp) / 'reconverted.db'
            logs = sorted((ROOT / 'results/part2/benchmark_logs').glob('*.log'))
            self.assertEqual(len(logs), 5)
            subprocess.run([sys.executable,converter,'-d',str(target),*map(str,logs)],
                           check=True,capture_output=True,text=True)
            with sqlite3.connect(target) as c:
                for table in ['runs','experiments','plannerConfigs','enums','progress']:
                    self.assertEqual([tuple(r) for r in self.con.execute(f'SELECT * FROM {table}')],
                                     c.execute(f'SELECT * FROM {table}').fetchall(), table)

    def test_representative_selection_and_independent_path_metrics(self):
        sys.path.insert(0, str(ROOT))
        from check_review import Geometry
        from arm_scene import load_scene
        geo = Geometry()
        scene = load_scene(ROOT / 'resources/scenes/clutter.yaml')
        selection = json.loads((ROOT / 'results/part2/paths/selection.json').read_text())
        self.assertEqual(len(selection), 4)
        for s in selection:
            exact = [c for c in s['candidates'] if c['status'] == 'exact']
            chosen = min(exact,key=lambda c:abs(c['length']-s['benchmark_query1_median_length']))
            self.assertEqual(s['chosen_seed'],chosen['seed'])
            result = json.loads((ROOT / s['saved']).read_text())
            self.assertEqual(result['scene'],scene)
            self.assertEqual(result['status'],'exact')
            self.assertTrue(geo.bounded(result['path']))
            self.assertEqual(result['path'][0],result['query']['start'])
            self.assertLess(math.dist(result['path'][-1],result['query']['goal']),1e-6)
            path = result['path']
            length = sum(math.dist(a,b) for a,b in zip(path,path[1:]))
            self.assertAlmostEqual(length,result['length'],places=10)
            smoothness = 0.
            for a,b,c in zip(path,path[1:],path[2:]):
                x,y,z = math.dist(a,b),math.dist(b,c),math.dist(a,c)
                cosine = (x*x+y*y-z*z)/(2*x*y)
                if -1 < cosine < 1:
                    smoothness += (2*(math.pi-math.acos(cosine))/(x+y))**2
            self.assertAlmostEqual(smoothness,result['smoothness'],places=10)
            self.assertAlmostEqual(geo.path_minimum(path,scene,geo.step),result['min_clearance'],places=10)
            self.assertGreater(geo.path_minimum(path,scene,.002),0)
            for suffix in ['_playback.png','_playback_mid.png']:
                self.assertTrue((ROOT / 'results/part2/paths' / (s['planner']+suffix)).exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)
