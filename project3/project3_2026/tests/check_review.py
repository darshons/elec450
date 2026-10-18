"""Read-only audit of Part 1 and archived October 14 benchmark/report numbers.

Run with the course environment: python -B tests/check_review.py
Optional: --native-dir DIRECTORY --smoke checks a separately built extension.
JSON goes to stdout; the audit never regenerates or overwrites source artifacts.
"""
import argparse
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import sqlite3
import statistics
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from arm_scene import PANDA_URDF, load_panda, load_scene, scene_obstacles


class Geometry:
    """Independent NumPy FK and signed sphere/OBB distance calculation."""
    def __init__(self):
        self.robot = load_panda()
        self.links = np.array([s[0] for s in self.robot.spheres])
        self.centers = np.array([s[1] for s in self.robot.spheres])
        self.radii = np.array([s[2] for s in self.robot.spheres])
        self.link_count = max(max(j[:2]) for j in self.robot.joints) + 1
        self.origins = []
        for _, _, _, xyz, rpy, _ in self.robot.joints:
            transform = np.eye(4)
            transform[:3, 3] = xyz
            transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
            self.origins.append(transform)
        self.step = 0.005 * np.linalg.norm(np.array(self.robot.upper) - self.robot.lower)

    def fk(self, qs):
        qs = np.atleast_2d(qs)
        transforms = np.broadcast_to(np.eye(4), (len(qs), self.link_count, 4, 4)).copy()
        for joint, origin in zip(self.robot.joints, self.origins):
            parent, child, index, _, _, axis = joint
            transforms[:, child] = transforms[:, parent] @ origin
            if index >= 0:
                rotations = Rotation.from_rotvec(qs[:, index, None] * np.array(axis)).as_matrix()
                transforms[:, child, :3, :3] = transforms[:, child, :3, :3] @ rotations
        selected = transforms[:, self.links]
        return (np.einsum("nsij,sj->nsi", selected[:, :, :3, :3], self.centers)
                + selected[:, :, :3, 3])

    def clearance(self, qs, scene):
        positions = self.fk(qs)
        spheres, boxes = scene_obstacles(scene)
        result = np.full(len(positions), np.inf)
        for center, radius in spheres:
            distances = np.linalg.norm(positions - center, axis=2) - self.radii - radius
            result = np.minimum(result, distances.min(axis=1))
        for center, half, quaternion in boxes:
            local = (positions - center) @ Rotation.from_quat(quaternion).as_matrix()
            delta = np.abs(local) - half
            distances = (np.linalg.norm(np.maximum(delta, 0), axis=2)
                         + np.minimum(delta.max(axis=2), 0) - self.radii)
            result = np.minimum(result, distances.min(axis=1))
        return result

    def bounded(self, path):
        qs = np.asarray(path)
        return bool(np.all((qs >= self.robot.lower) & (qs <= self.robot.upper)))

    def path_minimum(self, path, scene, step):
        minimum = float(self.clearance(path, scene).min())
        for start, end in zip(path[:-1], path[1:]):
            count = max(1, math.ceil(math.dist(start, end) / step))
            minimum = min(minimum, float(self.clearance(np.linspace(start, end, count + 1), scene).min()))
        return minimum


def audit_artifacts():
    geo = Geometry()
    report = {"dense_step_rad": 0.002, "queries": [], "paths": [], "benchmarks": {}}
    with np.load(ROOT / "tests/fixtures/panda_fk_pinocchio.npz") as reference:
        assert hashlib.sha256(PANDA_URDF.read_bytes()).hexdigest() == str(reference["urdf_sha256"])
        error = float(np.max(np.abs(geo.fk(reference["q"]) - reference["spheres"][:, :, :3])))
        assert error < 1e-10
        report["fk_reference_max_error_m"] = error
    for name in ("wall", "window"):
        scene = load_scene(ROOT / f"resources/scenes/{name}.yaml")
        spheres, boxes = scene_obstacles(scene)
        assert spheres and boxes
        assert any(not np.allclose(q, [0, 0, 0, 1]) for _, _, q in boxes)
        for query in scene["queries"]:
            pair = np.array([query["start"], query["goal"]])
            endpoints = geo.clearance(pair, scene)
            direct = float(geo.clearance(np.linspace(*pair, 401), scene).min())
            assert geo.bounded(pair) and np.all(endpoints > 0) and direct < 0
            report["queries"].append(dict(scene=name, query=query["name"],
                                          endpoint_clearance_m=endpoints.tolist(), direct_min_m=direct))
    for filename in sorted((ROOT / "results").glob("*.json")):
        saved = json.loads(filename.read_text())
        path, scene, query = saved["path"], saved["scene"], saved["query"]
        current = load_scene(ROOT / f"resources/scenes/{scene['name']}.yaml")
        assert scene == current and query in current["queries"]
        assert geo.bounded(path) and path[0] == query["start"]
        assert len(saved["robot"]["joint_names"]) == 7
        fixed = geo.path_minimum(path, scene, geo.step)
        dense = geo.path_minimum(path, scene, 0.002)
        length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        if saved["status"] == "exact":
            assert path[-1] == query["goal"] and fixed > 0
            assert math.isclose(length, saved["length"], abs_tol=1e-9)
            assert math.isclose(fixed, saved["min_clearance"], abs_tol=1e-9)
        else:
            assert saved["status"] == "approximate"
            assert all(saved[k] is None for k in ("length", "smoothness", "min_clearance"))
            assert math.isclose(math.dist(path[-1], query["goal"]),
                                min(math.dist(v, query["goal"]) for v in saved["vertices"]))
        if saved["planner"] == "RTP":
            assert dense > 0
            vertices, edges = saved["vertices"], saved["edges"]
            index = {tuple(v): i for i, v in enumerate(vertices)}
            assert len(index) == len(vertices) and len(edges) == len(vertices) - 1
            incoming = collections.Counter(b for a, b in edges)
            roots = [i for i in range(len(vertices)) if incoming[i] == 0]
            assert len(roots) == 1 and vertices[roots[0]] == query["start"]
            assert all(incoming[i] == 1 for i in range(len(vertices)) if i not in roots)
            children = collections.defaultdict(list)
            for a, b in edges:
                children[a].append(b)
            visited, pending = set(roots), list(roots)
            while pending:
                for child in children[pending.pop()]:
                    if child not in visited:
                        visited.add(child)
                        pending.append(child)
            assert len(visited) == len(vertices)
            edge_set = {tuple(e) for e in edges}
            assert all((index[tuple(a)], index[tuple(b)]) in edge_set for a, b in zip(path, path[1:]))
        report["paths"].append(dict(file=filename.name, status=saved["status"], seed=saved["seed"],
            time_s=saved["planning_time"], waypoints=len(path), length_rad=length,
            sampled_clearance_m=fixed, dense_clearance_m=dense))
    config = json.loads((ROOT / "configs/planners.json").read_text())
    settings = {entry["name"]: entry for entry in config["planners"]}
    assert all(n >= 3 for n in collections.Counter(e["planner"] for e in settings.values()).values())
    metrics = ("time", "solution_length", "solution_smoothness", "minimum_clearance", "graph_states")
    summaries = {}
    for name, tag, scene_name, limit in (("benchmark.db", "main", "clutter", 5),
            ("benchmark_t1.db", "t1", "clutter", 1), ("benchmark_t20.db", "t20", "clutter", 20),
            ("benchmark_shelf.db", "shelf", "shelf", 5)):
        directory = ROOT / "results" / "legacy_2026-10-14" if tag == "main" else ROOT / "results"
        connection = sqlite3.connect((directory / name).as_uri() + "?mode=ro", uri=True)
        connection.row_factory = sqlite3.Row
        assert connection.execute("PRAGMA integrity_check").fetchone()[0] == "ok"
        experiments = list(connection.execute("SELECT * FROM experiments"))
        assert len(experiments) == 5
        assert {e["name"] for e in experiments} == {f"{scene_name}_query{i}_group1" for i in range(1, 6)}
        assert all(e["timelimit"] == limit and e["runcount"] == 5 for e in experiments)
        queries = {q["name"]: q for q in load_scene(ROOT / f"resources/scenes/{scene_name}.yaml")["queries"]}
        for experiment in experiments:
            match = re.search(r"Start states:\s*RealVectorState \[([^]]+)\]\s*Goal state.*?"
                              r"state =\s*RealVectorState \[([^]]+)\]", experiment["setup"], re.S)
            assert match, experiment["name"]
            query = queries[experiment["name"].split("_")[1]]
            for values, endpoint in zip(match.groups(), ("start", "goal")):
                np.testing.assert_allclose([float(v) for v in values.split()], query[endpoint], atol=1e-6, rtol=0)
        for row in connection.execute("SELECT name, settings FROM plannerConfigs"):
            entry = settings[row["name"].removeprefix("geometric_")]
            parameters = dict(re.findall(r"([\w.]+) = ([^\n;]+)", row["settings"]))
            assert float(parameters["longest_valid_segment_fraction"]) == 0.005
            for key, value in entry["settings"].items():
                native_key = {"planner_range": "range", "max_neighbors": "max_nearest_neighbors"}.get(key, key)
                assert float(parameters[native_key]) == value
        runs = list(connection.execute("SELECT runs.*, plannerConfigs.name FROM runs "
                                       "JOIN plannerConfigs ON plannerConfigs.id=runs.plannerid"))
        assert len(runs) == 500
        grouped = collections.defaultdict(list)
        for run in runs:
            grouped[run["name"].removeprefix("geometric_")].append(run)
            assert bool(run["solved"]) == (run["status"] == 6)
        assert set(grouped) == set(settings)
        summary = list(csv.DictReader((directory / f"summary_{tag}.csv").open()))
        assert len(summary) == 20
        for row in summary:
            records = grouped[row["config"]]
            exact = [r for r in records if r["solved"] and not r["approximate_solution"]]
            assert len(records) == int(row["runs"]) == 25 and len(exact) == int(row["exact"])
            assert set(collections.Counter(r["experimentid"] for r in records).values()) == {5}
            for metric in metrics:
                values = [r[metric] for r in (records if metric == "time" else exact)]
                assert all(v is not None for v in values)
                for suffix, function in (("median", statistics.median), ("mean", statistics.mean)):
                    assert math.isclose(function(values), float(row[f"{metric}_{suffix}"]),
                                        rel_tol=1e-10, abs_tol=1e-12)
        summaries[tag] = {r["config"]: r for r in summary}
        report["benchmarks"][tag] = dict(runs=len(runs), exact=sum(r["solved"] for r in runs),
                                          time_limit_s=limit, csv_matches=True)
        connection.close()
    report["best_main_settings"] = {}
    for planner in ("RTP", "RRT", "RRTConnect", "PRM"):
        group = [r for r in summaries["main"].values() if r["config"].split("_")[0] == planner]
        report["best_main_settings"][planner] = {}
        for metric in metrics[:4]:
            winner = (max if metric == "minimum_clearance" else min)(
                group, key=lambda r: float(r[metric + "_median"]))
            report["best_main_settings"][planner][metric] = dict(config=winner["config"],
                median=float(winner[metric + "_median"]), exact=int(winner["exact"]))
    # The existing 20-row comparison table is retained; audit every displayed number.
    tex = (ROOT / "results/legacy_2026-10-14/project3_report.tex").read_text()
    table_settings = set()
    for line in tex.splitlines():
        if not re.match(r"^(RTP gb|RRT gb|RRT-Connect r|PRM k)", line):
            continue
        cells = [s.strip() for s in line.removesuffix("\\\\").split("&")]
        label = cells[0].replace("RRT-Connect", "RRTConnect")
        key = label.replace(" gb=", "_gb").replace(" r=", "_r").replace(" k=", "_k")
        table_settings.add(key)
        assert [int(v) for v in cells[1:5]] == [int(summaries[t][key]["exact"]) for t in ("t1", "main", "t20", "shelf")]
        row = summaries["main"][key]
        for displayed, metric, digits, factor in zip(cells[5:], metrics[:4], (3, 2, 2, 1), (1, 1, 1, 100)):
            assert abs(float(displayed) - float(row[metric + "_median"]) * factor) <= 0.5 * 10**(-digits) + 1e-10
    assert table_settings == set(settings)
    return report


def smoke(native_dir):
    results = []
    # Catch extension/RTTI integration failures that a standalone RTP test misses.
    for bias, budget, expected in ((1.0, 0.1, "exact"), (0.0, 0.0, "approximate")):
        program = f"""
import sys,json
sys.path[:0] = {json.dumps([str(native_dir), str(ROOT)])}
import project3_cpp as cpp
from arm_scene import load_panda
start=[0.,-.785,0.,-2.356,0.,1.571,.785]
goal=[.2,-.785,0.,-2.356,0.,1.571,.785]
r=cpp.plan(load_panda().native_model(),[],[],start,goal,planner='RTP',
    goal_bias={bias},time_limit={budget},seed=450)
assert r['status']=={expected!r}, r['status']
assert r['path'][0]==start
if r['status']=='exact':
    assert r['path'][-1]==goal and len(r['vertices'])==2
else:
    assert r['path']==[start] and all(r[k] is None for k in ('length','smoothness','min_clearance'))
print('RESULT_JSON='+json.dumps(r,allow_nan=False))
"""
        process = subprocess.run([sys.executable, "-B", "-c", program],
                                 capture_output=True, text=True, timeout=10)
        if process.returncode:
            raise RuntimeError(process.stdout + process.stderr)
        saved = json.loads(next(s[12:] for s in process.stdout.splitlines() if s.startswith("RESULT_JSON=")))
        results.append(dict(scene="free_space", status=saved["status"], goal_bias=bias,
                            time_limit_s=budget, waypoints=len(saved["path"])))
    for scene_name, query, seed in (("wall", "query1", 1), ("wall", "query2", 5),
                                    ("window", "query1", 1), ("window", "query2", 1)):
        program = f"""
import sys,json
sys.path[:0] = {json.dumps([str(native_dir), str(ROOT)])}
from arm_scene import load_scene
from plan_arm import run_trial
r=run_trial(load_scene({str(ROOT)!r}+'/resources/scenes/{scene_name}.yaml'),
    {query!r},'RTP',dict(time_limit=10.0,goal_bias=.05,resolution=.005),{seed})
assert r['status'] in ('exact','approximate')
assert r['path'][0]==r['query']['start']
if r['status']=='exact':
    assert r['path'][-1]==r['query']['goal']
else:
    import math
    assert all(r[k] is None for k in ('length','smoothness','min_clearance'))
    assert math.isclose(math.dist(r['path'][-1],r['query']['goal']),
        min(math.dist(v,r['query']['goal']) for v in r['vertices']))
assert len(r['edges'])==len(r['vertices'])-1
print('RESULT_JSON='+json.dumps(r,allow_nan=False))
"""
        process = subprocess.run([sys.executable, "-B", "-c", program],
                                 capture_output=True, text=True, timeout=30)
        if process.returncode:
            raise RuntimeError(process.stdout + process.stderr)
        saved = json.loads(next(s[12:] for s in process.stdout.splitlines() if s.startswith("RESULT_JSON=")))
        geo = Geometry()
        dense = geo.path_minimum(saved["path"], saved["scene"], 0.002)
        assert dense > 0
        results.append(dict(scene=scene_name, query=query, seed=seed, status=saved["status"],
            time_s=saved["planning_time"], waypoints=len(saved["path"]), dense_clearance_m=dense))
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native-dir", type=Path)
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args()
    if args.smoke and not args.native_dir:
        parser.error("--smoke requires --native-dir")
    result = audit_artifacts()
    if args.smoke:
        result["fresh_native_smoke"] = smoke(args.native_dir.resolve())
    print(json.dumps(result, indent=2, allow_nan=False))
