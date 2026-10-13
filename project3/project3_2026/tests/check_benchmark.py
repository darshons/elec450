"""Check native OMPL logs and Planner Arena database conversion.

Run after make: python tests/check_benchmark.py
Uses the installed ompl_benchmark_statistics.py converter. These checks do
not depend on the student's RTP implementation.
"""
import json
import math
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]


def rrt_config(name, step):
    return {"name": name, "planner": "RRT",
            "settings": {"goal_bias": 1.0, "planner_range": step}}


def native_experiment(log_path, configs, obstacle_x, obstacle_radius, run_count):
    # This rotating sphere has an analytically known path in the clear scene
    # and disconnected free intervals when the obstacle is on its orbit.
    program = """
import json, sys
import project3_cpp as cpp
request = json.load(sys.stdin)
cpp.set_seed(450)
model = cpp.ArmModel(
    [(0, 1, 0, [0., 0., 0.], [0., 0., 0.], [0., 0., 1.])],
    [(1, [1., 0., 0.], .05)], [-1.], [1.])
spheres = [([request.pop('obstacle_x'), 0., 0.], request.pop('obstacle_radius'))]
result = cpp.benchmark(model, spheres, [], [-1.], [1.], **request)
assert result is None
"""
    request = dict(log_path=str(log_path), configs=configs, obstacle_x=obstacle_x,
                   obstacle_radius=obstacle_radius, run_count=run_count,
                   experiment_name=log_path.stem, time_limit=.03,
                   resolution=.001, memory_limit=1024.0)
    subprocess.run([sys.executable, "-c", program], cwd=ROOT,
                   input=json.dumps(request), capture_output=True, text=True,
                   check=True, timeout=30)


class BenchmarkChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="project3-benchmark-check-")
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.output = Path(cls.temporary.name)
        cls.exact_log, cls.failed_log = cls.output / "exact.log", cls.output / "failed.log"
        native_experiment(
            cls.exact_log, [rrt_config("RRT_quarter", .25), rrt_config("RRT_half", .5)],
            obstacle_x=2.0, obstacle_radius=.1, run_count=3)
        native_experiment(
            cls.failed_log, [rrt_config("RRT_approximate", .1), rrt_config("RRT_timeout", 2.0)],
            obstacle_x=1.0, obstacle_radius=.15, run_count=2)
        cls.database = cls.output / "native.db"
        subprocess.run([sys.executable, shutil.which("ompl_benchmark_statistics.py"),
                        "-d", str(cls.database), str(cls.exact_log), str(cls.failed_log)],
                       check=True, capture_output=True, text=True, timeout=30)
        with sqlite3.connect(cls.database) as connection:
            connection.row_factory = sqlite3.Row
            cls.runs = [dict(row) for row in connection.execute("""
                SELECT runs.*, plannerConfigs.name, plannerConfigs.settings,
                       enums.description AS status_description
                FROM runs JOIN plannerConfigs ON plannerConfigs.id = runs.plannerid
                JOIN enums ON enums.name = 'status' AND enums.value = runs.status
                ORDER BY runs.id
            """)]

    def test_native_logs_roundtrip_preserves_runs_parameters_and_clear_lifecycle(self):
        self.assertEqual(len(self.runs), 10)
        with sqlite3.connect(self.database) as connection:
            experiments = connection.execute(
                "SELECT name, runcount, seed FROM experiments ORDER BY name").fetchall()
            self.assertEqual(experiments, [("exact", 3, "450"), ("failed", 2, "450")])
            columns = {row[1] for row in connection.execute("PRAGMA table_info(runs)")}
            self.assertFalse(any("simplif" in name for name in columns))
        for name, count, step in (("RRT_quarter", 9, .25), ("RRT_half", 5, .5)):
            runs = [run for run in self.runs if run["name"] == f"geometric_{name}"]
            self.assertEqual(len(runs), 3)
            for run in runs:
                self.assertEqual(run["solved"], 1)
                self.assertEqual(run["status_description"], "Exact solution")
                self.assertEqual(run["graph_states"], count)
                self.assertEqual(run["graph_motions"], count - 1)
                self.assertEqual(run["solution_segments"], count - 1)
                self.assertAlmostEqual(run["solution_length"], 2.0)
                self.assertAlmostEqual(run["solution_smoothness"], 0.0)
                self.assertIn(f"range = {step}", run["settings"])
                self.assertIn("goal_bias = 1", run["settings"])
        for log in (self.exact_log, self.failed_log):
            self.assertTrue(log.read_text().startswith("OMPL version "))

    def test_database_distinguishes_approximate_solutions_from_timeouts(self):
        failed = [run for run in self.runs if not run["solved"]]
        self.assertEqual(len(failed), 4)
        for run in failed:
            self.assertIsNone(run["minimum_clearance"])
            if run["name"] == "geometric_RRT_approximate":
                self.assertEqual(run["status_description"], "Approximate solution")
                self.assertEqual(run["approximate_solution"], 1)
                self.assertGreater(run["solution_length"], 0)
            else:
                self.assertEqual(run["name"], "geometric_RRT_timeout")
                self.assertEqual(run["status_description"], "Timeout")
                self.assertIsNone(run["solution_length"])
        self.assertEqual(sum(run["solved"] for run in self.runs), 6)

    def test_minimum_clearance_remains_distinct_from_ompl_average_clearance(self):
        for run in (run for run in self.runs if run["solved"]):
            count = run["solution_segments"] + 1
            angles = [-1.0 + 2.0 * i / (count - 1) for i in range(count)]
            # Sphere center is (cos(q),sin(q),0); obstacle center is (2,0,0).
            expected_mean = sum(math.sqrt(5 - 4 * math.cos(q)) - .15 for q in angles) / count
            self.assertAlmostEqual(run["solution_clearance"], expected_mean, delta=1e-5)
            self.assertAlmostEqual(run["minimum_clearance"], .85, places=12)
            self.assertGreater(run["solution_clearance"], run["minimum_clearance"] + .1)

    def test_cli_writes_only_database_and_logs_and_replaces_previous_runs(self):
        start = [0.0, -.785, 0.0, -2.356, 0.0, 1.571, .785]
        goal = [.8, -.785, 0.0, -2.356, 0.0, 1.571, .785]
        scene = {
            "name": "benchmark_smoke",
            "world": {"collision_objects": [{
                "id": "distant_sphere", "primitives": [{"type": "sphere", "dimensions": [.1]}],
                "primitive_poses": [{"position": [5., 0., 0.], "orientation": [0., 0., 0., 1.]}],
            }]},
            "queries": [{"name": "query1", "start": start, "goal": goal}],
        }
        scene_path, config_path = self.output / "scene.yaml", self.output / "config.json"
        scene_path.write_text(yaml.safe_dump(scene))
        config_path.write_text(json.dumps({"common": {"time_limit": .1, "resolution": .005},
                                          "planners": [rrt_config("short", .1), rrt_config("long", .4)]}))
        destination = self.output / "cli" / "benchmark.db"
        command = [
            sys.executable, str(ROOT / "benchmark.py"), "--scene", str(scene_path),
            "--config", str(config_path), "--repetitions", "2", "--seed", "821",
            "--output", str(destination),
        ]
        for invocation in range(2):
            with self.subTest(invocation=invocation + 1):
                subprocess.run(command, cwd=ROOT, check=True, capture_output=True, text=True, timeout=30)
                with sqlite3.connect(destination) as connection:
                    self.assertEqual(connection.execute("SELECT COUNT(*) FROM runs").fetchone()[0], 4)
                    self.assertEqual(connection.execute("SELECT SUM(solved) FROM runs").fetchone()[0], 4)
                    self.assertEqual(connection.execute("SELECT seed FROM experiments").fetchall(), [("821",)])
                    names = {row[0] for row in connection.execute("SELECT name FROM plannerConfigs")}
                    self.assertEqual(names, {"geometric_short", "geometric_long"})
                files = {path.relative_to(destination.parent) for path in destination.parent.rglob("*")
                         if path.is_file()}
                self.assertEqual(files, {
                    Path("benchmark.db"), Path("benchmark_logs/benchmark_smoke_query1_group1.log"),
                })


if __name__ == "__main__":
    unittest.main(verbosity=2)
