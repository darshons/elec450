"""Check the provided infrastructure, independently of the student's RTP.

Run from project3_2026: python tests/check_support.py
Add --check-starter to check that the unimplemented RTP returns a timeout.
The 32 KB FK fixture records Project 2's Pinocchio 4.1.0 output for the
ready pose, both joint-limit corners, and 20 configurations (NumPy seed 450).
Pinocchio and the Project 2 folder are not needed to run these checks.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
CHECK_STARTER = "--check-starter" in sys.argv
if CHECK_STARTER:
    sys.argv.remove("--check-starter")

import project3_cpp as cpp
from arm_scene import PANDA_URDF, load_panda, load_scene, scene_obstacles


def subprocess_result(program, request):
    """Each solve gets a fresh process, as OMPL seeds its RNGs globally."""
    process = subprocess.run(
        [sys.executable, "-c", program], cwd=ROOT,
        input=json.dumps(request), capture_output=True, text=True,
        timeout=30, check=True,
    )
    line = next(line for line in process.stdout.splitlines()
                if line.startswith("RESULT_JSON="))
    return json.loads(line.removeprefix("RESULT_JSON="))


def trial(scene, planner, time_limit=1.0):
    program = (
        "import json, sys; from plan_arm import run_trial; "
        "request = json.load(sys.stdin); "
        "result = run_trial(**request); "
        "print('RESULT_JSON=' + json.dumps(result, allow_nan=False))"
    )
    request = dict(scene=scene, query_name=scene["queries"][0]["name"],
                   planner=planner, settings={"time_limit": time_limit}, seed=450)
    return subprocess_result(program, request)


class SupportChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = load_panda()
        cls.model = cls.robot.native_model()

    def test_fk_matches_independent_pinocchio_reference(self):
        with np.load(Path(__file__).parent / "fixtures/panda_fk_pinocchio.npz") as reference:
            self.assertEqual(hashlib.sha256(PANDA_URDF.read_bytes()).hexdigest(),
                             str(reference["urdf_sha256"]))
            np.testing.assert_array_equal(self.robot.lower, reference["lower"])
            np.testing.assert_array_equal(self.robot.upper, reference["upper"])
            self.assertEqual(len(self.robot.joint_names), 7)
            for index, q in enumerate(reference["q"]):
                with self.subTest(configuration=index):
                    actual = np.array([(*center, radius)
                                       for center, radius in cpp.fk_spheres(self.model, q.tolist())])
                    self.assertEqual(actual.shape, (59, 4))
                    np.testing.assert_allclose(actual, reference["spheres"][index],
                                               rtol=0.0, atol=1e-10)

    def test_sphere_contact_clearance_and_joint_bounds(self):
        model = cpp.ArmModel([], [(0, [0.0, 0.0, 0.0], 0.25)], [-1.0], [1.0])
        for center_x, expected_valid, clearance in [(1.0, True, 0.5),
                                                   (0.5, False, 0.0),
                                                   (0.25, False, -0.25)]:
            with self.subTest(center_x=center_x):
                obstacles = [([center_x, 0.0, 0.0], 0.25)]
                self.assertEqual(cpp.is_valid(model, obstacles, [], [0.0]), expected_valid)
                self.assertAlmostEqual(cpp.clearance(model, obstacles, [], [0.0]), clearance)
        self.assertFalse(cpp.is_valid(model, [], [], [1.001]))
        self.assertTrue(cpp.is_valid(model, [], [], [1.0]))

    def test_rotated_box_geometry_and_containment(self):
        model = cpp.ArmModel([], [(0, [0.0, 1.4, 0.0], 0.5)], [-1.0], [1.0])
        axis_aligned = [([0.0, 0.0, 0.0], [1.0, 0.5, 0.5], [0.0, 0.0, 0.0, 1.0])]
        rotated = [([0.0, 0.0, 0.0], [1.0, 0.5, 0.5],
                    [0.0, 0.0, 2**-0.5, 2**-0.5])]
        self.assertTrue(cpp.is_valid(model, [], axis_aligned, [0.0]))
        self.assertFalse(cpp.is_valid(model, [], rotated, [0.0]))
        self.assertAlmostEqual(cpp.clearance(model, [], axis_aligned, [0.0]), 0.4)
        self.assertAlmostEqual(cpp.clearance(model, [], rotated, [0.0]), -0.1)
        contained = cpp.ArmModel([], [(0, [0.0, 0.0, 0.0], 0.5)], [-1.0], [1.0])
        self.assertFalse(cpp.is_valid(contained, [], axis_aligned, [0.0]))
        self.assertAlmostEqual(cpp.clearance(contained, [], axis_aligned, [0.0]), -1.0)

    def test_supplied_queries_have_valid_endpoints_and_blocked_straight_edges(self):
        for filename in ("clutter.yaml", "shelf.yaml"):
            scene = load_scene(ROOT / "resources/scenes" / filename)
            spheres, boxes = scene_obstacles(scene)
            self.assertEqual(len(scene["queries"]), 5)
            for query in scene["queries"]:
                with self.subTest(scene=filename, query=query["name"]):
                    self.assertTrue(cpp.is_valid(self.model, spheres, boxes, query["start"]))
                    self.assertTrue(cpp.is_valid(self.model, spheres, boxes, query["goal"]))
                    samples = np.linspace(query["start"], query["goal"], 401)
                    self.assertTrue(any(not cpp.is_valid(self.model, spheres, boxes, q.tolist())
                                        for q in samples))

    def assert_exact_valid_path(self, result):
        self.assertEqual(result["status"], "exact")
        path = np.asarray(result["path"])
        np.testing.assert_allclose(path[0], result["query"]["start"], rtol=0.0, atol=1e-9)
        np.testing.assert_allclose(path[-1], result["query"]["goal"], rtol=0.0, atol=1e-6)
        spheres, boxes = scene_obstacles(result["scene"])
        for start, end in zip(path[:-1], path[1:]):
            count = max(2, int(np.ceil(np.linalg.norm(end - start) / 0.01)) + 1)
            self.assertTrue(all(cpp.is_valid(self.model, spheres, boxes, q.tolist())
                                for q in np.linspace(start, end, count)))
        self.assertGreater(result["length"], 0.0)
        self.assertGreater(result["min_clearance"], 0.0)

    def test_built_in_planners_produce_exact_replayable_paths(self):
        start = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        goal = [0.2, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        scene = {
            "name": "support_smoke",
            "world": {"collision_objects": [{
                "id": "distant_sphere", "primitives": [{"type": "sphere", "dimensions": [0.1]}],
                "primitive_poses": [{"position": [5.0, 0.0, 0.0],
                                     "orientation": [0.0, 0.0, 0.0, 1.0]}],
            }]},
            "queries": [{"name": "smoke", "start": start, "goal": goal}],
        }
        for planner in ("RRT", "RRTConnect", "PRM"):
            with self.subTest(planner=planner):
                result = trial(scene, planner)
                self.assert_exact_valid_path(result)
                self.assertEqual(result["scene"], scene)
                self.assertEqual(result["robot"]["joint_names"], self.robot.joint_names)
                self.assertEqual(result["format"], "comp450-project3-path-v1")

    def test_obstacle_avoiding_path_remains_valid_between_waypoints(self):
        scene = load_scene(ROOT / "resources/scenes/clutter.yaml")
        self.assert_exact_valid_path(trial(scene, "RRTConnect", time_limit=3.0))

    def test_approximate_path_keeps_failed_status_and_excludes_quality_metrics(self):
        # One rotating sphere has disconnected free intervals around q=0.
        # The bounded joint cannot go around the obstacle through +/-pi.
        program = """
import json
import project3_cpp as cpp
model = cpp.ArmModel(
    [(0, 1, 0, [0., 0., 0.], [0., 0., 0.], [0., 0., 1.])],
    [(1, [1., 0., 0.], .05)], [-1.], [1.])
result = cpp.plan(model, [([1., 0., 0.], .15)], [], [-1.], [1.],
                  planner='RRT', time_limit=.05, seed=450,
                  planner_range=.1, resolution=.001)
print('RESULT_JSON=' + json.dumps(result, allow_nan=False))
"""
        result = subprocess_result(program, {})
        self.assertEqual(result["status"], "approximate")
        self.assertEqual(result["path"][0], [-1.0])
        self.assertLess(result["path"][-1][0], -0.1)
        for metric in ("length", "smoothness", "min_clearance"):
            self.assertIsNone(result[metric])

    @unittest.skipUnless(CHECK_STARTER, "use --check-starter before implementing RTP")
    def test_unimplemented_rtp_reports_timeout_without_quality_metrics(self):
        scene = load_scene(ROOT / "resources/scenes/clutter.yaml")
        result = trial(scene, "RTP", time_limit=0.01)
        self.assertEqual(result["status"], "timeout")
        self.assertEqual(result["path"], [])
        for metric in ("length", "smoothness", "min_clearance"):
            self.assertIsNone(result[metric])


if __name__ == "__main__":
    unittest.main(verbosity=2)
