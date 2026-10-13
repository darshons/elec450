# Sanity checks for RTP; run inside container from /work
import sys, numpy as np
sys.path.insert(0, "/work")
import project3_cpp as cpp
from arm_scene import load_panda, load_scene, scene_obstacles

model = load_panda().native_model()
bad = 0
for scene_name in sys.argv[1:] or ["clutter", "shelf"]:
    scene = load_scene(f"/work/resources/scenes/{scene_name}.yaml")
    sp, bx = scene_obstacles(scene)
    for q in scene["queries"]:
        for seed in range(1, 6):
            r = cpp.plan(model, sp, bx, q["start"], q["goal"], planner="RTP", time_limit=10, goal_bias=0.05, seed=seed)
            path = np.array(r["path"])
            ok = r["status"] == "exact" and np.allclose(path[0], q["start"]) and np.allclose(path[-1], q["goal"])
            # dense check of every edge
            for a, b in zip(path[:-1], path[1:]):
                n = max(2, int(np.linalg.norm(b - a) / 0.002))
                for t in np.linspace(0, 1, n):
                    if not cpp.is_valid(model, sp, bx, list(a + t * (b - a))):
                        ok = False
            # planner data: tree with |V|-1 edges
            ok = ok and len(r["edges"]) == len(r["vertices"]) - 1
            if not ok:
                bad += 1
                print("FAIL", scene_name, q["name"], seed, r["status"])
        print(scene_name, q["name"], "last:", r["status"], round(r["planning_time"], 3), len(r["path"]), "verts", len(r["vertices"]))
# approximate: tiny time limit
scene = load_scene("/work/resources/scenes/clutter.yaml"); sp, bx = scene_obstacles(scene); q = scene["queries"][0]
r = cpp.plan(model, sp, bx, q["start"], q["goal"], planner="RTP", time_limit=0.002, goal_bias=0.0, seed=3)
print("tiny limit:", r["status"], len(r["path"]), r["length"], np.allclose(r["path"][0], q["start"]))
print("failures:", bad)
