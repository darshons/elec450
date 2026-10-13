# Helper for designing scenes: checks queries and searches for configs with the hand in a box region.
# usage: python scene_tool.py scene.yaml [search xmin xmax ymin ymax zmin zmax]
import sys, numpy as np
sys.path.insert(0, "/work")
import project3_cpp as cpp
from arm_scene import load_panda, load_scene, scene_obstacles
desc = load_panda(); model = desc.native_model()
hand_ids = [i for i, s in enumerate(desc.spheres) if s[0] == max(t[0] for t in desc.spheres)]
def hand(q):
    s = cpp.fk_spheres(model, list(q)); return np.mean([s[i][0] for i in hand_ids], axis=0)
scene = load_scene(sys.argv[1]); sp, bx = scene_obstacles(scene)
def edge_ok(a, b):
    a, b = np.array(a), np.array(b)
    n = max(2, int(np.linalg.norm(b - a) / 0.005))
    return all(cpp.is_valid(model, sp, bx, list(a + t * (b - a))) for t in np.linspace(0, 1, n))
if len(sys.argv) > 2 and sys.argv[2] == "search":
    lo = np.array(desc.lower); hi = np.array(desc.upper); r = list(map(float, sys.argv[3:9]))
    rng = np.random.default_rng(int(sys.argv[9]) if len(sys.argv) > 9 else 0); found = 0
    while found < 8:
        q = rng.uniform(lo, hi); h = hand(q)
        if r[0]<h[0]<r[1] and r[2]<h[1]<r[3] and r[4]<h[2]<r[5] and cpp.is_valid(model, sp, bx, list(q)):
            print(np.round(q, 2).tolist(), np.round(h, 2).tolist(), round(cpp.clearance(model, sp, bx, list(q)), 3)); found += 1
else:
    for q in scene["queries"]:
        s, g = q["start"], q["goal"]
        print(q["name"], "start valid", cpp.is_valid(model, sp, bx, s), "clr", round(cpp.clearance(model, sp, bx, s), 3), "hand", np.round(hand(s), 2),
              "| goal valid", cpp.is_valid(model, sp, bx, g), "clr", round(cpp.clearance(model, sp, bx, g), 3), "hand", np.round(hand(g), 2),
              "| direct edge free:", edge_ok(s, g))
if len(sys.argv) > 2 and sys.argv[2] == "grid":
    # natural top-down grasp poses: j3=j5=0, j6=j2-j4 keeps the hand pointing down
    j1 = float(sys.argv[3])
    for j2 in np.arange(-0.4, 1.21, 0.2):
        for j4 in np.arange(-2.8, -0.99, 0.2):
            q = [j1, j2, 0.0, j4, 0.0, j2 - j4, 0.785]
            if desc.lower[5] <= q[5] <= desc.upper[5] and cpp.is_valid(model, sp, bx, q):
                print(np.round(q, 3).tolist(), np.round(hand(q), 2).tolist(), round(cpp.clearance(model, sp, bx, q), 3))
