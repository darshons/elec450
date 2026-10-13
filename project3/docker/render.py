# Headless Viser render: python render.py result.json out.png mode(scene|path) cx cy cz tx ty tz port
import sys, json, time, threading, logging
sys.path.insert(0, "/work")
import numpy as np
from yourdfpy import URDF
from viser_motion_planning import Visualizer
from arm_scene import PANDA_URDF, scene_environment
from visualize_path import playback_samples
logging.getLogger("yourdfpy.urdf").setLevel(logging.ERROR)
res = json.load(open(sys.argv[1])); out = sys.argv[2]; mode = sys.argv[3]
cam = np.array(list(map(float, sys.argv[4:7]))); tgt = np.array(list(map(float, sys.argv[7:10]))); port = int(sys.argv[10])
viz = Visualizer(port=port)
viz.server.gui.configure_theme(control_layout="collapsible", show_logo=False, control_width="small")
viz.add_grid()
viz.add_environment(scene_environment(res["scene"]))
q = res["query"]
if mode == "scene":
    viz.add_robot("start", PANDA_URDF, color=(0.2, 0.5, 1.0), opacity=0.55).update_cfg(np.array(q["start"]))
    viz.add_robot("goal", PANDA_URDF, color=(0.15, 0.8, 0.3), opacity=0.55).update_cfg(np.array(q["goal"]))
else:
    path = np.array(res["path"])
    viz.add_robot("start", PANDA_URDF, color=(0.2, 0.5, 1.0), opacity=0.45).update_cfg(path[0])
    viz.add_robot("goal", PANDA_URDF, color=(0.15, 0.8, 0.3), opacity=0.45).update_cfg(path[-1])
    for i, w in enumerate(path[1:-1]):
        viz.add_robot(f"wp{i}", PANDA_URDF, color=(0.95, 0.55, 0.1), opacity=0.22).update_cfg(w)
    kin = URDF.load(PANDA_URDF, load_meshes=False); ee = []
    for s in playback_samples(path):
        kin.update_cfg(dict(zip(res["robot"]["joint_names"], s))); ee.append(kin.get_transform("panda_grasptarget")[:3, 3])
    viz.add_path_line("ee", np.array(ee), color=(0.85, 0.1, 0.1), line_width=4.0)
@viz.server.on_client_connect
def _(client):
    client.camera.up_direction = (0.0, 0.0, 1.0)
    client.camera.position = tuple(cam); client.camera.look_at = tuple(tgt); client.camera.fov = 0.75
import os
flag = f"/tools/done{port}"
t0 = time.time()
while not os.path.exists(flag) and time.time() - t0 < 180:
    time.sleep(0.5)
if os.path.exists(flag):
    os.remove(flag)
viz.server.stop()
