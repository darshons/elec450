"""Replay a saved C++ path in Viser; no compiled planner is required."""
import argparse
import json
import logging
from pathlib import Path

import numpy as np
from yourdfpy import URDF
from viser_motion_planning import Visualizer

from arm_scene import PANDA_URDF, scene_environment

logging.getLogger("yourdfpy.urdf").setLevel(logging.ERROR)


def playback_samples(path, step=0.02):
    """Densify piecewise-linear joint motion for display, preserving its path."""
    path = np.asarray(path, dtype=float)
    samples = []
    for start, end in zip(path[:-1], path[1:]):
        count = max(1, int(np.ceil(np.linalg.norm(end - start) / step)))
        samples.extend(start + (end - start) * fraction
                       for fraction in np.linspace(0.0, 1.0, count, endpoint=False))
    samples.append(path[-1])
    return np.asarray(samples)


def show_result(result, port=8080):
    viz = Visualizer(port=port)
    viz.add_grid()
    viz.add_environment(scene_environment(result["scene"]))
    query = result["query"]
    start = viz.add_robot("start", PANDA_URDF, color=(0.2, 0.6, 1.0), opacity=0.25)
    goal = viz.add_robot("goal", PANDA_URDF, color=(0.2, 0.9, 0.35), opacity=0.25)
    start.update_cfg(np.array(query["start"]))
    goal.update_cfg(np.array(query["goal"]))
    robot = viz.add_robot(
        "panda", PANDA_URDF, load_collision_meshes=True,
        collision_color=(0.2, 0.7, 1.0), collision_opacity=0.3,
    )
    robot.update_cfg(np.array(query["start"]))
    robot.show_collision = False

    status = result["status"]
    with viz.server.gui.add_folder("Planning result"):
        viz.server.gui.add_text("Scene", initial_value=result["scene"]["name"], disabled=True)
        viz.server.gui.add_text("Query", initial_value=query["name"], disabled=True)
        viz.server.gui.add_text("Planner", initial_value=result["planner"], disabled=True)
        viz.server.gui.add_text("Status", initial_value=status.upper(), disabled=True)
        viz.server.gui.add_text("Planning time", initial_value=f"{result['planning_time']:.4f} s", disabled=True)
        show_ghosts = viz.server.gui.add_checkbox("Show start and goal", initial_value=True)
        show_spheres = viz.server.gui.add_checkbox("Show collision spheres", initial_value=False)
        if status != "exact":
            viz.server.gui.add_markdown("**The planner did not reach the goal.**")

    @show_ghosts.on_update
    def update_ghosts(_):
        start.set_visible(show_ghosts.value)
        goal.set_visible(show_ghosts.value)

    @show_spheres.on_update
    def update_spheres(_):
        robot.show_collision = show_spheres.value

    player = viz.create_player(duration=6.0)
    if result["path"]:
        samples = playback_samples(result["path"])
        player.set_trajectory(result["planner"], samples)
        kinematics = URDF.load(PANDA_URDF, load_meshes=False)
        ee_path = []
        for q in samples:
            kinematics.update_cfg(dict(zip(result["robot"]["joint_names"], q, strict=True)))
            ee_path.append(kinematics.get_transform("panda_grasptarget")[:3, 3])
        viz.add_path_line("end_effector", np.array(ee_path), color=(0.95, 0.55, 0.05))

    def on_frame():
        q = player.step()
        if q is not None:
            robot.update_cfg(q)

    print(f"Path viewer: http://localhost:{port} (Ctrl+C to stop)", flush=True)
    viz.run(on_frame)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    show_result(json.loads(args.path.read_text()), args.port)


if __name__ == "__main__":
    main()
