"""Top-level Visualizer: composes a viser server, robots, environment, and playback.

A thin facade over focused components:

    - `RobotHandle` (robots.py): named URDF instances with movable base frames
    - `Environment` (environment.py): obstacle data + viser export
    - `TrajectoryPlayer` (playback.py): playback GUI and state
    - `JointSliderGroup` (gui.py): per-joint configuration sliders

Typical usage::

    viz = Visualizer(port=8080)
    viz.add_grid()
    viz.add_environment(Environment.from_yaml("env.yaml"))
    robot = viz.add_robot("robot", urdf)
    player = viz.create_player(duration=5.0)
    player.set_trajectory("plan", traj)

    def on_frame():
        cfg = player.step()
        if cfg is not None:
            robot.update_cfg(cfg)

    viz.run(on_frame)
"""
from __future__ import annotations

import time
from typing import Callable, Dict, Optional, Tuple

import numpy as np
import numpy.typing as npt
import viser

from viser_motion_planning.environment import Environment
from viser_motion_planning.gui import JointSliderGroup
from viser_motion_planning.playback import TrajectoryPlayer
from viser_motion_planning.robots import RobotHandle


class Visualizer:
    """Composable robot/trajectory visualizer backed by a viser server."""

    def __init__(self, server: Optional[viser.ViserServer] = None, port: int = 8080):
        self.server = server if server is not None else viser.ViserServer(port=port)
        self._robots: Dict[str, RobotHandle] = {}

    # ------------------------------------------------------------------
    # Scene setup
    # ------------------------------------------------------------------
    def add_grid(
        self, width: float = 4.0, height: float = 4.0, cell_size: float = 0.25
    ) -> None:
        self.server.scene.add_grid(
            "/grid", width=width, height=height, cell_size=cell_size
        )

    def add_environment(
        self, environment: Environment, prefix: str = "/environment"
    ) -> None:
        environment.add_to_scene(self.server, prefix=prefix)

    def add_path_line(
        self,
        name: str,
        points: npt.NDArray[np.float64],
        color: Tuple[float, float, float] = (1.0, 0.0, 0.0),
        line_width: float = 2.0,
    ) -> None:
        """Draw a 3D polyline (e.g. an end-effector path) through `points` [N, 3]."""
        points = np.asarray(points, dtype=np.float32)
        if len(points) < 2:
            return
        segments = np.stack([points[:-1], points[1:]], axis=1)
        colors = np.broadcast_to(
            np.asarray(color, dtype=np.float32), segments.shape
        ).copy()
        self.server.scene.add_line_segments(
            f"/paths/{name}", points=segments, colors=colors, line_width=line_width
        )

    # ------------------------------------------------------------------
    # Robots
    # ------------------------------------------------------------------
    def add_robot(
        self,
        name: str,
        urdf,
        color: Optional[Tuple[float, float, float]] = None,
        opacity: float = 1.0,
        base_position: Optional[npt.NDArray[np.float64]] = None,
        base_wxyz: Optional[Tuple[float, float, float, float]] = None,
        load_meshes: bool = True,
        load_collision_meshes: bool = False,
        collision_color: Optional[Tuple[float, float, float]] = None,
        collision_opacity: float = 1.0,
    ) -> RobotHandle:
        """Add a named robot instance (translucent ghosts via color + opacity).

        Set `load_collision_meshes=True` to also load the URDF's `<collision>`
        geometry (e.g. a spherized collision model) as an overlay, toggled
        independently via the returned handle's `show_visual`/`show_collision`.
        """
        if name in self._robots:
            raise ValueError(f"Robot '{name}' already exists; remove it first.")
        handle = RobotHandle(
            self.server,
            name,
            urdf,
            color=color,
            opacity=opacity,
            base_position=base_position,
            base_wxyz=base_wxyz,
            load_meshes=load_meshes,
            load_collision_meshes=load_collision_meshes,
            collision_color=collision_color,
            collision_opacity=collision_opacity,
        )
        self._robots[name] = handle
        return handle

    def get_robot(self, name: str) -> Optional[RobotHandle]:
        return self._robots.get(name)

    def remove_robot(self, name: str) -> None:
        handle = self._robots.pop(name, None)
        if handle is not None:
            handle.remove()

    # ------------------------------------------------------------------
    # GUI / playback factories
    # ------------------------------------------------------------------
    def add_joint_sliders(
        self,
        robot: RobotHandle,
        label: str,
        initial_values: Optional[npt.NDArray[np.float64]] = None,
    ) -> JointSliderGroup:
        """Create a slider folder for `robot`'s actuated joints."""
        return JointSliderGroup(
            self.server, robot.joint_limits, label, initial_values=initial_values
        )

    def create_player(
        self, duration: float = 5.0, folder_label: str = "Playback"
    ) -> TrajectoryPlayer:
        return TrajectoryPlayer(self.server, duration=duration, folder_label=folder_label)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(
        self, on_frame: Optional[Callable[[], None]] = None, hz: float = 60.0
    ) -> None:
        """Run the visualization loop until interrupted (Ctrl+C)."""
        period = 1.0 / hz
        try:
            while True:
                if on_frame is not None:
                    on_frame()
                time.sleep(period)
        except KeyboardInterrupt:
            print("Visualization stopped.")
