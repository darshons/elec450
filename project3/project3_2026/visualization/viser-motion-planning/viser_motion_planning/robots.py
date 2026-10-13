"""Robot instance management: named URDF visuals with an optional base frame.

Each robot lives under its own scene frame (`/<name>`), so multiple instances
of the same URDF can coexist (e.g. translucent "start"/"goal" ghosts plus a
solid "playback" robot) and mobile bases can be moved by updating the frame.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np
import numpy.typing as npt
import viser
from viser.extras import ViserUrdf


class RobotHandle:
    """A named robot visual: a ViserUrdf parented to a movable base frame.

    Args:
        server: viser server.
        name: Unique name; the robot is placed under scene node `/<name>`.
        urdf: URDF path (str/Path) or yourdfpy.URDF object.
        color: Optional RGB tuple (0-1). None keeps the URDF's own materials.
        opacity: Alpha 0-1, only applied when `color` is given.
        base_position: Initial base frame position (default origin).
        base_wxyz: Initial base frame orientation quaternion (default identity).
        load_meshes: Whether to load/show the URDF's `<visual>` meshes.
        load_collision_meshes: Whether to also load/show the URDF's
            `<collision>` geometry (e.g. a spherized collision model), as an
            independently toggleable overlay (see `show_visual`/`show_collision`).
        collision_color: Optional RGB tuple (0-1) for the collision geometry.
            None keeps the URDF's own materials.
        collision_opacity: Alpha 0-1, only applied when `collision_color` is given.
    """

    def __init__(
        self,
        server: viser.ViserServer,
        name: str,
        urdf,
        color: Optional[Tuple[float, float, float]] = None,
        opacity: float = 1.0,
        base_position: Optional[npt.NDArray[np.float64]] = None,
        base_wxyz: Optional[Tuple[float, float, float, float]] = None,
        scale: float = 1.0,
        load_meshes: bool = True,
        load_collision_meshes: bool = False,
        collision_color: Optional[Tuple[float, float, float]] = None,
        collision_opacity: float = 1.0,
    ):
        self.name = name
        self._server = server
        self._frame: viser.FrameHandle = server.scene.add_frame(
            f"/{name}",
            position=tuple(base_position) if base_position is not None else (0.0, 0.0, 0.0),
            wxyz=tuple(base_wxyz) if base_wxyz is not None else (1.0, 0.0, 0.0, 0.0),
            show_axes=False,
        )
        self.urdf_vis = ViserUrdf(
            server,
            urdf,
            scale=scale,
            root_node_name=f"/{name}",
            mesh_color_override=(*color, opacity) if color is not None else None,
            collision_mesh_color_override=(
                (*collision_color, collision_opacity) if collision_color is not None else None
            ),
            load_meshes=load_meshes,
            load_collision_meshes=load_collision_meshes,
        )

    # ------------------------------------------------------------------
    # Visual / collision geometry visibility
    # ------------------------------------------------------------------
    @property
    def show_visual(self) -> bool:
        return self.urdf_vis.show_visual

    @show_visual.setter
    def show_visual(self, visible: bool) -> None:
        self.urdf_vis.show_visual = visible

    @property
    def show_collision(self) -> bool:
        return self.urdf_vis.show_collision

    @show_collision.setter
    def show_collision(self, visible: bool) -> None:
        self.urdf_vis.show_collision = visible

    # ------------------------------------------------------------------
    # Joint info
    # ------------------------------------------------------------------
    @property
    def actuated_joint_names(self) -> List[str]:
        return list(self.urdf_vis.get_actuated_joint_names())

    @property
    def joint_limits(self) -> Dict[str, Tuple[float, float]]:
        """Actuated joint limits with None/inf bounds clamped to [-pi, pi]."""
        limits = {}
        for joint_name, (lower, upper) in self.urdf_vis.get_actuated_joint_limits().items():
            lower = lower if lower is not None and np.isfinite(lower) else -np.pi
            upper = upper if upper is not None and np.isfinite(upper) else np.pi
            limits[joint_name] = (float(lower), float(upper))
        return limits

    @property
    def num_actuated_joints(self) -> int:
        return len(self.actuated_joint_names)

    def midpoint_cfg(self) -> npt.NDArray[np.float64]:
        """Configuration at the midpoint of each joint's limits."""
        return np.array([(lo + hi) / 2.0 for lo, hi in self.joint_limits.values()])

    # ------------------------------------------------------------------
    # Updates
    # ------------------------------------------------------------------
    def update_cfg(self, cfg: npt.NDArray[np.float64]) -> None:
        """Set joint configuration (array over actuated joints, in URDF order)."""
        self.urdf_vis.update_cfg(np.asarray(cfg))

    def set_base_pose(
        self,
        position: Optional[npt.NDArray[np.float64]] = None,
        wxyz: Optional[Tuple[float, float, float, float]] = None,
    ) -> None:
        """Move the robot's base frame (for mobile robots / multi-robot layouts)."""
        if position is not None:
            self._frame.position = tuple(np.asarray(position, dtype=np.float64))
        if wxyz is not None:
            self._frame.wxyz = tuple(wxyz)

    def set_visible(self, visible: bool) -> None:
        self._frame.visible = visible

    def remove(self) -> None:
        self.urdf_vis.remove()
        self._frame.remove()
