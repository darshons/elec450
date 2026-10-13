"""6-DoF pose controls: sliders for fine x/y/z + roll/pitch/yaw tuning around a pose."""
from __future__ import annotations

from typing import Callable, List

import numpy as np
import viser
from scipy.spatial.transform import Rotation

from viser_motion_planning.obstacles import Pose


class PoseControls:
    """A folder of six sliders that fine-tune a pose around an initial reference.

    Position sliders are absolute (initial +/- ``position_range_m``); orientation sliders
    are a roll/pitch/yaw delta in degrees (initial +/- ``angle_range_deg``) applied on top
    of the reference orientation, which is ideal for nudging a calibration estimate.

    Args:
        server: viser server.
        label: Folder label.
        initial: Reference pose the sliders are centred on.
        position_range_m: Half-range of each position slider, in metres.
        angle_range_deg: Half-range of each orientation slider, in degrees.
    """

    def __init__(
        self,
        server: viser.ViserServer,
        label: str,
        initial: Pose,
        position_range_m: float = 0.15,
        angle_range_deg: float = 10.0,
    ):
        self._initial = initial
        self._reference_rotation = Rotation.from_quat(initial.xyzw)
        with server.gui.add_folder(label):
            self._pos = [
                server.gui.add_slider(
                    axis,
                    min=float(initial.position[i] - position_range_m),
                    max=float(initial.position[i] + position_range_m),
                    step=position_range_m / 500.0,
                    initial_value=float(initial.position[i]),
                )
                for i, axis in enumerate(("x (m)", "y (m)", "z (m)"))
            ]
            self._rpy = [
                server.gui.add_slider(
                    axis, min=-angle_range_deg, max=angle_range_deg,
                    step=angle_range_deg / 500.0, initial_value=0.0,
                )
                for axis in ("roll (deg)", "pitch (deg)", "yaw (deg)")
            ]
        self._sliders = self._pos + self._rpy

    @property
    def value(self) -> Pose:
        """Current tuned pose."""
        position = np.array([s.value for s in self._pos], dtype=np.float64)
        delta = Rotation.from_euler("xyz", [s.value for s in self._rpy], degrees=True)
        rotation = delta * self._reference_rotation
        return Pose(position=position, wxyz=np.roll(rotation.as_quat(), 1))

    def on_change(self, callback: Callable[[Pose], None]) -> None:
        """Register ``callback(pose)`` to fire whenever any slider moves."""
        for slider in self._sliders:
            slider.on_update(lambda _: callback(self.value))

    def reset(self) -> None:
        """Return all sliders to the initial reference pose."""
        for i, slider in enumerate(self._pos):
            slider.value = float(self._initial.position[i])
        for slider in self._rpy:
            slider.value = 0.0
