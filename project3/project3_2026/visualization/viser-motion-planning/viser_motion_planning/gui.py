"""GUI building blocks: joint slider groups for setting configurations by hand."""
from __future__ import annotations

from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
import numpy.typing as npt
import viser


class JointSliderGroup:
    """A folder of per-joint sliders representing one robot configuration.

    Built from joint names and limits (use `RobotHandle.joint_limits`), so it
    works for any URDF without robot-specific code.
    """

    def __init__(
        self,
        server: viser.ViserServer,
        joint_limits: Dict[str, Tuple[float, float]],
        label: str,
        initial_values: Optional[npt.NDArray[np.float64]] = None,
    ):
        self._sliders: List[viser.GuiSliderHandle] = []
        names = list(joint_limits.keys())
        if initial_values is None:
            initial_values = np.array([(lo + hi) / 2.0 for lo, hi in joint_limits.values()])
        initial_values = np.asarray(initial_values, dtype=np.float64)
        if len(initial_values) != len(names):
            raise ValueError(
                f"initial_values has {len(initial_values)} entries, expected {len(names)}"
            )

        with server.gui.add_folder(label, expand_by_default=False):
            for joint_name, init in zip(names, initial_values):
                lower, upper = joint_limits[joint_name]
                slider = server.gui.add_slider(
                    joint_name,
                    min=lower,
                    max=upper,
                    step=max((upper - lower) / 200.0, 1e-3),
                    initial_value=float(np.clip(init, lower, upper)),
                )
                self._sliders.append(slider)

    @property
    def values(self) -> npt.NDArray[np.float64]:
        """Current configuration as an array over actuated joints."""
        return np.array([s.value for s in self._sliders])

    def set_values(self, cfg: npt.NDArray[np.float64]) -> None:
        for slider, value in zip(self._sliders, np.asarray(cfg)):
            slider.value = float(value)

    def on_change(self, callback: Callable[[npt.NDArray[np.float64]], None]) -> None:
        """Register `callback(cfg)` to fire whenever any slider moves."""
        for slider in self._sliders:
            slider.on_update(lambda _, cb=callback: cb(self.values))
