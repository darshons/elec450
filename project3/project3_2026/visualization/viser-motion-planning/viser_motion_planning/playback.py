"""Trajectory playback: GUI controls (slider / play / duration) over named trajectories.

The player owns only playback state; it does not know about robots. Each loop
iteration, call `step()` to get the current joint configuration and apply it
to whatever robot(s) you like. This keeps playback reusable for ghosts,
multi-robot scenes, and non-robot consumers (e.g. FK path highlighting).
"""
from __future__ import annotations

import time
from typing import Dict, Optional, Tuple

import numpy as np
import numpy.typing as npt
import viser


class TrajectoryPlayer:
    """Plays back one or more named joint-space trajectories.

    Trajectories are arrays of shape [timesteps, dof], or [timesteps, 1 + dof]
    with time in the first column when `has_time=True` (then playback honors
    the timestamps via the time display, and `step()` strips the time column).

    GUI elements are created lazily on the first `set_trajectory` call so an
    idle player adds no clutter.
    """

    def __init__(
        self,
        server: viser.ViserServer,
        duration: float = 5.0,
        folder_label: str = "Playback",
    ):
        self._server = server
        self._default_duration = duration
        self._folder_label = folder_label

        self._trajs: Dict[str, Tuple[npt.NDArray[np.float64], bool]] = {}
        self._current_name: Optional[str] = None
        self._cycle_start: float = time.time()

        # GUI handles (created lazily)
        self._folder: Optional[viser.GuiFolderHandle] = None
        self._slider: Optional[viser.GuiSliderHandle] = None
        self._playing: Optional[viser.GuiCheckboxHandle] = None
        self._duration_slider: Optional[viser.GuiSliderHandle] = None
        self._selector: Optional[viser.GuiDropdownHandle] = None
        self._time_text: Optional[viser.GuiTextHandle] = None

    # ------------------------------------------------------------------
    # Trajectory management
    # ------------------------------------------------------------------
    def set_trajectory(
        self,
        name: str,
        trajectory: npt.NDArray[np.float64],
        has_time: bool = False,
        select: bool = True,
    ) -> None:
        """Add or replace a named trajectory and (optionally) make it current."""
        trajectory = np.asarray(trajectory, dtype=np.float64)
        if trajectory.ndim != 2 or len(trajectory) < 1:
            raise ValueError(
                f"Trajectory must have shape [timesteps, dof], got {trajectory.shape}"
            )
        self._trajs[name] = (trajectory, has_time)
        if select or self._current_name is None:
            self._current_name = name
        self._build_or_update_gui()
        self.restart()

    @property
    def playing(self) -> bool:
        return self._playing is not None and self._playing.value

    @playing.setter
    def playing(self, value: bool) -> None:
        if self._playing is not None:
            self._playing.value = value
            if value:
                self.restart()

    def restart(self) -> None:
        """Restart the playback cycle from the first timestep."""
        self._cycle_start = time.time()
        if self._slider is not None:
            self._slider.value = 0

    # ------------------------------------------------------------------
    # Main loop hook
    # ------------------------------------------------------------------
    def step(self) -> Optional[npt.NDArray[np.float64]]:
        """Advance playback and return the current joint configuration.

        Returns None if no trajectory has been set yet. Call once per loop
        iteration; the caller applies the returned config to its robot(s).
        """
        if not self._trajs:
            return None

        if self._selector is not None and self._selector.value != self._current_name:
            self._current_name = self._selector.value
            self.restart()

        trajectory, has_time = self._trajs[self._current_name]
        n = len(trajectory)

        if self._playing.value and n > 1:
            duration = max(self._duration_slider.value, 1e-3)
            progress = ((time.time() - self._cycle_start) % duration) / duration
            self._slider.value = int(progress * (n - 1))

        idx = min(self._slider.value, n - 1)
        row = trajectory[idx]

        if has_time:
            if self._time_text is not None:
                self._time_text.value = f"{row[0]:.3f} s"
            return row[1:]
        return row

    # ------------------------------------------------------------------
    # GUI
    # ------------------------------------------------------------------
    def _build_or_update_gui(self) -> None:
        max_len = max(len(t) for t, _ in self._trajs.values())
        any_has_time = any(has_time for _, has_time in self._trajs.values())

        if self._slider is None:
            self._folder = self._server.gui.add_folder(self._folder_label)
            with self._folder:
                self._slider = self._server.gui.add_slider(
                    "Timestep", min=0, max=max(max_len - 1, 1), step=1, initial_value=0
                )
                self._playing = self._server.gui.add_checkbox("Playing", initial_value=False)
                self._duration_slider = self._server.gui.add_slider(
                    "Duration (s)", min=0.5, max=30.0, step=0.5,
                    initial_value=self._default_duration,
                )
                self._playing.on_update(lambda _: self.restart())
        else:
            self._slider.max = max(max_len - 1, 1)

        if any_has_time and self._time_text is None:
            with self._folder:
                self._time_text = self._server.gui.add_text("Time", initial_value="0.000 s")

        names = list(self._trajs.keys())
        if len(names) > 1:
            if self._selector is None:
                with self._folder:
                    self._selector = self._server.gui.add_dropdown(
                        "Trajectory", options=names, initial_value=self._current_name
                    )
            else:
                self._selector.options = names
