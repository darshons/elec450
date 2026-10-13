"""Point-cloud layer: a named, in-place updatable point cloud in a viser scene."""
from __future__ import annotations

from typing import Optional

import numpy as np
import numpy.typing as npt
import viser


class PointCloudLayer:
    """A named point cloud whose points and colours can be updated in place.

    Args:
        server: viser server.
        name: Scene node name (e.g. ``/cloud``).
        points: Initial points, shape ``[N, 3]``.
        colors: Optional per-point RGB uint8, shape ``[N, 3]``; defaults to light grey.
        point_size: Rendered point radius in metres.
    """

    def __init__(
        self,
        server: viser.ViserServer,
        name: str,
        points: npt.NDArray,
        colors: Optional[npt.NDArray] = None,
        point_size: float = 0.005,
    ):
        self._name = name
        self._point_size = point_size
        self._handle = server.scene.add_point_cloud(
            name,
            points=self._as_points(points),
            colors=self._as_colors(points, colors),
            point_size=point_size,
            point_shape="circle",
            precision="float32",
        )

    @staticmethod
    def _as_points(points: npt.NDArray) -> npt.NDArray[np.float32]:
        return np.asarray(points, dtype=np.float32).reshape(-1, 3)

    @staticmethod
    def _as_colors(points: npt.NDArray, colors: Optional[npt.NDArray]) -> npt.NDArray[np.uint8]:
        if colors is None:
            return np.full((len(points), 3), 200, dtype=np.uint8)
        return np.asarray(colors, dtype=np.uint8).reshape(-1, 3)

    def update(self, points: npt.NDArray, colors: Optional[npt.NDArray] = None) -> None:
        """Replace the displayed points (and optionally colours)."""
        self._handle.points = self._as_points(points)
        if colors is not None:
            self._handle.colors = self._as_colors(points, colors)

    def set_visible(self, visible: bool) -> None:
        self._handle.visible = visible

    def remove(self) -> None:
        self._handle.remove()
