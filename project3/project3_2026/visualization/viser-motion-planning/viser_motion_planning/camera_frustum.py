"""Camera frustum layer: a movable viewing-frustum gizmo in a viser scene."""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
import numpy.typing as npt
import viser


class CameraFrustum:
    """A camera viewing frustum placed at a pose, with an optional preview image.

    The frustum is drawn at ``position``/``wxyz`` (the frame the caller supplies) with the
    given vertical field of view. Callers pass the camera pose already expressed in the
    convention viser expects for cameras.

    Args:
        server: viser server.
        name: Scene node name.
        vfov_rad: Vertical field of view in radians.
        aspect: Width / height aspect ratio.
        position: Frustum apex position ``[x, y, z]``.
        wxyz: Frustum orientation quaternion ``[w, x, y, z]``.
        scale: Frustum size in metres.
        color: Line colour as an RGB 0-255 tuple.
        image: Optional preview image ``[H, W, 3]`` uint8 shown on the far plane.
    """

    def __init__(
        self,
        server: viser.ViserServer,
        name: str,
        vfov_rad: float,
        aspect: float,
        position: npt.NDArray,
        wxyz: npt.NDArray,
        scale: float = 0.3,
        color: Tuple[int, int, int] = (240, 170, 40),
        image: Optional[npt.NDArray] = None,
    ):
        self._handle = server.scene.add_camera_frustum(
            name,
            fov=float(vfov_rad),
            aspect=float(aspect),
            scale=scale,
            color=color,
            image=image,
            position=tuple(np.asarray(position, dtype=float)),
            wxyz=tuple(np.asarray(wxyz, dtype=float)),
        )

    def set_pose(self, position: npt.NDArray, wxyz: npt.NDArray) -> None:
        """Move the frustum to a new pose."""
        self._handle.position = tuple(np.asarray(position, dtype=float))
        self._handle.wxyz = tuple(np.asarray(wxyz, dtype=float))

    def update_image(self, image: Optional[npt.NDArray]) -> None:
        """Replace the preview image on the frustum's far plane (e.g. a live camera feed)."""
        self._handle.image = image

    def set_visible(self, visible: bool) -> None:
        self._handle.visible = visible

    def remove(self) -> None:
        self._handle.remove()
