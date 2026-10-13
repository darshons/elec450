"""Obstacle primitives and poses for environment description and visualization.

Conventions (important, these differ from some upstream tools):
    - Quaternions are stored as **wxyz** everywhere in this package (matching
      viser and jaxlie). MoveIt-style YAML files use xyzw; conversion happens
      once, at the YAML-parsing boundary in `environment.py`.
    - Box sizes are stored as **half_extents** (matching the YAML environment
      format used by bimanual_trajopt / VAMP). `to_trimesh()` converts to the
      full extents trimesh expects.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path as FilePath
from typing import Optional, Tuple

import numpy as np
import numpy.typing as npt
import trimesh
from scipy.spatial.transform import Rotation


def _as_f64(x, shape: Tuple[int, ...]) -> npt.NDArray[np.float64]:
    arr = np.asarray(x, dtype=np.float64)
    if arr.shape != shape:
        raise ValueError(f"Expected shape {shape}, got {arr.shape}")
    return arr


@dataclass(frozen=True)
class Pose:
    """Rigid-body pose: position [x, y, z] and orientation quaternion [w, x, y, z]."""

    position: npt.NDArray[np.float64] = field(
        default_factory=lambda: np.zeros(3, dtype=np.float64)
    )
    wxyz: npt.NDArray[np.float64] = field(
        default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    )

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", _as_f64(self.position, (3,)))
        object.__setattr__(self, "wxyz", _as_f64(self.wxyz, (4,)))

    @classmethod
    def from_xyzw(cls, position, xyzw) -> "Pose":
        """Build from scipy/MoveIt-convention quaternion [x, y, z, w]."""
        return cls(position=np.asarray(position), wxyz=np.roll(np.asarray(xyzw), 1))

    @property
    def xyzw(self) -> npt.NDArray[np.float64]:
        """Orientation as [x, y, z, w] (scipy convention)."""
        return np.roll(self.wxyz, -1)

    def as_matrix(self) -> npt.NDArray[np.float64]:
        """4x4 homogeneous transform."""
        mat = np.eye(4)
        mat[:3, :3] = Rotation.from_quat(self.xyzw).as_matrix()
        mat[:3, 3] = self.position
        return mat

    def __matmul__(self, other: "Pose") -> "Pose":
        """Compose two poses: (self @ other) applies `other` then `self`."""
        r_self = Rotation.from_quat(self.xyzw)
        r_out = r_self * Rotation.from_quat(other.xyzw)
        p_out = self.position + r_self.apply(other.position)
        return Pose(position=p_out, wxyz=np.roll(r_out.as_quat(), 1))


@dataclass(frozen=True)
class SphereObstacle:
    pose: Pose = field(default_factory=Pose)
    radius: float = 1.0
    name: str = ""

    def inflated(self, padding: float) -> "SphereObstacle":
        return SphereObstacle(self.pose, self.radius + padding, self.name)

    def to_trimesh(self) -> trimesh.Trimesh:
        mesh = trimesh.creation.icosphere(radius=self.radius)
        mesh.apply_transform(self.pose.as_matrix())
        return mesh


@dataclass(frozen=True)
class BoxObstacle:
    pose: Pose = field(default_factory=Pose)
    half_extents: Tuple[float, float, float] = (0.5, 0.5, 0.5)
    name: str = ""

    def inflated(self, padding: float) -> "BoxObstacle":
        new_half = tuple(h + 0.5 * padding for h in self.half_extents)
        return BoxObstacle(self.pose, new_half, self.name)

    def to_trimesh(self) -> trimesh.Trimesh:
        mesh = trimesh.creation.box(extents=[2.0 * h for h in self.half_extents])
        mesh.apply_transform(self.pose.as_matrix())
        return mesh


@dataclass(frozen=True)
class CylinderObstacle:
    pose: Pose = field(default_factory=Pose)
    radius: float = 1.0
    length: float = 1.0
    name: str = ""

    def inflated(self, padding: float) -> "CylinderObstacle":
        return CylinderObstacle(self.pose, self.radius + padding, self.length, self.name)

    def to_trimesh(self) -> trimesh.Trimesh:
        mesh = trimesh.creation.cylinder(radius=self.radius, height=self.length)
        mesh.apply_transform(self.pose.as_matrix())
        return mesh


@dataclass(frozen=True)
class CapsuleObstacle:
    pose: Pose = field(default_factory=Pose)
    radius: float = 1.0
    length: float = 1.0
    name: str = ""

    def inflated(self, padding: float) -> "CapsuleObstacle":
        return CapsuleObstacle(self.pose, self.radius + padding, self.length, self.name)

    def to_trimesh(self) -> trimesh.Trimesh:
        mesh = trimesh.creation.capsule(radius=self.radius, height=self.length)
        mesh.apply_transform(self.pose.as_matrix())
        return mesh


@dataclass(frozen=True)
class MeshObstacle:
    pose: Pose = field(default_factory=Pose)
    mesh_path: str = ""
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    name: str = ""
    # Resolved lazily; search_dir lets paths be relative to the YAML file.
    search_dir: Optional[str] = None

    def inflated(self, padding: float) -> "MeshObstacle":
        if abs(padding) > 1e-9:
            raise NotImplementedError("Mesh obstacle padding is not supported.")
        return self

    def resolve_path(self) -> FilePath:
        path = FilePath(self.mesh_path)
        if path.is_absolute():
            return path
        if self.search_dir is not None and (FilePath(self.search_dir) / path).exists():
            return FilePath(self.search_dir) / path
        return FilePath.cwd() / path

    def to_trimesh(self) -> trimesh.Trimesh:
        mesh = trimesh.load(self.resolve_path(), force="mesh")
        mesh.apply_scale(self.scale)
        mesh.fix_normals()
        mesh.apply_transform(self.pose.as_matrix())
        return mesh
