"""Environment container: a collection of obstacles with loaders and viser export.

The YAML format is the MoveIt-style format used by bimanual_trajopt:

    world:
      collision_objects:
        - id: Sphere1
          primitives:
            - type: sphere          # sphere | box | cylinder | capsule
              dimensions: [0.2]     # sphere: [r]; box: half_extents [x,y,z];
                                    # cylinder/capsule: [r, length]
          primitive_poses:
            - position: [x, y, z]
              orientation: [x, y, z, w]   # xyzw (MoveIt convention)
        - id: Mesh1
          meshes:
            - resource: path/to/mesh.obj
              dimensions: [1.0, 1.0, 1.0]  # scale
          mesh_poses:
            - position: [x, y, z]
              orientation: [x, y, z, w]

Quaternions are converted to wxyz at this boundary; see `obstacles.py` for
package-wide conventions.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path as FilePath
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np
import yaml

from viser_motion_planning.obstacles import (
    BoxObstacle,
    CapsuleObstacle,
    CylinderObstacle,
    MeshObstacle,
    Pose,
    SphereObstacle,
)

Obstacle = Union[
    SphereObstacle, BoxObstacle, CylinderObstacle, CapsuleObstacle, MeshObstacle
]

# Light, semi-transparent default so robots remain visible through obstacles.
DEFAULT_OBSTACLE_RGBA: Tuple[float, float, float, float] = (0.8, 0.6, 0.6, 0.65)


@dataclass
class Environment:
    """A set of obstacles describing the static world.

    Pure data plus format converters: knows nothing about planners or
    collision backends, so those can evolve independently. Exporters for new
    backends (pyroki, VAMP, ...) should be added as `to_*` methods here or as
    free functions consuming `Environment`.
    """

    obstacles: List[Obstacle] = field(default_factory=list)
    name: str = ""

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------
    @classmethod
    def from_yaml(cls, yaml_path: Union[str, FilePath]) -> "Environment":
        """Load from a MoveIt-style YAML file (see module docstring for format)."""
        yaml_path = FilePath(yaml_path)
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)

        env = cls(name=data.get("name", yaml_path.stem))
        for obj in data.get("world", {}).get("collision_objects", []) or []:
            obj_id = obj.get("id", "")
            for prim, pose_dict in zip(
                obj.get("primitives", []) or [], obj.get("primitive_poses", []) or []
            ):
                obstacle = _parse_primitive(prim, pose_dict, obj_id)
                if obstacle is not None:
                    env.obstacles.append(obstacle)
            for mesh, pose_dict in zip(
                obj.get("meshes", []) or [], obj.get("mesh_poses", []) or []
            ):
                env.obstacles.append(
                    MeshObstacle(
                        pose=_parse_pose(pose_dict),
                        mesh_path=mesh.get("resource", ""),
                        scale=tuple(mesh.get("dimensions", [1.0, 1.0, 1.0])),
                        name=obj_id,
                        search_dir=str(yaml_path.parent),
                    )
                )
        return env

    # ------------------------------------------------------------------
    # Typed views (useful for collision exporters)
    # ------------------------------------------------------------------
    @property
    def spheres(self) -> List[SphereObstacle]:
        return [o for o in self.obstacles if isinstance(o, SphereObstacle)]

    @property
    def boxes(self) -> List[BoxObstacle]:
        return [o for o in self.obstacles if isinstance(o, BoxObstacle)]

    @property
    def cylinders(self) -> List[CylinderObstacle]:
        return [o for o in self.obstacles if isinstance(o, CylinderObstacle)]

    @property
    def capsules(self) -> List[CapsuleObstacle]:
        return [o for o in self.obstacles if isinstance(o, CapsuleObstacle)]

    @property
    def meshes(self) -> List[MeshObstacle]:
        return [o for o in self.obstacles if isinstance(o, MeshObstacle)]

    # ------------------------------------------------------------------
    # Functional transforms (return new Environment, never mutate)
    # ------------------------------------------------------------------
    def inflated(self, padding: float) -> "Environment":
        """Return a copy with all obstacles inflated by `padding` (for collision)."""
        return Environment(
            obstacles=[o.inflated(padding) for o in self.obstacles], name=self.name
        )

    def transformed(self, transform: Pose) -> "Environment":
        """Return a copy with all obstacle poses pre-multiplied by `transform`."""
        moved: List[Obstacle] = []
        for o in self.obstacles:
            moved.append(_replace_pose(o, transform @ o.pose))
        return Environment(obstacles=moved, name=self.name)

    # ------------------------------------------------------------------
    # Viser export
    # ------------------------------------------------------------------
    def add_to_scene(
        self,
        server,
        prefix: str = "/environment",
        rgba: Tuple[float, float, float, float] = DEFAULT_OBSTACLE_RGBA,
        random_colors: Optional[
            Tuple[Sequence[float], Sequence[float]]
        ] = None,
        seed: int = 0,
    ) -> List[Any]:
        """Add all obstacles to a viser scene as meshes; returns scene handles.

        Obstacles are grouped under `prefix` so the whole environment can be
        hidden or removed at once.

        Args:
            rgba: Single colour applied to every obstacle (used when
                ``random_colors`` is None).
            random_colors: Optional ``(lower_rgba, upper_rgba)`` pair of 4-vectors;
                each obstacle gets an independent colour sampled uniformly in
                ``[lower, upper]`` per channel. Use a high-brightness, narrow
                range for pastel-like colours.
            seed: PRNG seed for ``random_colors`` (stable colours across redraws).
        """
        rng = None
        if random_colors is not None:
            rng = np.random.default_rng(seed)
            lo = np.asarray(random_colors[0], dtype=float)
            hi = np.asarray(random_colors[1], dtype=float)

        handles = []
        for i, obstacle in enumerate(self.obstacles):
            mesh = obstacle.to_trimesh()
            if rng is not None:
                mesh.visual.vertex_colors = np.clip(rng.uniform(lo, hi), 0.0, 1.0).tolist()
            else:
                mesh.visual.vertex_colors = list(rgba)
            label = obstacle.name or type(obstacle).__name__.lower()
            handles.append(
                server.scene.add_mesh_trimesh(f"{prefix}/{label}_{i}", mesh)
            )
        return handles

    def __str__(self) -> str:
        lines = [f"Environment '{self.name}' with {len(self.obstacles)} obstacles:"]
        for o in self.obstacles:
            lines.append(f"  {type(o).__name__}({o.name}) at {o.pose.position}")
        return "\n".join(lines)


def _parse_pose(pose_dict: Dict[str, Any]) -> Pose:
    return Pose.from_xyzw(
        position=pose_dict.get("position", [0.0, 0.0, 0.0]),
        xyzw=pose_dict.get("orientation", [0.0, 0.0, 0.0, 1.0]),
    )


def _parse_primitive(
    prim: Dict[str, Any], pose_dict: Dict[str, Any], obj_id: str
) -> Optional[Obstacle]:
    prim_type = prim.get("type", "")
    dims: Sequence[float] = prim.get("dimensions", [])
    pose = _parse_pose(pose_dict)

    if prim_type == "sphere" and len(dims) == 1:
        return SphereObstacle(pose=pose, radius=dims[0], name=obj_id)
    if prim_type == "box" and len(dims) == 3:
        return BoxObstacle(pose=pose, half_extents=tuple(dims), name=obj_id)
    if prim_type == "cylinder" and len(dims) == 2:
        return CylinderObstacle(pose=pose, radius=dims[0], length=dims[1], name=obj_id)
    if prim_type == "capsule" and len(dims) == 2:
        return CapsuleObstacle(pose=pose, radius=dims[0], length=dims[1], name=obj_id)

    print(f"Warning: skipping unsupported primitive '{prim_type}' (dims={dims}) in '{obj_id}'")
    return None


def _replace_pose(obstacle: Obstacle, pose: Pose) -> Obstacle:
    import dataclasses

    return dataclasses.replace(obstacle, pose=pose)
