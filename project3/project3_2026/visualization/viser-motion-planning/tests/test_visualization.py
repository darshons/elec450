"""Tests for visualization data structures: poses, obstacles, environment parsing.

Only covers the server-free logic; viser GUI behavior needs a manual/interactive
smoke test against a running viser server (not covered here).
"""
from __future__ import annotations

import textwrap
from pathlib import Path

import numpy as np
import pytest

from viser_motion_planning.environment import Environment
from viser_motion_planning.obstacles import (
    BoxObstacle,
    CapsuleObstacle,
    Pose,
    SphereObstacle,
)

FIXTURES = Path(__file__).resolve().parent / "fixtures"


class TestPose:
    """Tests for the Pose dataclass and quaternion conventions."""

    def test_identity_defaults(self) -> None:
        pose = Pose()
        np.testing.assert_allclose(pose.position, np.zeros(3))
        np.testing.assert_allclose(pose.wxyz, [1.0, 0.0, 0.0, 0.0])

    def test_xyzw_roundtrip(self) -> None:
        """from_xyzw stores wxyz; .xyzw converts back."""
        xyzw = np.array([0.0, 0.0, 0.707, 0.707])
        pose = Pose.from_xyzw(position=[1.0, 2.0, 3.0], xyzw=xyzw)
        np.testing.assert_allclose(pose.wxyz, [0.707, 0.0, 0.0, 0.707])
        np.testing.assert_allclose(pose.xyzw, xyzw)

    def test_matmul_translation(self) -> None:
        """Composing pure translations adds positions."""
        a = Pose(position=np.array([1.0, 0.0, 0.0]))
        b = Pose(position=np.array([0.0, 2.0, 0.0]))
        np.testing.assert_allclose((a @ b).position, [1.0, 2.0, 0.0])

    def test_matmul_rotates_child_position(self) -> None:
        """A 90-degree z-rotation moves a +x offset to +y."""
        rot_z90 = Pose.from_xyzw([0, 0, 0], [0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)])
        child = Pose(position=np.array([1.0, 0.0, 0.0]))
        np.testing.assert_allclose(
            (rot_z90 @ child).position, [0.0, 1.0, 0.0], atol=1e-9
        )

    def test_as_matrix_identity(self) -> None:
        np.testing.assert_allclose(Pose().as_matrix(), np.eye(4))


class TestObstacles:
    """Tests for obstacle primitives."""

    def test_sphere_inflated(self) -> None:
        sphere = SphereObstacle(radius=0.5)
        assert sphere.inflated(0.1).radius == pytest.approx(0.6)
        assert sphere.radius == pytest.approx(0.5)  # original unchanged

    def test_box_inflated_half_extents(self) -> None:
        box = BoxObstacle(half_extents=(0.1, 0.2, 0.3))
        inflated = box.inflated(0.2)
        np.testing.assert_allclose(inflated.half_extents, (0.2, 0.3, 0.4))

    def test_box_trimesh_full_extents(self) -> None:
        """to_trimesh must double the half_extents."""
        box = BoxObstacle(half_extents=(0.1, 0.2, 0.3))
        mesh = box.to_trimesh()
        np.testing.assert_allclose(mesh.extents, (0.2, 0.4, 0.6), atol=1e-9)

    def test_sphere_trimesh_translated(self) -> None:
        sphere = SphereObstacle(
            pose=Pose(position=np.array([1.0, 2.0, 3.0])), radius=0.5
        )
        mesh = sphere.to_trimesh()
        np.testing.assert_allclose(mesh.bounding_box.centroid, [1.0, 2.0, 3.0], atol=1e-6)


class TestEnvironmentYaml:
    """Tests for MoveIt-style YAML parsing."""

    @pytest.fixture
    def yaml_file(self, tmp_path):
        content = textwrap.dedent(
            """
            name: test_env
            world:
              collision_objects:
                - id: MySphere
                  primitives:
                    - type: sphere
                      dimensions: [0.25]
                  primitive_poses:
                    - position: [1.0, 2.0, 3.0]
                      orientation: [0, 0, 0, 1]
                - id: MyBox
                  primitives:
                    - type: box
                      dimensions: [0.1, 0.2, 0.3]
                  primitive_poses:
                    - position: [0.5, 0.0, 0.5]
                      orientation: [0, 0, 0.707, 0.707]
                - id: MyCapsule
                  primitives:
                    - type: capsule
                      dimensions: [0.1, 0.6]
                  primitive_poses:
                    - position: [0.0, 1.0, 0.0]
                      orientation: [0, 0, 0, 1]
            """
        )
        path = tmp_path / "env.yaml"
        path.write_text(content)
        return path

    def test_counts_and_types(self, yaml_file) -> None:
        env = Environment.from_yaml(yaml_file)
        assert env.name == "test_env"
        assert len(env.obstacles) == 3
        assert len(env.spheres) == 1
        assert len(env.boxes) == 1
        assert len(env.capsules) == 1

    def test_sphere_fields(self, yaml_file) -> None:
        env = Environment.from_yaml(yaml_file)
        sphere = env.spheres[0]
        assert sphere.name == "MySphere"
        assert sphere.radius == pytest.approx(0.25)
        np.testing.assert_allclose(sphere.pose.position, [1.0, 2.0, 3.0])

    def test_box_quaternion_converted_to_wxyz(self, yaml_file) -> None:
        """YAML xyzw [0, 0, 0.707, 0.707] becomes wxyz [0.707, 0, 0, 0.707]."""
        env = Environment.from_yaml(yaml_file)
        np.testing.assert_allclose(env.boxes[0].pose.wxyz, [0.707, 0.0, 0.0, 0.707])

    def test_capsule_dimensions(self, yaml_file) -> None:
        env = Environment.from_yaml(yaml_file)
        capsule = env.capsules[0]
        assert isinstance(capsule, CapsuleObstacle)
        assert capsule.radius == pytest.approx(0.1)
        assert capsule.length == pytest.approx(0.6)

    def test_inflated_is_functional(self, yaml_file) -> None:
        env = Environment.from_yaml(yaml_file)
        padded = env.inflated(0.05)
        assert padded.spheres[0].radius == pytest.approx(0.30)
        assert env.spheres[0].radius == pytest.approx(0.25)

    def test_transformed_translates_all(self, yaml_file) -> None:
        env = Environment.from_yaml(yaml_file)
        shifted = env.transformed(Pose(position=np.array([10.0, 0.0, 0.0])))
        np.testing.assert_allclose(shifted.spheres[0].pose.position, [11.0, 2.0, 3.0])
        # Original untouched.
        np.testing.assert_allclose(env.spheres[0].pose.position, [1.0, 2.0, 3.0])

    def test_real_environment_files_parse(self) -> None:
        """The bundled example environments must parse."""
        env = Environment.from_yaml(FIXTURES / "sphere_cage.yaml")
        assert len(env.spheres) == 14
        env2 = Environment.from_yaml(FIXTURES / "bimanual_test_1.yaml")
        assert len(env2.spheres) > 0
