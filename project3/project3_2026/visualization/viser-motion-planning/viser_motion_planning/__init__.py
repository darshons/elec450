"""viser-motion-planning: viser-based robot, environment, and trajectory visualization.

Standalone viser scene-building toolkit: named robot instances, a MoveIt-style
YAML world/obstacle model, trajectory playback, point clouds, camera frustums,
and pose-tuning GUI controls. No dependency on any motion-planning library —
usable on its own or as the visualization layer for one.
"""
from __future__ import annotations

from viser_motion_planning.camera_frustum import CameraFrustum
from viser_motion_planning.environment import Environment
from viser_motion_planning.gui import JointSliderGroup
from viser_motion_planning.obstacles import (
    BoxObstacle,
    CapsuleObstacle,
    CylinderObstacle,
    MeshObstacle,
    Pose,
    SphereObstacle,
)
from viser_motion_planning.playback import TrajectoryPlayer
from viser_motion_planning.pointcloud import PointCloudLayer
from viser_motion_planning.pose_controls import PoseControls
from viser_motion_planning.robots import RobotHandle
from viser_motion_planning.visualizer import Visualizer

__version__ = "0.1.0"

__all__ = [
    "Visualizer",
    "RobotHandle",
    "Environment",
    "TrajectoryPlayer",
    "JointSliderGroup",
    "PointCloudLayer",
    "CameraFrustum",
    "PoseControls",
    "Pose",
    "SphereObstacle",
    "BoxObstacle",
    "CylinderObstacle",
    "CapsuleObstacle",
    "MeshObstacle",
]
