"""NumPy forward kinematics and sphere/box clearance used by check_part2.py."""
import math
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from arm_scene import load_panda, scene_obstacles


class Geometry:
    """Independent NumPy FK and signed sphere/OBB distance calculation."""
    def __init__(self):
        self.robot = load_panda()
        self.links = np.array([s[0] for s in self.robot.spheres])
        self.centers = np.array([s[1] for s in self.robot.spheres])
        self.radii = np.array([s[2] for s in self.robot.spheres])
        self.link_count = max(max(j[:2]) for j in self.robot.joints) + 1
        self.origins = []
        for _, _, _, xyz, rpy, _ in self.robot.joints:
            transform = np.eye(4)
            transform[:3, 3] = xyz
            transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
            self.origins.append(transform)
        self.step = 0.005 * np.linalg.norm(np.array(self.robot.upper) - self.robot.lower)

    def fk(self, qs):
        qs = np.atleast_2d(qs)
        transforms = np.broadcast_to(np.eye(4), (len(qs), self.link_count, 4, 4)).copy()
        for joint, origin in zip(self.robot.joints, self.origins):
            parent, child, index, _, _, axis = joint
            transforms[:, child] = transforms[:, parent] @ origin
            if index >= 0:
                rotations = Rotation.from_rotvec(qs[:, index, None] * np.array(axis)).as_matrix()
                transforms[:, child, :3, :3] = transforms[:, child, :3, :3] @ rotations
        selected = transforms[:, self.links]
        return (np.einsum("nsij,sj->nsi", selected[:, :, :3, :3], self.centers)
                + selected[:, :, :3, 3])

    def clearance(self, qs, scene):
        positions = self.fk(qs)
        spheres, boxes = scene_obstacles(scene)
        result = np.full(len(positions), np.inf)
        for center, radius in spheres:
            distances = np.linalg.norm(positions - center, axis=2) - self.radii - radius
            result = np.minimum(result, distances.min(axis=1))
        for center, half, quaternion in boxes:
            local = (positions - center) @ Rotation.from_quat(quaternion).as_matrix()
            delta = np.abs(local) - half
            distances = (np.linalg.norm(np.maximum(delta, 0), axis=2)
                         + np.minimum(delta.max(axis=2), 0) - self.radii)
            result = np.minimum(result, distances.min(axis=1))
        return result

    def bounded(self, path):
        qs = np.asarray(path)
        return bool(np.all((qs >= self.robot.lower) & (qs <= self.robot.upper)))

    def path_minimum(self, path, scene, step):
        minimum = float(self.clearance(path, scene).min())
        for start, end in zip(path[:-1], path[1:]):
            count = max(1, math.ceil(math.dist(start, end) / step))
            minimum = min(minimum, float(self.clearance(np.linspace(start, end, count + 1), scene).min()))
        return minimum
