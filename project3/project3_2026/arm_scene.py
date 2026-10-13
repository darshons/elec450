"""Load the supplied Panda and the Project 2 obstacle format."""
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parent
PANDA_URDF = ROOT / "resources/panda/panda_spherized.urdf"
DEFAULT_SCENE = ROOT / "resources/scenes/clutter.yaml"
DEFAULT_CONFIG = ROOT / "configs/planners.json"


@dataclass
class PandaDescription:
    joint_names: list
    joints: list
    spheres: list
    lower: list
    upper: list

    def native_model(self):
        import project3_cpp

        return project3_cpp.ArmModel(self.joints, self.spheres, self.lower, self.upper)


def load_panda() -> PandaDescription:
    """Translate URDF data once; C++ performs FK during collision checking."""
    robot = ET.parse(PANDA_URDF).getroot()
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    pending = list(robot.findall("joint"))
    children = {joint.find("child").attrib["link"] for joint in pending}
    root_name = next(name for name in links if name not in children)
    indices = {root_name: 0}
    joints, names, lower, upper = [], [], [], []

    while pending:
        joint = next(j for j in pending if j.find("parent").attrib["link"] in indices)
        pending.remove(joint)
        parent = indices[joint.find("parent").attrib["link"]]
        child_name = joint.find("child").attrib["link"]
        child = len(indices)
        indices[child_name] = child
        origin = joint.find("origin")
        xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
        q_index = -1
        axis = [0.0, 0.0, 1.0]
        if joint.attrib["type"] == "revolute":
            q_index = len(names)
            names.append(joint.attrib["name"])
            axis = [float(v) for v in joint.find("axis").attrib["xyz"].split()]
            limit = joint.find("limit")
            lower.append(float(limit.attrib["lower"]))
            upper.append(float(limit.attrib["upper"]))
        joints.append((parent, child, q_index, xyz, rpy, axis))

    spheres = []
    for name, link in links.items():
        for collision in link.findall("collision"):
            center = [float(v) for v in collision.find("origin").attrib["xyz"].split()]
            radius = float(collision.find("geometry/sphere").attrib["radius"])
            spheres.append((indices[name], center, radius))
    return PandaDescription(names, joints, spheres, lower, upper)


def load_scene(path):
    return yaml.safe_load(Path(path).read_text())


def scene_obstacles(scene):
    """C++ inputs: sphere (center, radius), box (center, half-extents, xyzw)."""
    spheres, boxes = [], []
    for obj in scene["world"]["collision_objects"]:
        for primitive, pose in zip(obj["primitives"], obj["primitive_poses"], strict=True):
            if primitive["type"] == "sphere":
                spheres.append((pose["position"], primitive["dimensions"][0]))
            elif primitive["type"] == "box":
                boxes.append((pose["position"], primitive["dimensions"], pose["orientation"]))
            else:
                raise ValueError("Project 3 scenes use sphere and box obstacles.")
    return spheres, boxes


def scene_environment(scene):
    """Viser obstacles from the embedded scene, without a native extension."""
    from viser_motion_planning.environment import Environment
    from viser_motion_planning.obstacles import BoxObstacle, Pose, SphereObstacle

    obstacles = []
    for obj in scene["world"]["collision_objects"]:
        for primitive, pose in zip(obj["primitives"], obj["primitive_poses"], strict=True):
            placement = Pose.from_xyzw(pose["position"], pose["orientation"])
            if primitive["type"] == "sphere":
                obstacles.append(SphereObstacle(placement, primitive["dimensions"][0], obj["id"]))
            elif primitive["type"] == "box":
                obstacles.append(BoxObstacle(placement, tuple(primitive["dimensions"]), obj["id"]))
            else:
                raise ValueError("Project 3 scenes use sphere and box obstacles.")
    return Environment(obstacles=obstacles, name=scene["name"])
