"""Run one C++ planning trial and save a replayable JSON result."""
import argparse
import json
from pathlib import Path

from arm_scene import DEFAULT_SCENE, load_panda, load_scene, scene_obstacles


def run_trial(scene, query_name, planner, settings, seed):
    import project3_cpp

    robot = load_panda()
    model = robot.native_model()
    spheres, boxes = scene_obstacles(scene)
    query = next(query for query in scene["queries"] if query["name"] == query_name)
    result = project3_cpp.plan(
        model, spheres, boxes, query["start"], query["goal"],
        planner=planner, seed=seed, **settings,
    )
    return {
        "format": "comp450-project3-path-v1",
        "robot": {"name": "panda", "joint_names": robot.joint_names},
        "scene": scene,
        "query": query,
        "planner": planner,
        "settings": settings,
        "seed": seed,
        **result,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--query", default="query1",
                        help="Select a start/goal pair in the scene YAML (default: query1)")
    parser.add_argument("--planner", default="RRTConnect")
    parser.add_argument("--time-limit", type=float, default=2.0)
    parser.add_argument("--goal-bias", type=float, default=0.05)
    parser.add_argument("--range", dest="planner_range", type=float, default=0.4)
    parser.add_argument("--max-neighbors", type=int, default=10)
    parser.add_argument("--resolution", type=float, default=0.005)
    parser.add_argument("--seed", type=int, default=1,
                        help="OMPL random seed for this planning run (default: 1)")
    parser.add_argument("--output", type=Path, default=Path("results/path.json"))
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()

    settings = {key: getattr(args, key) for key in (
        "time_limit", "goal_bias", "planner_range", "max_neighbors", "resolution",
    )}
    result = run_trial(load_scene(args.scene), args.query, args.planner, settings, args.seed)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(f"{args.planner}: {result['status']}, {result['planning_time']:.4f} s, "
          f"{len(result['path'])} waypoints; saved {args.output}")
    if args.visualize:
        from visualize_path import show_result

        show_result(result, port=args.port)


if __name__ == "__main__":
    main()
