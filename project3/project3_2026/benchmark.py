"""Run OMPL benchmarks and create a database to upload to Planner Arena."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from arm_scene import DEFAULT_CONFIG, DEFAULT_SCENE, load_panda, load_scene, scene_obstacles


def group_configurations(config, planner_names=None, time_limit=None):
    """One OMPL Request shares a time limit and collision resolution."""
    groups = {}
    for entry in config["planners"]:
        if planner_names is not None and entry["name"] not in planner_names:
            continue
        settings = {**config["common"], **entry["settings"]}
        if time_limit is not None:
            settings["time_limit"] = time_limit
        budget = settings.pop("time_limit")
        resolution = settings.pop("resolution")
        key = (budget, resolution)
        if key not in groups:
            groups[key] = {"time_limit": budget, "resolution": resolution, "configs": []}
        groups[key]["configs"].append({
            "name": entry["name"], "planner": entry["planner"], "settings": settings,
        })
    return list(groups.values())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, nargs="+", default=[DEFAULT_SCENE])
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--planners", nargs="+", help="Configuration names from the JSON file")
    parser.add_argument("--queries", nargs="+", help="Query names; default: all five in each scene")
    parser.add_argument("--repetitions", type=int, default=5,
                        help="OMPL runs per planner configuration on each query")
    parser.add_argument("--time-limit", type=float, help="Override the time limit for every configuration")
    parser.add_argument("--memory-limit", type=float, default=1024.0, help="OMPL memory limit in MB")
    parser.add_argument("--seed", type=int, default=1000,
                        help="Initialize OMPL once; random generators advance across experiments")
    parser.add_argument("--output", type=Path, default=Path("results/benchmark.db"))
    args = parser.parse_args()

    import project3_cpp

    project3_cpp.set_seed(args.seed)
    model = load_panda().native_model()
    groups = group_configurations(json.loads(args.config.read_text()), args.planners, args.time_limit)
    log_directory = args.output.parent / f"{args.output.stem}_logs"
    log_directory.mkdir(parents=True, exist_ok=True)
    logs = []
    for scene_path in args.scene:
        scene = load_scene(scene_path)
        spheres, boxes = scene_obstacles(scene)
        for query in scene["queries"]:
            if args.queries is not None and query["name"] not in args.queries:
                continue
            for index, group in enumerate(groups, start=1):
                name = f"{scene['name']}_{query['name']}_group{index}"
                log_file = log_directory / f"{name}.log"
                project3_cpp.benchmark(
                    model, spheres, boxes, query["start"], query["goal"],
                    group["configs"], str(log_file), experiment_name=name,
                    time_limit=group["time_limit"], run_count=args.repetitions,
                    resolution=group["resolution"], memory_limit=args.memory_limit,
                )
                logs.append(log_file)

    # Convert exactly this invocation's logs into a fresh database on every run.
    converter = shutil.which("ompl_benchmark_statistics.py")
    with tempfile.TemporaryDirectory(dir=args.output.parent, prefix=".benchmark-") as temporary:
        database = Path(temporary) / "benchmark.db"
        subprocess.run([sys.executable, converter, "-d", str(database),
                        *(str(log) for log in logs)], check=True)
        database.replace(args.output)
    print(f"Upload {args.output} to https://planner-arena.kavrakilab.rice.edu/")


if __name__ == "__main__":
    main()
