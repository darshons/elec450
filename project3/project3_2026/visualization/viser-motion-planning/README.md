# viser-motion-planning

A standalone [viser](https://github.com/nerfstudio-project/viser)-based visualization toolkit for robotics: named robot instances (URDF), a MoveIt-style YAML world/obstacle model, trajectory playback, point clouds, camera frustums, and pose-tuning GUI controls.

No dependency on any motion-planning library — usable on its own, or as the visualization layer for one (e.g. [sbmp_jax](https://github.com/EmilianoHFlores/sbmp_jax)).

## Install

```bash
uv pip install -e .
# or, as a dependency elsewhere:
uv add "viser-motion-planning @ git+https://github.com/EmilianoHFlores/viser-motion-planning"
```

## Usage

```python
from viser_motion_planning import Visualizer, Environment

viz = Visualizer(port=8080)
viz.add_grid()
viz.add_environment(Environment.from_yaml("env.yaml"))
robot = viz.add_robot("robot", "path/to/robot.urdf")
viz.run()
```

## Tests

```bash
uv pip install -e ".[dev]"
pytest tests/
```
