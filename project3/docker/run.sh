#!/bin/bash
# Run a command inside the ompl_env container with the project mounted at /work
P="/home/darshon/courses/COMP-ELEC-MECH 450-550 001/project3/project3_2026"
exec docker run --rm -i --user "$(id -u):$(id -g)" -e HOME=/tmp -e ENV_NAME=ompl_env -e CXX=/usr/bin/g++ \
  -e PYTHONPATH=/work/visualization/viser-motion-planning \
  --network host -v "$P":/work -w /work comp450-ompl bash -c "$*"
