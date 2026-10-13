#!/bin/bash
# render.sh result.json out.png mode cx cy cz tx ty tz port
P="/home/darshon/courses/COMP-ELEC-MECH 450-550 001/project3/project3_2026"
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -e PYTHONPATH=/work/visualization/viser-motion-planning --network host -v "$P":/work -v /tmp/p3:/tools -w /work comp450-ompl python /tools/render.py "$1" /dev/null "${@:3}" > /tmp/p3/srv_${10}.log 2>&1 &
/tmp/p3/venv/bin/python /tmp/p3/shot.py "${10}" "$2"
wait
