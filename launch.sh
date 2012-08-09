#!/bin/bash

#
# Generate a world file, Start Robot Controllers.
#
ROBOT_COUNT=$1
if [ -f $1 ]; then
    ROBOT_COUNT=5
fi

echo "Spawning $ROBOT_COUNT robots"

./generate_world.py $ROBOT_COUNT > se306p1.world

rosrun stage stageros se306p1.world &

sleep 1

for i in $(seq 0 $(($ROBOT_COUNT - 1))); do
    exec -a robot_controller_${i} se306p1/bin/robot_controller _rid:=${i} &
    echo $i
done

wait
