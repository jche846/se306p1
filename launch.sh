#!/bin/bash

#
# Start the ROS core, controllers and finally the supervisor.
#

roscore &
./generate_world.py $1 > se306p1.world
rosrun stage stageros se306p1.world &
se306p1/bin/rotate_supervisor &
sleep 1
for i in $(seq 0 $(($1 - 1))); do
    exec -a robot_controller_${RANDOM} se306p1/bin/robot_controller _rid:=${i} &
    echo $i
done
wait
