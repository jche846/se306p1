#!/bin/bash

#
# Start the ROS core, controllers and finally the supervisor.
#

#roscore &
se306p1/bin/rotate_supervisor &
sleep 1
exec -a robot_controller_${RANDOM} se306p1/bin/robot_controller _rid:=0 &
exec -a robot_controller_${RANDOM} se306p1/bin/robot_controller _rid:=1 &
exec -a robot_controller_${RANDOM} se306p1/bin/robot_controller _rid:=2 &
exec -a robot_controller_${RANDOM} se306p1/bin/robot_controller _rid:=3 &
exec -a robot_controller_${RANDOM} se306p1/bin/robot_controller _rid:=4 &
$1 &
wait
