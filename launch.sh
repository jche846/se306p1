#!/bin/bash

#
# Start the ROS core, controllers and finally the supervisor.
#

roscore &
exec -a robot_controller_${RANDOM} se306p1/bin/robot_controller _rid:=54 &
exec -a robot_controller_${RANDOM} se306p1/bin/robot_controller _rid:=55 &
exec -a robot_controller_${RANDOM} se306p1/bin/robot_controller _rid:=56 &
se306p1/bin/rotate_supervisor &
$1 &
wait
