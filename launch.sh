#!/bin/bash

#
# Start the ROS core, controllers and finally the supervisor.
#

roscore &
se306p1/bin/robot_controller _rid:=54 &
se306p1/bin/rotate_supervisor
$1 &
wait
