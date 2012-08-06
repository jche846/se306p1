#!/bin/bash

#
# Start the ROS core, controllers and finally the supervisor.
#

roscore &
se306p1/bin/robot_controller r0 &
#se306p1/bin/robot_controller r1 &
#se306p1/bin/robot_controller r2 &
#se306p1/bin/robot_controller r3 &
#se306p1/bin/robot_controller r4 &
$1 &
wait
