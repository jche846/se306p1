#!/bin/bash
. ./setup.sh

roscore &
ROSCORE_PID=$!
rosmake --status-rate=0 -t se306p1
ROSMAKE_RESULT=$?
kill $ROSCORE_PID
exit $ROSMAKE_RESULT
