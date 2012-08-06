#!/bin/bash

ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH rosmake --status-rate=0 -t se306p1
