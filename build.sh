#!/bin/bash

export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
rosmake se306p1
