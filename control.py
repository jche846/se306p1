#!/usr/bin/env python

import sys
import os

if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.stderr.write("usage: {} NUM_ROBOTS NUM_GROUPS\n".format(sys.argv[0]))
        sys.exit(1)

    num_robots = int(sys.argv[1])
    num_groups = int(sys.argv[2])

    os.system("./generate_world.py {} {} > se306p1.world".format(
        num_robots, num_groups
    ))

    os.system("rosrun stage stageros se306p1.world &")

    for i in range(num_robots):
        os.system("se306p1/bin/robot_controller _rid:={} &".format(i))

    raw_input("Press ENTER to kill all processes.")
    os.killpg(os.getpgid(os.getpid()), 9)

