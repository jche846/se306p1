#!/usr/bin/env python

import sys
import os

if __name__ == '__main__':
    if len(sys.argv) > 2:
        sys.stderr.write("usage: {} [CONFIG_FILE]\n".format(sys.argv[1]))
        sys.exit(1)

    if len(sys.argv) == 2:
        config_fn = sys.argv[1]
    else:
        config_fn = "launch.config"

    with open(config_fn) as f:
        raw_n, raw_gn = f.read().split()

    num_robots = int(raw_n)
    num_groups = int(raw_gn)

    os.system("./generate_world.py {} {} > se306p1.world".format(
        num_robots, num_groups
    ))

    os.system("rosrun stage stageros se306p1.world &")

    for i in range(num_robots):
        os.system("se306p1/bin/robot_controller _rid:={} &".format(i))

    try:
        raw_input("Press ENTER to kill all processes.")
    except:
        os.killpg(os.getpgid(os.getpid()), 9)

