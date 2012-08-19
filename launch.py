#!/usr/bin/env python

import sys
import os

if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.stderr.write("usage: {} [supervisor|controller]\n".format(sys.argv[0]))
        sys.exit(1)

    node_to_launch = sys.argv[1]

    with open("launch.config") as f:
        raw_n, raw_gn = f.read().split()

    num_robots = int(raw_n)
    num_groups = int(raw_gn)

    if node_to_launch == "supervisor":
        num_members = int(round(num_robots / float(num_groups)))

        for i in range(num_groups):
            os.system("se306p1/bin/supervisor _sid:={} _rmin:={} _rmax:={} &".format(
                i, i * num_members, (i + 1) * num_members - 1
            ))
    elif node_to_launch == "controller":
        for i in range(num_robots):
            os.system("se306p1/bin/robot_controller _rid:={} &".format(i))

        os.system("./generate_world.py {} {} > se306p1.world".format(
            num_robots, num_groups
        ))

        os.system("rosrun stage stageros se306p1.world &")

    try:
        raw_input("Press ENTER to kill all processes.")
        raise Exception
    except:
        os.killpg(os.getpgid(os.getpid()), 9)

