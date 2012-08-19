#!/usr/bin/env python

import sys
import os

if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.stderr.write("usage: {} NUM_ROBOTS NUM_GROUPS\n".format(sys.argv[0]))
        sys.exit(1)

    num_robots = int(sys.argv[1])
    num_groups = int(sys.argv[2])
    num_members = int(round(num_robots / float(num_groups)))

    for i in range(num_groups):
        os.system("se306p1/bin/rotate_supervisor _sid:={} _rmin:={} _rmax:={} &".format(
            i, i * num_members, (i + 1) * num_members - 1
        ))

    raw_input("Press ENTER to kill all processes.")
    os.killpg(os.getpgid(os.getpid()), 9)

