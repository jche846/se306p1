#!/usr/bin/env python

import sys
import os
import time
import argparse


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Launch the controller and supervisor nodes.')
    parser.add_argument('-nr', '--numrobots', default=24, help='The number of robots to spawn. (default 24)')
    parser.add_argument('-ns', '--numsupervisors', default=4, help='The number of supervisors to spawn. The number of supervisors dictates the number of robot groups. The size of the groups will be equal to the number of robots divided by the number of supervisors. (default 4)')
    parser.add_argument('-x', '--xrange', default=40, help='The maximum distance away from the origin along the x axis to spawn robots. (default 40)')
    parser.add_argument('-y', '--yrange', default=40, help='The maximum distance away from the origin along the y axis to spawn robots. (default 40)')

    args = parser.parse_args()
    print(args)

    if args.numrobots % args.numsupervisors != 0:
        sys.stderr.write("{} does not divide evenly by {}. There must be an equal number of robots per group.\n".format(args.numrobots, args.numsupervisors))
        sys.exit(1)

    num_members = args.numrobots / args.numsupervisors

    # Launch stage
    os.system("./generate_world.py {} {} {} {} > se306p1.world".format(
        args.numrobots, args.numsupervisors, args.xrange, args.yrange
    ))

    os.system("rosrun stage stageros se306p1.world &")

    time.sleep(2.0)

    # Launch controllers
    print("Launching controllers...")
    for i in range(args.numrobots):
        os.system("se306p1/bin/robot_controller _rid:={} &".format(i))

    time.sleep(3.0)

    # Launch supervisors
    print("Launching supervisors...")
    for i in range(args.numsupervisors):
        os.system("se306p1/bin/supervisor _sid:={} _rmin:={} _rmax:={} &".
                  format(i, i * num_members, (i + 1) * num_members - 1))

    try:
        raw_input("Press ENTER to kill all processes.")
        raise Exception
    except:
        print("Killing controllers and supervisors")
        os.killpg(os.getpgid(os.getpid()), 9)
