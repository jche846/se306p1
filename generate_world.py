#!/usr/bin/env python

import random
import math
import sys
import os

COLORS = ["blue", "green", "black", "yellow", "red", "brown"]

HEADER = """\
define mylaser laser
(
  range_max 5.0
  fov 180
  samples 180
  color "black"
  size [ 0.05 0.05 0.1 ]
  obstacle_return 0
)

define myrobot position
(
  localization_origin [ 0 0 0 0 ]
  size [0.35 0.35 0.25]
  drive "diff"
  mylaser(pose [ 0.050 0.000 0.000 0.000 ])
  obstacle_return 0
)

define floorplan model
(
  color "gray30"
  boundary 1
  laser_return 1
)

define barcode model
(
  color "black"
  laser_return 1
)

resolution 0.02
interval_sim 100

window
( 
  show_data 1
  size [ 745.000 448.000 ]
  scale 30
)

floorplan
( 
  name "swarm"
  bitmap "swarm_world.pgm"
  size [ 54.0 58.7 0.5 ]
  pose [ 0.0 0.0 0.0 0.0 ]
)
"""


robot_positions = []

def append_robot(robots, n, x, y):
    robots.append("""\
myrobot
(
  name "r{i}"
  pose [ {x} {y} 0 0 ]
  color "{color}"
)
""".format(
        i=len(robots), x=x, y=y, color=COLORS[n % len(COLORS)]
    ))
def append_barcodes():
  barcodes = []
  count = 0
  for filename in os.listdir("images"):
    barcodes.append("""\
barcode
( 
  name "{name}"
  bitmap "images/{fileName}"
  size [ 6 4.5 0.5 ]
  pose [ 60.0 {y}.0 0.0 0.0 ]
)
""".format(
        name=filename.replace(".pgm", "", 1), fileName=filename, y=count * 8
  ))
    count += 1
  return barcodes


def generate(n, gn):
    robots = []

    for i in range(n):
        robot_positions.append((random.randint(-40, 40),
                                random.randint(-40, 40)))

    robot_positions.sort(key=lambda pos: math.sqrt(pos[0] * pos[0] + pos[1] * pos[1]))

    cluster_heads = robot_positions[:gn]
    members = robot_positions[gn:]
    members.reverse()

    num_members = int(round(n / float(gn)))

    for i, (x, y) in enumerate(cluster_heads):
        append_robot(robots, i, x, y)

        for _ in range(num_members - 1):
            if not members: break
            x, y = members.pop()

            append_robot(robots, i, x, y)

    return HEADER + "\n".join(robots) + "\n".join(append_barcodes())

if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.stderr.write("usage: {} NUM_ROBOTS NUM_GROUPS\n".format(sys.argv[0]))
        sys.exit(1)

    print(generate(int(sys.argv[1]), int(sys.argv[2])))


