#!/usr/bin/env python

import random
import sys

COLORS = ["blue", "green", "black", "yellow", "red", "brown"]

HEADER = """\
define mylaser laser
(
  range_max 5.0
  fov 180
  samples 180
  color "black"
  size [ 0.05 0.05 0.1 ]
)

define myrobot position
(
  size [0.35 0.35 0.25]
  drive "diff"
  mylaser(pose [ 0.050 0.000 0 0.000 ])
)

define floorplan model
(
  color "gray30"
  boundary 1
  laser_return 1
)

resolution 0.02
interval_sim 100

window
( 
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

def generate(n):
    robots = []

    for i in range(n):
        robots.append("""\
myrobot
(
  name "r{n}"
  pose [ {x} {y} 0 0 ]
  color "{color}"
)
""".format(
            n=i,
            x=random.randint(-30, 30),
            y=random.randint(-30, 30),
            color=COLORS[i % len(COLORS)]
        ))

    return HEADER + "\n".join(robots)

if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.stderr.write("usage: {} NUM_ROBOTS\n".format(sys.argv[0]))
        sys.exit(1)

    print(generate(int(sys.argv[1])))

