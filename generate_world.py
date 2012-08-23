#!/usr/bin/env python

import random
import math
import sys
import os

HEADER = """\
resolution 0.02
interval_sim 100

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

window
(
  show_data 1
  size [ 745.000 448.000 ]
  scale 30
)

"""

COLORS = [line.split(
    "\t")[-1].strip() for line in open("/etc/X11/rgb.txt").readlines()[1:]]


def append_robot(robots, num_clusters, cluster_size, cluster_index, x, y):
    num_colors = len(COLORS)
    slice_size = num_colors / num_clusters
    min_color = (cluster_index * slice_size) % num_colors
    max_color = (min_color + cluster_size) % num_colors

    robots.append("""\
  myrobot
  (
    name "r{i}"
    pose [ {x}.0 {y}.0 0.0 0.0 ]
    color "{color}"
  )

  """.format(
                  i=len(robots), x=x, y=y,
                  color=random.choice(COLORS[min_color:max_color])
                  ))


def append_barcodes(barcodes, i, filename):
    barcodes.append("""\
  barcode
  (
    name "{name}"
    bitmap "images/{image_file}"
    size [ 7.5 3.5 0.5 ]
    pose [ 60.0 {y}.0 0.0 0.0 ]
  )

  """.format(
        name=filename.replace(".pgm", "", 1), image_file=filename, y=i * 8
    ))


def generate(num_robots, num_groups, x_range, y_range, zeronode):
    # Generate all the robot positions. No robot is allowed to be on the
    # origin.
    robot_positions = []
    for i in range(num_robots):
        (x, y) = (0, 0)
        while (x, y) == (0, 0):
            (x, y) = (random.randint(-x_range, x_range), random.randint(-y_range, y_range))

        robot_positions.append((x, y))
    # Sort them by distance from the origin
    robot_positions.sort(
        key=lambda pos: math.sqrt(pos[0] * pos[0] + pos[1] * pos[1]))
    
    if zeronode:
      robot_positions[num_groups] = (0, 0)
  
    # Cluster heads are the robots closest to the origin.
    cluster_heads = robot_positions[:num_groups]
    cluster_members = robot_positions[num_groups:]
    cluster_size = num_robots / num_groups

    random.shuffle(cluster_members)

    # Generate robots
    robots = []
    for i, (x, y) in enumerate(cluster_heads):
        append_robot(robots, len(cluster_heads), cluster_size, i, x, y)

        for _ in range(cluster_size - 1):
            if not cluster_members:
                break
            x, y = cluster_members.pop()

            append_robot(robots, len(cluster_heads), cluster_size, i, x, y)

    # Generate barcodes
    barcodes = []
    for i, filename in enumerate(sorted(os.listdir("images"))):
        append_barcodes(barcodes, i, filename)

    # Return the generated robots and barcodes with the header.
    return HEADER + "\n".join(robots) + "\n".join(barcodes)

if __name__ == '__main__':
    if len(sys.argv) == 6:
        num_robots = int(sys.argv[1])
        num_groups = int(sys.argv[2])
        x_range = int(sys.argv[3])
        y_range = int(sys.argv[4])
        zeronode = sys.argv[5] == 'True'
    else:
        sys.stderr.write("Not enough arguments supplied.")
        sys.exit(1)

    print(generate(num_robots, num_groups, x_range, y_range, zeronode))
