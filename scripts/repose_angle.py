#!/usr/bin/env python3

import sys
import pandas as pd
import numpy as np
# import matplotlib.pyplot as plt

argv = sys.argv
if len(argv) != 3:
    print("usage: " + argv[0] + " <input_csv_file> <data_dir>")
    exit(1)

infile = argv[1]
dirname = argv[2]

t = dirname.split('_')[-3:]

data = pd.read_csv(infile)
pos = np.array(data[['x','y','z']])

max_i = np.argmax(pos[:,2])
top_point = pos[max_i,:]
pos = pos[pos[:,0].argsort()]
r_z = np.zeros((pos.shape[0],2))
r_z[:,0] = np.sqrt((pos[:,0]-top_point[0])**2 + (pos[:,1]-top_point[1])**2)
r_z[:,1] = pos[:,2]

part_size = 1.0
rmax = np.max(r_z[:,0])
top_layer = []
# Divide r into partitions
# Get highest point in each partition
# => Gives profile of top surface of mound
# Decide where mounding starts and measure angle to top point.
# IDEA: find highest point, then chose the point (on the top surface) that maximizes
# the slope between it and the highest point. DANGER: don't let it choose a point
# right next to the highest point.
for r in np.arange(0, rmax, part_size):
    lo = r
    hi = r + part_size
    section = r_z[np.where(np.logical_and(r_z[:,0] > lo, r_z[:,0] <= hi))]
    if section.shape[0] == 0:
        continue
    top_layer.append(section[np.argmax(section[:,1]), :])

# top_layer = np.array(top_layer)
# plt.figure()
# plt.scatter(top_layer[:,0], top_layer[:,1])
# plt.xlabel('r')
# plt.ylabel('z')
# plt.axis('equal')
# plt.show()

max_slope = 0.0
for point in top_layer:
    slope = (top_point[2] - point[1]) / point[0]
    if slope > max_slope:
        max_slope = slope

theta = np.degrees(np.arctan(max_slope))
print(",".join([t[0], t[1], t[2], str(theta)]))