#!/usr/bin/env python3

import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

argv = sys.argv
if len(argv) != 2:
    print("usage: " + argv[0] + " <input_csv_file>")
    exit(1)

infile = argv[1]
t = infile.split('_')[-3:]
data = pd.read_csv(infile)
pos = np.array(data[['x','y','z']])

max_i = np.argmax(pos[:,2])
center = pos[max_i,:]

r_z = np.zeros((pos.shape[0],2))
r_z[:,1] = pos[:,2]
r_z[:,0] = np.sqrt((pos[:,0] - center[0])**2 + (pos[:,1] - center[1])**2)
r_z = r_z[r_z[:,0].argsort()]
# print(r_z)

lo, hi = np.percentile(r_z[:,0], [95,100])
third_quart = r_z[np.where(np.logical_and(lo <= r_z[:,0], r_z[:,0] <= hi)),:][0]
mid_point_i = np.argmax(third_quart[:,1])
mid_point = third_quart[mid_point_i,:]

slope = (center[2] - mid_point[1]) / mid_point[0]
theta = np.degrees(np.arctan(slope))

print(",".join([t[0], t[1], t[2], str(theta)]))