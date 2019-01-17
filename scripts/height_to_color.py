#!/usr/bin/env python2
import numpy as np
from matplotlib import pyplot as plt
import sys

x = []
y = []
z = []
vabs = []

fname = sys.argv[1]

with open(fname, 'r') as f:
    first = True
    for line in f.readlines():
        # Skip first line
        if (first):
            first = False
            continue
        l=line.split(",")

        x.append(float(l[0]))
        y.append(float(l[1]))
        z.append(float(l[2]))

x = np.array(x)
y = np.array(y)
z = np.array(z)

zmin = min(z)
zmax = max(z)

zrange = zmax - zmin

b = (z - zmin) / zrange
r = 1.0 - ((z - zmin) / zrange)
print(zmin, zmax, zrange)

g = "0.0"

ofilename = fname[:-4] + "_colors.csv"

with open(ofilename, 'w') as ofile:
    for i in range(len(b)):
        ostr = ",".join([str(r[i]), g, str(b[i])]) + "\n"
        ofile.write(ostr)



