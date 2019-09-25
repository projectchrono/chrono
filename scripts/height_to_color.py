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
    lines = f.readlines()
    print(len(lines))
    z = np.zeros(len(lines)-1)
    for i in range(1, len(lines)):

        l=lines[i].split(",")

        z[i-1] = float(l[2])

print("loading done")

# x = np.array(x)
# y = np.array(y)
z = np.array(z)

zmin = min(z)
zmax = max(z)


colorlist = []
colorlist.append([.28,.92,1])
colorlist.append([1,.48,.28])
colorlist.append([1,1,1])
colorlist.append([1,.68,.28])
colorlist.append([.68,.48,1])
colorlist.append([.28,.92,1])
colorlist.append([1,.48,.28])
colorlist.append([1,1,1])
colorlist.append([1,.68,.28])
colorlist.append([.68,.48,1])

ncolors = len(colorlist)

zrange = zmax - zmin
print(zmin, zmax, zrange)
# set zero at min
ofilename = fname[:-4] + "_colors.csv"

print("starting coloring")


with open(ofilename, 'w') as ofile:
    for i in range(len(z)):
        # should be between zero and slightly less than one
        zpct = (z[i] - zmin) / ( zrange)
        # should be between 0 and ncolors
        index = int(round((ncolors - 1) * zpct))
        colorval = colorlist[index]
        r = colorval[0]
        g = colorval[1]
        b = colorval[2]
        ostr = ",".join([str(r), str(g), str(b)]) + "\n"
        ofile.write(ostr)


print("coloring done!")


