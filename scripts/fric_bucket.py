#!/usr/bin/env python2
import numpy as np
from matplotlib import pyplot as plt
import sys

heights = []
forces = []

hname = 'heights.txt'
fname = 'forces.txt'

with open(hname, 'r') as f:
    for line in f.readlines():
        heights.append(float(line))

with open(fname, 'r') as f:
    for line in f.readlines():
        forces.append(float(line))

plt.plot(heights,forces,'ko')
plt.xlabel("Bucket $\\frac{h}{L}$")
plt.ylabel("Net bottom force $\\frac{F_x}{mg}$")
plt.show()