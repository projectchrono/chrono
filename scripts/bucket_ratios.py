#!/usr/bin/env python2
import numpy as np
from matplotlib import pyplot as plt
import sys

height = []
ratio = []

fname = sys.argv[1]

with open(fname, 'r') as f:
    for line in f.readlines():
        l=line.split(" ")

        height.append(float(l[0]) / 100)
        ratio.append(float(l[1]))

plt.plot(height,ratio,'ko')
plt.xlabel("Bucket $\\frac{h}{L}$")
plt.ylabel("Confining Force ratio $\\frac{F_x}{mg}$")
plt.show()