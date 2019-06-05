#!/usr/bin/env python2
import numpy as np
from matplotlib import pyplot as plt
import sys
import glob 

x = []
y = []
z = []
id = []

def loadForces(dir):
    fy = []
    fz = []
    file_list = sorted(glob.glob(dir + "/forces/*"))
    for file in file_list:
        with open(file, 'r') as f:
            lines = f.readlines()
            fyline = lines[4].split(",")
            fzline = lines[5].split(",")
            fy.append(abs(float(fyline[2])))
            fz.append(abs(float(fzline[3])))
    return np.array(fy), np.array(fz)

def plotForces(dir, name, marker) :
    fy, fz = loadForces(dir)
    xvals = np.arange(len(fz))
    plt.plot(xvals, fz, marker, label=name)


plotForces("gran_bucket_euler", "euler",'bx-')
plotForces("gran_bucket_chung", "chung",'g^-')
plotForces("gran_bucket_verlet", "verlet",'r.-')
plt.legend()

plt.show()


