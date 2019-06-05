import numpy as np
from matplotlib import pyplot as plt

t = []
vx = []
vy = []
vz = []

fname="Data-212846.out"

with open(fname) as f:
    for line in f.readlines():
        l=line.split()
        t.append(float(l[0]) - 1)
        vx.append(float(l[1]))
        vy.append(float(l[2]))
        vz.append(float(l[3]))

plt.plot(t, vx)
plt.xlabel("time (s)")
plt.ylabel("$F_x$ on cylinder (N)")
plt.title("Dam break with cylinder")
plt.show()