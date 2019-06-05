#!/usr/bin/env python2
import numpy as np
import sys

x = []
y = []
z = []

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

print("Max height is ", np.max(z), " min is ", np.min(z))
