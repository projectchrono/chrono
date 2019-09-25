#!/usr/bin/env python3

import sys
import re

if len(sys.argv) != 2:
    print('usage: ' + sys.argv[0] + 'inputfilename (job output)')
    exit(1)

infilename = sys.argv[1]
outfilename = 'dam_break_gathered.csv'

radius = -1
density = -1
friction = -1
force_x = []

with open(infilename, 'r') as infile:
    for line in infile.readlines():
        if 'Radius' in line:
            radius = line.split()[1]
        if 'Density' in line:
            density = line.split()[1]
        if 'Run Mode' in line:
            run_mode = int(line.split()[2])
            friction = 0.0 if run_mode == 1 else 0.5
        if 'cyl force' in line:
            force_x.append(re.search('\(.*\)', line)[0][1:-1].split(',')[0])
            
with open(outfilename, 'a') as outfile:
    outfile.write('{},{},{},'.format(radius, density, friction))
    outfile.write(','.join(force_x))
    outfile.write('\n')