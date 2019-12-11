#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from numpy import pi

grav = 980.0
fig_num = 1
eps = 0.01
fps = 50.0
dt = 1.0 / fps
ymin,ymax = -2000,4000

infilename = 'dam_break_gathered.csv'
with open(infilename, 'r') as infile:
    lines = infile.readlines()
    
def plot(title, plot_radius, plot_fric):
    global fig_num, eps, grav, lines

    rhos = []
    times = []
    forces = []
    
    plt.figure(fig_num)
    fig_num += 1
    plt.title(title)
    plt.xlabel('Time (s)')
    plt.ylabel(r'$\frac{f_x}{\rho}$ on Cylinder')
    plt.ylim((ymin,ymax))
    for line in lines:
        t = line.split(',')
        radius = float(t[0])
        density = float(t[1])
        
        friction = float(t[2])
        if plot_fric - eps < friction < plot_fric + eps and plot_radius - eps < radius < plot_radius + eps:
            force_x = [float(fx) / density for fx in t[3:]]
            forces.append(force_x)
            times.append([n * dt for n in range(len(force_x))])
            rhos.append(density)
            
    forces = np.array(forces)
    times = np.array(times)
    rhos = np.array(rhos)
    
    # Sort by rho
    sorted_i = np.argsort(rhos)
    forces = forces[sorted_i]
    times = times[sorted_i]
    rhos = rhos[sorted_i]
    
    # Plot each density
    for i,rho in enumerate(rhos):
        plt.plot(times[i], forces[i], label=r'$\rho =$ {} $g/cm^3$'.format(rho))
        
    plt.legend()
    plt.savefig('DamBreakPlot_r{}_fric{}.pdf'.format(plot_radius, plot_fric), dpi=400, bbox_inches='tight')

radii = [0.5, 1.0, 2.0]

# Frictionless
for radius in radii:
    plot('Dam Break with Cylinder - Frictionless - Radius {}'.format(radius), radius, 0.0)
    plot(r'Dam Break with Cylinder - $\mu = 0.5$ - Radius {}'.format(radius), radius, 0.5)