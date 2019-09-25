#!/usr/bin/env python3

import matplotlib.pyplot as plt
from numpy import pi

grav = 980.0
fig_num = 1
eps = 0.01
fps = 50.0
dt = 1.0 / fps

infilename = 'dam_break_gathered.csv'
with open(infilename, 'r') as infile:
    lines = infile.readlines()
    
def plot(title, plot_radius, plot_fric):
    global fig_num, eps, grav, lines

    plt.figure(fig_num)
    fig_num += 1
    plt.title(title)
    plt.xlabel('Time (s)')
    plt.ylabel(r'$\frac{f_x}{m g}$ on Cylinder')
    for line in lines:
        t = line.split(',')
        radius = float(t[0])
        density = float(t[1])
        mass = 4.0 * pi * radius**3 * density / 3.0
        
        friction = float(t[2])
        if plot_fric - eps < friction < plot_fric + eps and plot_radius - eps < radius < plot_radius + eps:
            force_x = [float(fx) / (mass * grav) for fx in t[3:]]
            plt.plot([n * dt for n in range(len(force_x))], force_x, label=r'$\rho =$ {}'.format(density))
        
    plt.legend()
    plt.savefig('DamBreakPlot_r{}_fric{}.pdf'.format(plot_radius, plot_fric), dpi=400, bbox_inches='tight')

radii = [0.5, 1.0, 2.0]

# Frictionless
for radius in radii:
    plot('Dam Break with Cylinder - Frictionless - Radius {}'.format(radius), radius, 0.0)
    plot(r'Dam Break with Cylinder - $\mu = 0.5$ - Radius {}'.format(radius), radius, 0.5)