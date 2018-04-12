#!/usr/bin/env python2
import matplotlib.pyplot as plt
import numpy as np


times = np.array([1.40, 2.44, 4.96, 6.40, 11.94, 17.13, 23.94, 29.34, 34.84,
                  42.12, 46.62, 51.47, 56.98, 69.15, 81.08, 95.70, 105.28, 114.73])

timesnowrite = np.array([0.52, 0.67, 1.17, 1.41, 2.65, 3.78, 4.89, 6.33, 7.17, 8.59,
                         9.73, 10.86, 11.87, 14.86, 17.94, 20.34, 22.06, 23.99, ])

hoptimes = np.array([2.04, 3.94, 6.73, 9.20, 15.41, 25.94, 28.41, 36.94, 43.21, 50.38,
                     60.99, 63.09, 71.61, 81.68, 97.73, 110.85, 122.21, 135.02, ])
hoptimesnowrite = np.array([0.62, 0.69, 0.95, 1.06, 1.68, 2.32, 2.87, 3.47, 4.09,
                            4.76, 5.36, 5.95, 6.68, 7.80, 8.94, 10.14, 11.36, 12.58, ])


sizes = np.array([11792, 23952, 49792, 61952, 125792, 188864, 251952, 315792, 378864,
                  441952, 505792, 568864, 631952, 758864, 885792, 1011952, 1138864, 1265792])
bodies = np.array([30407, 62164, 151204])
static = np.array([2753.45, 8319.28, 28853.1]) / 40
dynamic = np.array([2375.1, 5903.56, 19932.6]) / 40


plt.plot(sizes, times, 'ro', label="Maxwell GTX Titan X on local machine, 50 fps output")
plt.plot(sizes, hoptimes, 'bo', label="Tesla P100 on remote server, 50 fps output")
plt.plot(sizes, timesnowrite, 'r*', label="Maxwell GTX Titan X on local machine, no file writes")
plt.plot(sizes, hoptimesnowrite, 'b*', label="Tesla P100 on remote server, no file writes")
# plt.plot(dynamic, bodies, label="Chrono::Parallel")
plt.xlabel("# Particles")
plt.ylabel("Simulation time in s")
plt.legend()
plt.title("Chrono::Granular scaling for 1000 timesteps")

plt.show()
