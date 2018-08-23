#!/usr/bin/env python2
import matplotlib.pyplot as plt


sizes = np.array([4992, 15552, 31680, 71040, 90432, 182784, 381888, 777600, 969408, 1973760, 3945024, 7990656,
                  10010304, 20056512, 40132224, 80436096, 100781184, 202121280, 252269568, 303027072, 353443392])

hoptimes4 = np.array([0.81, 0.79, 0.92, 1.44, 1.69, 2.65, 4.96, 9.15, 11.22, 21.91, 43.41, 87.59,
                      109.93, 217.43, 436.19, 879.47, 1161.48, 2972.85, 8582.76, 12606.40, 26594.10])

fit = np.poly1d(np.polyfit(sizes[-4:], hoptimes4[-4:], 2, w=sizes[-4:] ** 2))
fit2 = np.poly1d(np.polyfit(sizes, hoptimes4, 2, w=sizes ** 2))
s = np.linspace(sizes[0], sizes[-1], 100)

plt.plot(sizes, hoptimes4, 'b*', label="Tesla P100 on remote server,no file writes")
plt.plot(s, fit(s), 'r-', label="Quadratic fit")
plt.plot(s, fit2(s), 'g+', label="Quadratic fit 2")
print fit(1000000000)
print fit2(1000000000)
# plt.plot(dynamic,bodies,label="Chrono::Parallel")
plt.xlabel("# Particles")
plt.ylabel("Simulation time in s")
plt.legend()
plt.title("Chrono::Granular scaling for 1000 timesteps")

plt.show()
