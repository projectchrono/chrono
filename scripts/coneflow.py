import numpy as np
from matplotlib import pyplot as plt
import sys


GRAVITY=9.8

import scipy.signal as signal
# First, design the Buterworth filter
N  = 5    # Filter order
Wn = 0.05 # Cutoff frequency
B, A = signal.butter(N, Wn, output='ba')


D_0 = []
r = []
g = []
rho = []
slopes = []

aperture_offset = 2

def plotFlow(prefix_str, num):
    t = []
    fx = []
    fy = []
    fz = []
    cone_file = prefix_str + str(num) + ".out_processed.txt"
    data_file = prefix_str + str(num) + ".out_data.txt"

    with open(cone_file) as f:
        for line in f.readlines():
            l=line.split()
            t.append(float(l[0]))
            fx.append(abs(float(l[1])))
            fy.append(abs(float(l[2])))
            fz.append(abs(float(l[3])))

    global D_0
    global r
    global g
    global rho
    global slopes

    with open(data_file) as f:
        l = f.readlines()[0] # get just the first line
        s = l.split("_")
        print(s)
        D_0 .append( int(s[1]) - aperture_offset)
        r .append( float(s[2]))
        g .append( int(s[3]))
        rho .append( int(s[4]))

    t = np.array(t)
    fx = np.array(fx)
    fy = np.array(fy)
    fz = np.array(fz)


    print()

    fx_smooth = signal.filtfilt(B,A, fx)
    fz_smooth = signal.filtfilt(B,A, fz)


    force_fit_pts = (t > 1) & (t < 9)

    fzfit = np.polyfit(t[force_fit_pts], fz_smooth[force_fit_pts], 1)
    fzfitfn = np.poly1d(fzfit)

    t_arr = np.linspace(min(t[force_fit_pts]), max(t[force_fit_pts]))
    # plt.plot(t, fz_smooth, label="$D_0=" + str(D_0) + "$: weight on bottom, smoothed")
    # plt.plot(t_arr, fzfitfn(t_arr), label='Linear Fit, W(t) = ' + str(fzfitfn).strip())

    slopes.append(fzfitfn[1])

    return fzfitfn[1]

skip = [50, 75, 100, 125, 150 ]

for i in range(33, 143):
    if i in skip:
        continue
    print (i)
    slope = plotFlow("coneflow-233899_", i)
    print("slope is ", slope)

for i in range(143, 167):
    if i in skip:
        continue
    print (i)
    slope = plotFlow("coneflow-239659_", i)
    print("slope is ", slope)
# 
D_0 = np.array(D_0)
r = np.array(r)
g = np.array(g)
rho = np.array(rho)
slopes = np.array(slopes)

g980rho1 = np.where((580 == g) & (1 == rho))
g980rho2 = np.where((580 == g) & (2 == rho))
g980rho3 = np.where((580 == g) & (3 == rho))
g980rho4 = np.where((580 == g) & (4 == rho))
g980rho5 = np.where((580 == g) & (5 == rho))

D08g180 = np.where((180 == g)  & (8 == D_0))
D08g380 = np.where((380 == g)  & (8 == D_0))
D08g580 = np.where((580 == g)  & (8 == D_0))
D08g780 = np.where((780 == g)  & (8 == D_0))
D08g980 = np.where((980 == g)  & (8 == D_0))

D010grho1 = np.where((1 == rho)  & (18 == D_0))
D010grho2 = np.where((2 == rho)  & (18 == D_0))
D010grho3 = np.where((3 == rho)  & (18 == D_0))
D010grho4 = np.where((4 == rho)  & (18 == D_0))
D010grho5 = np.where((5 == rho)  & (18 == D_0))


plt.figure(1)
plt.plot(g[D010grho1], slopes[D010grho1], "s-", label="rho = 1")
plt.plot(g[D010grho2], slopes[D010grho2], "x-", label="rho = 2")
plt.plot(g[D010grho3], slopes[D010grho3], ".-", label="rho = 3")
plt.plot(g[D010grho4], slopes[D010grho4], "o-", label="rho = 4")
plt.plot(g[D010grho5], slopes[D010grho5], "^-", label="rho = 5")
plt.title("Hopper mass flow rate vs material density")
plt.xlabel("gravity")
plt.ylabel("flow rate")
plt.legend()
plt.grid()

plt.figure(2)
plt.plot(rho[D08g180], slopes[D08g180],  "s-", label="g = 180")
plt.plot(rho[D08g380], slopes[D08g380],  "x-", label="g = 380")
plt.plot(rho[D08g580], slopes[D08g580],  ".-", label="g = 580")
plt.plot(rho[D08g780], slopes[D08g780],  "o-", label="g = 780")
plt.plot(rho[D08g980], slopes[D08g980],  "^-", label="g = 980")
plt.title("Hopper mass flow rate vs gravity")
plt.xlabel("density")
plt.ylabel("flow rate")
plt.legend()
plt.grid()

plt.figure(3)
plt.plot(D_0[g980rho1], slopes[g980rho1], "s-",  label="g = 980, $\\rho = 1$")
plt.plot(D_0[g980rho2], slopes[g980rho2], "x-",  label="g = 980, $\\rho = 2$")
plt.plot(D_0[g980rho3], slopes[g980rho3], ".-",  label="g = 980, $\\rho = 3$")
plt.plot(D_0[g980rho4], slopes[g980rho4], "o-",  label="g = 980, $\\rho = 4$")
plt.plot(D_0[g980rho5], slopes[g980rho5], "^-",  label="g = 980, $\\rho = 5$")
plt.title("Hopper mass flow rate vs aperture opening")
plt.xlabel("aperture opening (cm)")
plt.ylabel("flow rate")
plt.legend()
plt.grid()

plt.show()