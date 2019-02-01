import numpy as np
from matplotlib import pyplot as plt
from matplotlib import rcParams
rcParams.update({'figure.autolayout': True})

PARTICLE_RADIUS=0.5
PARTICLE_DIAM=2 * PARTICLE_RADIUS
EARTH_GRAV=9.80
RHO_CGS_TO_SI = 1000

G_CGS_TO_SI = 1. / 100.

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

def getIndices(g_=None, rho_=None, D0_ = None, rad_ = None):
    ret = np.array([1] * len(g))
    if g_ is not None:   
        ret = ret & np.isclose(g, g_) 

    if rho_ is not None:   
        ret = ret & np.isclose(rho, rho_)

    if D0_ is not None:   
        ret = ret & np.isclose(D_0, D0_)

    if rad_ is not None:   
        ret = ret & np.isclose(rad, rad_)
    return ret.nonzero()

def plotInstance(t, t_arr, vals, function, D0):

    plt.plot(t, vals, label="$D_0=" + str(D0) + "$: weight on bottom, smoothed")
    plt.plot(t_arr, function(t_arr), label='Linear Fit, W(t) = ' + str(function).strip())
    plt.title("Force on bottom vs time")
    plt.xlabel("Time (s)")
    plt.ylabel("Force (N)")
    plt.legend()

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

    D0 = 0

    with open(data_file) as f:
        l = f.readlines()[0] # get just the first line
        s = l.split("_")
        D0 = float(s[1])

        D_0 .append( D0 )
        r .append( float(s[2]))
        g .append( G_CGS_TO_SI * float(s[3] ) )
        rho .append( float(s[4]) * RHO_CGS_TO_SI)

    t = np.array(t)
    fx = np.array(fx)
    fy = np.array(fy)
    fz = np.array(fz)


    fx_smooth = signal.filtfilt(B,A, fx)
    fz_smooth = signal.filtfilt(B,A, fz)


    fmin, fmax = np.min(fz_smooth), np.max(fz_smooth)
    force_fit_pts = (fz_smooth > 0.1 * abs(fmax)) & (fz_smooth < 0.9  * fmax)

    t_arr = np.linspace(min(t[force_fit_pts]), max(t[force_fit_pts]))
    fzfit = np.polyfit(t[force_fit_pts], fz_smooth[force_fit_pts], 1)
    fzfitfn = np.poly1d(fzfit)

    if (num == 199):
        plotInstance(t, t_arr, fz_smooth, fzfitfn, D0)


    slopes.append(fzfitfn[1])

    return fzfitfn[1]

skip = [50, 75, 100, 125, 150, 175, 200 ]

plt.figure(0)


for i in range(33, 143):
    if i in skip:
        continue
    slope = plotFlow("coneflow-233899_", i)
    print("slope is ", slope)

for i in range(143, 223):
    if i in skip:
        continue
    slope = plotFlow("coneflow-239659_", i)
    print("slope is ", slope)
# 
D_0 = np.array(D_0)
r = np.array(r)
g = np.array(g)
rho = np.array(rho)
slopes = np.array(slopes)



massflow = slopes / g






plt.figure(1)
plt.plot(np.sqrt(g[getIndices(rho_=1000, D0_ = 10)] / EARTH_GRAV), massflow[getIndices(rho_=1000, D0_ = 10)], "s-", label="$rho = 1000, D_0 = 10$")
plt.plot(np.sqrt(g[getIndices(rho_=2000, D0_ = 10)] / EARTH_GRAV), massflow[getIndices(rho_=2000, D0_ = 10)], "x-", label="$rho = 2000, D_0 = 10$")
plt.plot(np.sqrt(g[getIndices(rho_=3000, D0_ = 10)] / EARTH_GRAV), massflow[getIndices(rho_=3000, D0_ = 10)], ".-", label="$rho = 3000, D_0 = 10$")
plt.plot(np.sqrt(g[getIndices(rho_=4000, D0_ = 10)] / EARTH_GRAV), massflow[getIndices(rho_=4000, D0_ = 10)], "o-", label="$rho = 4000, D_0 = 10$")
plt.plot(np.sqrt(g[getIndices(rho_=5000, D0_ = 10)] / EARTH_GRAV), massflow[getIndices(rho_=5000, D0_ = 10)], "^-", label="$rho = 5000, D_0 = 10$")
plt.title("Hopper mass flow rate vs gravity")
plt.xlabel("$\\sqrt{\\frac{g}{g_{earth}}}$")
plt.ylabel("Mass flow rate (kg / s)")
plt.legend()
plt.grid()

plt.figure(2)
plt.plot(rho[getIndices(g_ = 1.80, D0_ = 10)], massflow[getIndices(g_ = 1.80, D0_ = 10)],  "s-", label="$g = 1.80, D_0 = 10$")
plt.plot(rho[getIndices(g_ = 3.80, D0_ = 10)], massflow[getIndices(g_ = 3.80, D0_ = 10)],  "x-", label="$g = 3.80, D_0 = 10$")
plt.plot(rho[getIndices(g_ = 5.80, D0_ = 10)], massflow[getIndices(g_ = 5.80, D0_ = 10)],  ".-", label="$g = 5.80, D_0 = 10$")
plt.plot(rho[getIndices(g_ = 7.80, D0_ = 10)], massflow[getIndices(g_ = 7.80, D0_ = 10)],  "o-", label="$g = 7.80, D_0 = 10$")
plt.plot(rho[getIndices(g_ = 9.80, D0_ = 10)], massflow[getIndices(g_ = 9.80, D0_ = 10)],  "^-", label="$g = 9.80, D_0 = 10$")

plt.title("Hopper mass flow rate vs material density")
plt.xlabel("$\\rho_{mat}$ (kg / m$^3$)")
plt.ylabel("Mass flow rate (kg / s)")
plt.legend()
plt.grid()

plt.figure(3)
plt.plot(pow(D_0[ getIndices(g_=1.80, rho_ = 2000)] / PARTICLE_DIAM, 5/2), massflow[ getIndices(g_=1.80, rho_ = 2000)], "s-",  label="g = 1.80, $\\rho = 2000$")
plt.plot(pow(D_0[ getIndices(g_=3.80, rho_ = 2000)] / PARTICLE_DIAM, 5/2), massflow[ getIndices(g_=3.80, rho_ = 2000)], "x-",  label="g = 3.80, $\\rho = 2000$")
plt.plot(pow(D_0[ getIndices(g_=5.80, rho_ = 2000)] / PARTICLE_DIAM, 5/2), massflow[ getIndices(g_=5.80, rho_ = 2000)], ".-",  label="g = 5.80, $\\rho = 2000$")
plt.plot(pow(D_0[ getIndices(g_=7.80, rho_ = 2000)] / PARTICLE_DIAM, 5/2), massflow[ getIndices(g_=7.80, rho_ = 2000)], "o-",  label="g = 7.80, $\\rho = 2000$")
plt.plot(pow(D_0[ getIndices(g_=9.80, rho_ = 2000)] / PARTICLE_DIAM, 5/2), massflow[ getIndices(g_=9.80, rho_ = 2000)], "^-",  label="g = 9.80, $\\rho = 2000$")
plt.title("Hopper mass flow rate vs aperture opening")
plt.xlabel("$\\left(\\frac{D_0}{d_p}\\right) ^ \\frac{5}{2}$")
plt.ylabel("Mass flow rate (kg / s)")
plt.legend()
plt.grid()

# 
# plt.figure(5)
# plotInstance()

plt.show()
