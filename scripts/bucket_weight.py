import numpy as np
from matplotlib import pyplot as plt

t = []
fx = []
fy = []
fz = []

GRAVITY=9.8

import scipy.signal as signal
# First, design the Buterworth filter
N  = 5    # Filter order
Wn = 0.1 # Cutoff frequency
B, A = signal.butter(N, Wn, output='ba')

# fname="cone_flow.txt"
fname="cone_data.txt"

with open(fname) as f:
    for line in f.readlines():
        l=line.split()
        t.append(float(l[0]))
        fx.append(abs(float(l[1])))
        fy.append(abs(float(l[2])))
        fz.append(abs(float(l[3])))

t = np.array(t)
fx = np.array(fx)
fy = np.array(fy)
fz = np.array(fz)

fx_smooth = signal.filtfilt(B,A, fx)
fz_smooth = signal.filtfilt(B,A, fz)


force_fit_pts = (t > 1) & (t < 9)

fzfit = np.polyfit(t[force_fit_pts], fz_smooth[force_fit_pts], 1)
fzfitfn = np.poly1d(fzfit)

t_arr = np.linspace(min(t[force_fit_pts]), max(t[force_fit_pts]))
plt.plot(t, fz, label="Weight on bottom, smoothed")
plt.plot(t_arr, fzfitfn(t_arr), label='Linear Fit, W(t) = ' + str(fzfitfn).strip())


plt.xlabel("time (s)")
plt.ylabel("Material weight on bottom plane (N)")
plt.grid()
plt.legend()
plt.title("Hopper mass flow rate")
plt.show()