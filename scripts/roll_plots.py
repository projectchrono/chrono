import numpy as np
from matplotlib import pyplot as plt
import sys
import glob 

filenums = np.char.array(["{:06d}".format(a) for a in np.arange(0, 201)]) 


def loadVals(filenames):
    absv = []
    wy = []
    for file in filenames:
        with open(file) as f:
            dataline=f.readlines()[1]
            s = dataline.split(",")
            absv.append(float(s[3]))
            wy.append(float(s[5]))
    return np.array(absv), np.array(wy)

def plotVals(dir, radius, roll_str):
    filenames = np.array(dir + "step" + filenums + ".csv")
    absv, wy = loadVals(filenames)
    
    # Multiply by radius to get linear velocity
    wyR = wy * radius
    t = np.arange(0,201) / 100.
    plt.figure()
    plt.plot(t, absv, label="Absv")
    plt.plot(t, wyR, label="wy * R")
    plt.legend()
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (cm / s)")
    plt.title("Ball Roll, radius ={}, {}".format(radius, roll_str))
    plt.savefig("ball_roll_{}_{}.png".format(radius, roll_str.replace(" ", "")))
    

dirs = ["test_results_r1_noroll/",  "test_results_r1_withroll/",  "test_results_r2_noroll/",  "test_results_r2_withroll/"]

plotVals(dirs[0], 1., "No Rolling Resistance")
plotVals(dirs[1], 1., "Constant Torque Resistance")
plotVals(dirs[2], 2., "No Rolling Resistance")
plotVals(dirs[3], 2., "Constant Torque Resistance")

plt.show()