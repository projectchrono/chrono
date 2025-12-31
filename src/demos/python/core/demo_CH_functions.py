# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================

import pychrono as chrono
import errno
import os
import math

#------------------------------------------------------------------------------

# Define a custom function
class MyFunction(chrono.ChFunction):
    def GetVal(self, x):
        y = math.cos(math.pi * x)
        return y

#------------------------------------------------------------------------------

print ('Demonstration of functions for y = f(x)')

# Set output root directory
chrono.SetChronoOutputPath("../DEMO_OUTPUT/")

# Create the output directory
out_dir = chrono.GetChronoOutputPath() + "Functions/"
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
       print("Error creating output directory " )


# Ramp function
# -------------

f_ramp = chrono.ChFunctionRamp()
f_ramp.SetAngularCoeff(0.1)  # angular coefficient
f_ramp.SetStartVal(0.1)   # y value at x = 0

# Evaluate the function and its first derivative at a given value
y = f_ramp.GetVal(10)
yd = f_ramp.GetDer(10)
print(' Ramp function   f(10) = ', y, ', 1st derivative df/dx(10) = ', yd)

# Sine function
# -------------

f_sine = chrono.ChFunctionSine()
f_sine.SetAmplitude(2)     # amplitude
f_sine.SetFrequency(1.5)  # frequency

# Evaluate the function and its derivatives at 101 points in [0,2] and write to file
sine_file = open(out_dir + "/f_sine.out","w+")
for i in range(101):
    x = i / 50.0
    y = f_sine.GetVal(x)
    yd = f_sine.GetDer(x)
    ydd = f_sine.GetDer2(x)
    sine_file.write("%f  %f  %f  %f\n" % (x, y, yd, ydd))
sine_file.close()

# Custom function
# ---------------

# Create a custom function for y = f(pi*x)
f_test = MyFunction()

# Evaluate the function and its derivatives at 101 points in [0,2] and write to file
test_file = open(out_dir + "/f_test.out", "w+")
for i in range(101):
    x = i / 50.0
    y = f_test.GetVal(x)
    yd = f_test.GetDer(x)
    ydd = f_test.GetDer2(x)
    test_file.write("%f  %f  %f  %f\n" % (x, y, yd, ydd))
test_file.close()

# Function sequence
# -----------------

f_seq = chrono.ChFunctionSequence()

f_const_acc1 = chrono.ChFunctionConstAcc()
f_const_acc1.SetDuration(0.5)  # ramp length
f_const_acc1.SetDisplacement(0.3)    # ramp height
f_seq.InsertFunct(f_const_acc1, 0.5, 1, False, False, False, 0)

f_const = chrono.ChFunctionConst()
f_seq.InsertFunct(f_const, 0.4, 1, True, False, False, -1)

f_const_acc2 = chrono.ChFunctionConstAcc()
f_const_acc2.SetDuration(0.6)  # ramp length
f_const_acc2.SetFirstAccelerationEnd(0.3)   # acceleration ends after 30% length
f_const_acc2.SetSecondAccelerationStart(0.7)   # deceleration starts after 70% length
f_const_acc2.SetDisplacement(-0.2)   # ramp height
f_seq.InsertFunct(f_const_acc2, 0.6, 1, True, False, False, -1)

f_seq.Setup();

# Evaluate the function and its derivatives at 101 points in [0,2] and write to file
seq_file = open(out_dir + "/f_seq.out", "w+")
for i in range(101):
    x = i / 50.0
    y = f_seq.GetVal(x)
    yd = f_seq.GetDer(x)
    ydd = f_seq.GetDer2(x)
    seq_file.write("%f  %f  %f  %f\n" % (x, y, yd, ydd))
seq_file.close()

# Repeating sequence
# ------------------

f_part1 = chrono.ChFunctionRamp()
f_part1.SetAngularCoeff(0.50)
f_part2 = chrono.ChFunctionConst()
f_part2.SetConstant(1.0)
f_part3 = chrono.ChFunctionRamp()
f_part3.SetAngularCoeff(-0.50)

f_seq = chrono.ChFunctionSequence()
f_seq.InsertFunct(f_part1, 1.0, 1, True)
f_seq.InsertFunct(f_part2, 1.0, 1., True)
f_seq.InsertFunct(f_part3, 1.0, 1., True)

f_rep_seq = chrono.ChFunctionRepeat(f_seq)
f_rep_seq.SetSliceWidth(3.0)
f_rep_seq.SetSliceStart(0.0)
f_rep_seq.SetSliceShift(3.0)

# Evaluate the function and its derivatives at 101 points in [0,2] and write to file
rep_file = open(out_dir + "/f_rep.out", "w+")
for i in range(1001):
    x = i / 50.0
    y = f_rep_seq.GetVal(x)
    yd = f_rep_seq.GetDer(x)
    ydd = f_rep_seq.GetDer2(x)
    rep_file.write("%f  %f  %f  %f\n" % (x, y, yd, ydd))
rep_file.close()
