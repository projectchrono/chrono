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
    def Get_y(self, x):
        y = math.cos(math.pi * x)
        return y

#------------------------------------------------------------------------------

print ('Demonstration of functions for y = f(x)')

# Create the output directory
out_dir = os.path.join(os.path.dirname(__file__), "Functions_demo")
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
       print("Error creating output directory " )


# Ramp function
# -------------

f_ramp = chrono.ChFunction_Ramp()
f_ramp.Set_ang(0.1)  # angular coefficient
f_ramp.Set_y0(0.1)   # y value at x = 0

# Evaluate the function and its first derivative at a given value
y = f_ramp.Get_y(10)
yd = f_ramp.Get_y_dx(10)
print(' Ramp function   f(10) = ', y, ', 1st derivative df/dx(10) = ', yd)

# Sine function
# -------------

f_sine = chrono.ChFunction_Sine()
f_sine.Set_amp(2)     # amplitude
f_sine.Set_freq(1.5)  # frequency

# Evaluate the function and its derivatives at 101 points in [0,2] and write to file
sine_file = open(out_dir + "/f_sine.out","w+")
for i in range(101):
    x = i / 50.0
    y = f_sine.Get_y(x)
    yd = f_sine.Get_y_dx(x)
    ydd = f_sine.Get_y_dxdx(x)
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
    y = f_test.Get_y(x)
    yd = f_test.Get_y_dx(x)
    ydd = f_test.Get_y_dxdx(x)
    test_file.write("%f  %f  %f  %f\n" % (x, y, yd, ydd))
test_file.close()

# Function sequence
# -----------------

f_seq = chrono.ChFunction_Sequence()

f_const_acc1 = chrono.ChFunction_ConstAcc()
f_const_acc1.Set_end(0.5)  # ramp length
f_const_acc1.Set_h(0.3)    # ramp height
f_seq.InsertFunct(f_const_acc1, 0.5, 1, False, False, False, 0)

f_const = chrono.ChFunction_Const()
f_seq.InsertFunct(f_const, 0.4, 1, True, False, False, -1)

f_const_acc2 = chrono.ChFunction_ConstAcc()
f_const_acc2.Set_end(0.6)  # ramp length
f_const_acc2.Set_av(0.3)   # acceleration ends after 30% length
f_const_acc2.Set_aw(0.7)   # deceleration starts after 70% length
f_const_acc2.Set_h(-0.2)   # ramp height
f_seq.InsertFunct(f_const_acc2, 0.6, 1, True, False, False, -1)

f_seq.Setup();

# Evaluate the function and its derivatives at 101 points in [0,2] and write to file
seq_file = open(out_dir + "/f_seq.out", "w+")
for i in range(101):
    x = i / 50.0
    y = f_seq.Get_y(x)
    yd = f_seq.Get_y_dx(x)
    ydd = f_seq.Get_y_dxdx(x)
    seq_file.write("%f  %f  %f  %f\n" % (x, y, yd, ydd))
seq_file.close()

# Repeating sequence
# ------------------

f_part1 = chrono.ChFunction_Ramp()
f_part1.Set_ang(0.50)
f_part2 = chrono.ChFunction_Const()
f_part2.Set_yconst(1.0)
f_part3 = chrono.ChFunction_Ramp()
f_part3.Set_ang(-0.50)

f_seq = chrono.ChFunction_Sequence()
f_seq.InsertFunct(f_part1, 1.0, 1, True)
f_seq.InsertFunct(f_part2, 1.0, 1., True)
f_seq.InsertFunct(f_part3, 1.0, 1., True)

f_rep_seq = chrono.ChFunction_Repeat()
f_rep_seq.Set_fa(f_seq)
f_rep_seq.Set_window_length(3.0)
f_rep_seq.Set_window_start(0.0)
f_rep_seq.Set_window_phase(3.0)

# Evaluate the function and its derivatives at 101 points in [0,2] and write to file
rep_file = open(out_dir + "/f_rep.out", "w+")
for i in range(1001):
    x = i / 50.0
    y = f_rep_seq.Get_y(x)
    yd = f_rep_seq.Get_y_dx(x)
    ydd = f_rep_seq.Get_y_dxdx(x)
    rep_file.write("%f  %f  %f  %f\n" % (x, y, yd, ydd))
rep_file.close()
