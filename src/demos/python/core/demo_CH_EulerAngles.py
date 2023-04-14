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
import os
import math

try:
    import numpy as np
    from numpy import linalg as LA
except ImportError:
    print("You need NumPy to run this demo!")

#------------------------------------------------------------------------------

print ('Test Euler sequence 1-2-3')

q = chrono.ChQuaternionD()
eu = chrono.ChVectorD()

alpha1 = 10
beta1 = 11
gamma1 = 12

alpha2 = -17.3
beta2 = -41
gamma2 = -0.7

# --------
print("  Rotations about frame axes:")

print("    Rotation about X of ", alpha1, " deg")
q.Q_from_AngX(chrono.CH_C_DEG_TO_RAD * alpha1)
eu = q.Q_to_Euler123() * chrono.CH_C_RAD_TO_DEG
print("    eu = {", eu.x, ";", eu.y, ";", eu.z, "}")

print("    Rotation about Y of ", beta1, " deg")
q.Q_from_AngY(chrono.CH_C_DEG_TO_RAD * beta1)
eu = q.Q_to_Euler123() * chrono.CH_C_RAD_TO_DEG
print("    eu = {", eu.x, ";", eu.y, ";", eu.z, "}")

print("    Rotation about Z of ", gamma1, " deg")
q.Q_from_AngZ(chrono.CH_C_DEG_TO_RAD * gamma1)
eu = q.Q_to_Euler123() * chrono.CH_C_RAD_TO_DEG
print("    eu = {", eu.x, ";", eu.y, ";", eu.z, "}")


print("    Rotation about X of ", alpha2, " deg")
q.Q_from_AngX(chrono.CH_C_DEG_TO_RAD * alpha2)
eu = q.Q_to_Euler123() * chrono.CH_C_RAD_TO_DEG
print("    eu = {", eu.x, ";", eu.y, ";", eu.z, "}")

print("    Rotation about Y of ", beta2, " deg")
q.Q_from_AngY(chrono.CH_C_DEG_TO_RAD * beta2)
eu = q.Q_to_Euler123() * chrono.CH_C_RAD_TO_DEG
print("    eu = {", eu.x, ";", eu.y, ";", eu.z, "}")

print("    Rotation about Z of ", gamma2, " deg")
q.Q_from_AngZ(chrono.CH_C_DEG_TO_RAD * gamma2)
eu = q.Q_to_Euler123() * chrono.CH_C_RAD_TO_DEG
print("    eu = {", eu.x, ";", eu.y, ";", eu.z, "}")

# --------
print("  Quaternion from Euler angles:")

eu.x = alpha1
eu.y = beta1
eu.z = gamma1
print("    Input = {", eu.x, ";", eu.y, ";", eu.z, "}")
q.Q_from_Euler123(eu * chrono.CH_C_DEG_TO_RAD)
eu = q.Q_to_Euler123() * chrono.CH_C_RAD_TO_DEG
print("    Output = {", eu.x, ";", eu.y, ";", eu.z, "}")

eu.x = alpha2
eu.y = beta2
eu.z = gamma2
print("    Input = {", eu.x, ";", eu.y, ";", eu.z, "}")
q.Q_from_Euler123(eu * chrono.CH_C_DEG_TO_RAD)
eu = q.Q_to_Euler123() * chrono.CH_C_RAD_TO_DEG
print("    Output = {", eu.x, ";", eu.y, ";", eu.z, "}")

# --------
print("  Rotation matrix for sequence (90, 90, 90)")
eu.x = math.pi / 2
eu.y = math.pi / 2
eu.z = math.pi / 2
q.Q_from_Euler123(eu)
R = chrono.ChMatrix33D(q)

 # Create a 2D npy array from the list extracted from ChMatrix33
npmat = np.asarray(R.GetMatr())
np.set_printoptions(suppress=True)
print(npmat)
