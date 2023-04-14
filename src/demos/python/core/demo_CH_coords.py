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

print ("First tutorial for PyChrono: vectors, matrices etc.");


# Load the Chrono::Engine core module!
import pychrono as chrono
try:
    import numpy as np
    from numpy import linalg as LA
except ImportError:
    print("You need NumPy to run this demo!")
        

# Test logging
chrono.GetLog().Bar()
chrono.GetLog() << "result is: " << 11+1.5 << "\n"
chrono.GetLog().Bar()


# Test vectors
my_vect1 = chrono.ChVectorD()
my_vect1.x=5
my_vect1.y=2
my_vect1.z=3
my_vect2 = chrono.ChVectorD(3,4,5)
my_vect4 = my_vect1*10 + my_vect2
my_len = my_vect4.Length()
print ('vect sum   =', my_vect1 + my_vect2)
print ('vect cross =', my_vect1 % my_vect2)
print ('vect dot   =', my_vect1 ^ my_vect2)

# Test quaternions
my_quat = chrono.ChQuaternionD(1,2,3,4)
my_qconjugate = ~my_quat
print ('quat. conjugate  =', my_qconjugate)
print ('quat. dot product=', my_qconjugate ^ my_quat)
print ('quat. product=',     my_qconjugate % my_quat)

# Test matrices and NumPy interoperability
mlist = [[1,2,3,4], [5,6,7,8], [9,10,11,12], [13,14,15,16]]
ma = chrono.ChMatrixDynamicD() 
ma.SetMatr(mlist)   # Create a Matrix from a list. Size is adjusted automatically.
npmat = np.asarray(ma.GetMatr()) # Create a 2D npy array from the list extracted from ChMatrixDynamic
w, v = LA.eig(npmat)  # get eigenvalues and eigenvectors using numpy
mb = chrono.ChMatrixDynamicD(4,4)
prod = v * npmat   # you can perform linear algebra operations with numpy and then feed results into a ChMatrixDynamicD using SetMatr 
mb.SetMatr(v.tolist())    # create a ChMatrixDynamicD from the numpy eigenvectors
mr = chrono.ChMatrix33D()
mr.SetMatr([[1,2,3], [4,5,6], [7,8,9]])
print (mr*my_vect1);


# Test frames -
#  create a frame representing a translation and a rotation
#  of 20 degrees on X axis
my_frame = chrono.ChFrameD(my_vect2, chrono.Q_from_AngAxis(20*chrono.CH_C_DEG_TO_RAD, chrono.ChVectorD(1,0,0)))
my_vect5 = my_vect1 >> my_frame

# Print the class hierarchy of a chrono class
import inspect
inspect.getmro(chrono.ChStreamOutAsciiFile)



# Use the ChFunction classes
my_funct = chrono.ChFunction_Sine(0,0.5,3)
print ('function f(0.2)=', my_funct.Get_y(0.2) )


# Inherit from the ChFunction, from the Python side,
# (do not forget the __init__ constructor)

class MySquareFunct (chrono.ChFunction):
    def __init__(self):
         chrono.ChFunction.__init__(self)
    def Get_y(self,x):
         return x*x


my_funct2 = MySquareFunct()
print ('function f(2) =', my_funct2.Get_y(3) )
print ('function df/dx=', my_funct2.Get_y_dx(3) )
