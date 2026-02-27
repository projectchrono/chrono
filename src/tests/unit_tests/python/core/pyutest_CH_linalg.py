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
# Authors: Radu Serban
# =============================================================================
#
# Tests for miscellaneous linear algebra support.
#
# =============================================================================

import pytest
import pychrono as chrono
import numpy as np
import math

ABS_ERR = 1e-10

def test_create_assign():
    Md1 = chrono.ChMatrixDynamicd(5, 7)
    Md2 = chrono.ChMatrixDynamicd(4, 4)
    A33 = chrono.ChMatrix33d(chrono.QuatFromAngleY(0.4))

    for i in range(4): # Fill a matrix with an element
        for j in range(4):
            Md2[i,j] = 0.1
    max_coeff = max(Md2[i, j] for i in range(4) for j in range(4))
    min_coeff = min(Md2[i, j] for i in range(4) for j in range(4))
    assert max_coeff == pytest.approx(0.1, abs = ABS_ERR)
    assert min_coeff == pytest.approx(0.1, abs = ABS_ERR)

    np_random = np.random.rand(5, 7) # initialize with random numbers using numpy
    for i in range(5):
        for j in range(7):
            Md1[i, j] = np_random[i, j]
    e32 = Md1[3, 2]
    Md1_transpose = chrono.ChMatrixDynamicd(7, 5) # create new tranposed matrix
    for i in range(7):
        for j in range(5):
            Md1_transpose[i, j] = Md1[j, i]
    assert (Md1_transpose[2, 3] == pytest.approx(e32, abs = ABS_ERR)) 

    Md1 = chrono.ChMatrixDynamicd(2, 2) # resize
    Md1.SetZero() # set all elements to zero
    max = max(Md1[i, j] for i in range(2) for j in range(2))
    min = min(Md1[i, j] for i in range(2) for j in range(2))
    assert max == pytest.approx(0.0, abs = ABS_ERR)
    assert min == pytest.approx(0.0, abs = ABS_ERR)

    Md2 = chrono.ChMatrixDynamicd(4, 4)
    Md2.SetZero()
    for i in range(4): # Create a diagonal matrix
        Md2[i, i] = 3.0
    Md2_trace = sum(Md2[i, i] for i in range(4))
    assert Md2_trace == pytest.approx(4 * 3.0, abs = ABS_ERR)

    M1 = chrono.ChMatrixDynamicd(4, 4) # Copy constructor
    M2 = chrono.ChMatrixDynamicd(4, 4) # Negated matrix
    for i in range(4):
        for j in range(4):
            M1[i, j] = Md2[i, j]
            M2[i, j] = -Md2[i, j]  
    M3 = chrono.ChMatrixDynamicd(4, 4) # 4x4 uninitialized matrix
    for i in range(4): # Copy (resize as needed)
        for j in range(4):
            M3[i, j] = M1[i, j]
    M1_transpose = chrono.ChMatrixDynamicd(4, 4)
    for i in range(4):
        for j in range(4):
            M1_transpose[i, j] = M1[j, i]
    M3 = M1_transpose # transposed copy.
    M2_transpose = chrono.ChMatrixDynamicd(4, 4)
    for i in range(4):
        for j in range(4):
            M2_transpose[i, j] = M2[j, i]
    is_zero = all(abs(M3[i, j] + M2_transpose[i, j]) < ABS_ERR for i in range(4) for j in range(4))
    assert is_zero == True

    B33 = chrono.ChMatrix33d(1.0)
    print("3x3 identity matrix\n", B33)
    B33_trace = sum(B33[i, i] for i in range(3))
    assert B33_trace == pytest.approx(3.0, abs=ABS_ERR)

def test_operations():
    A = chrono.ChMatrixDynamicd(2, 3)
    B = chrono.ChMatrixDynamicd(2, 3)
    np.random.seed(42)
    for i in range(2):
        for j in range(3):
            A[i, j] = np.random.rand()
            B[i, j] = np.random.rand()
    
    sum1 = chrono.ChMatrixDynamicd(2, 3)
    for i in range(2):
        for j in range(3):
            sum1[i, j] = A[i, j] + B[i, j]
    sum2 = chrono.ChMatrixDynamicd(2, 3)
    for i in range(2):
        for j in range(3):
            sum2[i, j] = A[i, j]
            sum2[i, j] += B[i, j]
    assert sum2 == sum1

    # Different ways to do subtraction..
    diff1 = chrono.ChMatrixDynamicd(2, 3)
    for i in range(2):
        for j in range(3):
            diff1[i, j] = A[i, j] - B[i, j]
    diff2 = chrono.ChMatrixDynamicd(2, 3)
    for i in range(2):
        for j in range(3):
            diff2[i, j] = A[i, j]
            diff2[i, j] -= B[i, j]
    assert diff1 == diff2

    # Multiplication with scalar
    ms1 = chrono.ChMatrixDynamicd(2, 3)
    for i in range(2):
        for j in range(3):
            ms1[i, j] = A[i, j] * 10
    ms2 = chrono.ChMatrixDynamicd(2, 3)
    for i in range(2):
        for j in range(3):
            ms2[i, j] = 10 * A[i, j]
    ms3 = chrono.ChMatrixDynamicd(2, 3)
    for i in range(2):
        for j in range(3):
            ms3[i, j] = A[i, j]
            ms3[i, j] *= 10
    assert ms2 == ms1
    assert ms3 == ms1
    
    # Matrix multiplications
    C = chrono.ChMatrixDynamicd(3, 2)
    D = chrono.ChMatrixDynamicd(2, 3)
    C[0, 0] = 1; C[0, 1] = 2; C[1, 0] = 3; C[1, 1] = 4; C[2, 0] = 5; C[2, 1] = 6;
    D[0, 0] = 1; D[0, 1] = 2; D[0, 2] = 3; D[1, 0] = 4; D[1, 1] = 5; D[1, 2] = 6;
    R = chrono.ChMatrix33d(chrono.QuatFromAngleX(chrono.CH_PI_2))

    CD = chrono.ChMatrixDynamicd(3, 3)
    for i in range(3):
        for j in range(3):
            val = 0.0
            for k in range(2):
                val += C[i, k] * D[k, j]
            CD[i, j] = val
    CD_t = chrono.ChMatrixDynamicd(3, 3)
    for i in range(3):
        for j in range(3):
            val = 0.0
            for k in range(2):
                val += D[k, i] * C[j, k]
            CD_t[i, j] = val
    RC = chrono.ChMatrixDynamicd(3, 2)
    for i in range(3):
        for j in range(2):
            val = 0.0
            for k in range(3):
                val += R[i, k] * C[k, j]
            RC[i, j] = val
    DR = chrono.ChMatrixDynamicd(2, 3)
    for i in range(2):
        for j in range(3):
            val = 0.0
            for k in range(3):
                val += D[i, k] * R[k, j]
            DR[i, j] = val
    print(R, "\n")
    print("rot * matrix\n", RC, "\n")
    print("matrix * rot\n", DR, "\n")
    for i in range(3):
        for j in range(3):
            assert CD[j, i] == pytest.approx(CD_t[i, j], abs = ABS_ERR)
    assert RC[1, 0] == pytest.approx(-C[2, 0], abs = ABS_ERR)
    assert DR[1, 2] == pytest.approx(-D[1, 1], abs = ABS_ERR)

    # Component-wise matrix multiplication and division
    D_t = chrono.ChMatrixDynamicd(3, 2)
    for i in range(3):
        for i in range(2):
            D_t[i, j] = D[j, i]
    C_times_D = chrono.ChMatrixDynamicd(3, 2)
    C_div_D = chrono.ChMatrixDynamicd(3, 2)
    for i in range(3):
        for j in range(2):
            C_times_D[i, j] = C[i, j] * D_t[i, j]
            C_div_D[i, j] = C[i, j] / D_t[i, j]
    print("C\n", C, "\n")
    print("D'\n", D_t, "\n")
    print("C_times_D' (component-wise)\n", C_times_D, "\n")
    print("C_div_D' (component-wise)\n", C_div_D, "\n")

    # Assigning matrix rows
    J = chrono.ChMatrix33d()
    J[0,0], J[0,1], J[0,2] = 10, 20, 30; 
    J[1,0], J[1,1], J[1,2] = 40, 50, 60; 
    J[2,0], J[2,1], J[2,2] = 70, 80, 90;
    print("3x3 matrix J\n", J, "\n")
    V = chrono.ChVectorDynamicd(10)
    for i in range(10):
        V[i] = 0.0
    for k in range(3):
        V[3 + k] = J[0, k]
    print("Place row0 in V starting at index 3\n", [V[i] for i in range(10)], "\n")
    for k in range(3):
        V[7 + k] = J[1, k]
    print("Place row1 in V starting at index 7\n", [V[i] for i in range(10)], "\n")

def test_vector_rotation():
    q = chrono.ChQuaterniond(1, 2, 3, 4)
    q.Normalize()
    A = chrono.ChMatrix33d(q)

    v1 = chrono.ChVector3d(1, 2, 3)
    v1_vals = [v1.x, v1.y, v1.z]
    v2_x = sum(A[0, k] * v1_vals[k] for k in range(3))
    v2_y = sum(A[1, k] * v1_vals[k] for k in range(3))
    v2_z = sum(A[2, k] * v1_vals[k] for k in range(3))
    v2 = chrono.ChVector3d(v2_x, v2_y, v2_z)
    v3_x = A[0, 0] * v2.x + A[0, 1] * v2.y + A[0, 2] * v2.z
    v3_y = A[1, 0] * v2.x + A[1, 1] * v2.y + A[1, 2] * v2.z
    v3_z = A[2, 0] * v2.x + A[2, 1] * v2.y + A[2, 2] * v2.z
    v3 = chrono.ChVector3d(v3_x, v3_y, v3_z)

    print(A, "\n")
    print(v1, "\n")
    print(v2, "\n")
    print(v3, "\n")
    assert abs(v3.x - v1.x) < 1e-8 and abs(v3.y - v1.y) < 1e-8 and abs(v3.z - v1.z) < 1e-8

def test_extensions():
    A = chrono.ChMatrixDynamicd(2, 3)
    np.random.seed(43)
    for i in range(2):
        for j in range(3):
            A[i, j] = np.random.rand()
    print("random 2x3 matrix A:\n", A, "\n")
    for i in range(min(2, 3)):
        A[i, i] = 10.1
    print("fill diagonal with 10.1:\n", A, "\n")
    for i in range(2):
        for j in range(3):
            A[i, j] = 2.1
    print("fill entire matrix with 2.1:\n", A, "\n")

    B = chrono.ChMatrixDynamicd(2, 3)
    for i in range(2):
        for j in range(3):
            B[i, j] = A[i, j]
    B[1, 2] += 0.01
    print("matrix B = A with B(1,2) incremented by 0.01\n", B, "\n")
    diff_norm = math.sqrt(sum((A[i, j] - B[i, j]) ** 2 for i in range(2) for j in range(3)))
    print("|A-B| < 0.1?   ", diff_norm < 0.1, "\n")
    print("|A-B| < 0.001? ", diff_norm < 0.001, "\n")
    assert diff_norm < 0.1
    assert diff_norm >= 0.001

    v = chrono.ChVectorDynamicd(3)
    v[0], v[1], v[2] = 2, 3, 4
    w = chrono.ChVectorDynamicd(3)
    w[0], w[1], w[2] = 0.1, 0.2, 0.3
    wrms_ref = math.sqrt((0.2 * 0.2 + 0.6 * 0.6 + 1.2 * 1.2) / 3)
    if hasattr(v, "wrmsNorm"):
        wrms_val = v.wrmsNorm(w)
    else:
        wrms_val = math.sqrt(sum((v[i] * w[i]) ** 2 for i in range(3)) / 3)
    print("||v||_wrms, w = ", wrms, "\n")
    v_plus_v = chrono.ChVectorDynamicd(3)
    for i in range(3):
        v_plus_v[i] = v[i] + v[i]
    if hasattr(v_plus_v, "wrmsNorm"):
        wrms_2 = v_plus_v.wrmsNorm(w)
    else:
        wrms_2 = math.sqrt(sum((v_plus_v[i] * w[i]) ** 2 for i in range(3)) / 3)
    print("||v + v||_wrms, w = ", wrms2, "\n")
    assert wrms_val == pytest.approx(wrms_ref, abs = ABS_ERR)
    assert wrms_2 == pytest.approx(2 * wrms_ref, abs = ABS_ERR)

    v_plus_v_plus_1 = chrono.ChVectorDynamicd(3)
    for i in range(3):
        v_plus_v_plus_1[i] = v[i] + v[i] + 1
    print("v + v + 1: ", [v_plus_v_plus_1[i] for i in range(3)], "\n")
    one_plus_v = chrono.ChVectorDynamicd(3)
    for i in range(3):
        one_plus_v[i] = 1 + v[i]
    print("1 + v: ", [one_plus_v[i] for i in range(3)], "\n")

def test_pasting():
    A = chrono.ChMatrixDynamicd(4, 6)
    np.random.seed(44)
    for i in range(4):
        for j in range(6):
            A[i, j] = np.random.rand()
    print(A, "\n\n")

    B = chrono.ChMatrixDynamicd(2, 3)
    B[0,0], B[0,1], B[0,2] = 1, 2, 3
    B[1,0], B[1,1], B[1,2] = 4, 5, 6
    print(B, "\n\n")

    C = chrono.ChMatrixDynamicd(5, 7)
    for i in range(2):
        for j in range(3):
            C[1 + i, 1 + j] = [10, 20, 30, 40, 50, 60][i * 3 + j]
    print(C, "\n\n")

    X = chrono.ChMatrixDynamicd(4, 6)
    for i in range(4):
        for j in range(6):
            X[i, j] = A[i, j]
    for i in range(2):
        for j in range(3):
            X[1 + i, 2 + j] = B[i, j]
    print(X, "\n\n")
    assert X[1, 2] == pytest.approx(B[0, 0], abs = ABS_ERR)
    assert X[1, 3] == pytest.approx(B[0, 1], abs = ABS_ERR)
    assert X[1, 4] == pytest.approx(B[0, 2], abs = ABS_ERR)

    for i in range(2):
        for j in range(3):
            X[1 + i, 2 + j] += C[1 + i, 1 + j]
    print(X, "\n\n")
    assert X[1, 2] == pytest.approx(B[0, 0] + C[1, 1], abs = ABS_ERR)
    assert X[1, 3] == pytest.approx(B[0, 1] + C[1, 2], abs = ABS_ERR)
    assert X[1, 4] == pytest.approx(B[0, 2] + C[1, 3], abs = ABS_ERR)
