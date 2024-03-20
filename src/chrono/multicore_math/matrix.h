// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Description: definition of a 3x3 matrix class
// =============================================================================

#pragma once

#include "chrono/core/ChApiCE.h"

#include "chrono/multicore_math/ChCudaDefines.h"
#include "chrono/multicore_math/real2.h"
#include "chrono/multicore_math/real3.h"
#include "chrono/multicore_math/real4.h"

namespace chrono {

/// @addtogroup chrono_mc_math
/// @{

class ChApi Mat33 {
  public:
    // Zero constructor
    CUDA_HOST_DEVICE Mat33() {
        array[0] = 0;
        array[1] = 0;
        array[2] = 0;
        array[3] = 0;  // not used
        array[4] = 0;
        array[5] = 0;
        array[6] = 0;
        array[7] = 0;  // not used
        array[8] = 0;
        array[9] = 0;
        array[10] = 0;
        array[11] = 0;  // not used
    }
    // Diagonal matrix constructor
    CUDA_HOST_DEVICE Mat33(real v) {
        array[0] = v;
        array[1] = 0;
        array[2] = 0;
        array[3] = 0;
        array[4] = 0;
        array[5] = v;
        array[6] = 0;
        array[7] = 0;
        array[8] = 0;
        array[9] = 0;
        array[10] = v;
        array[11] = 0;
    }
    // Diagonal matrix constructor
    CUDA_HOST_DEVICE Mat33(real3 v) {
        array[0] = v.x;
        array[1] = 0;
        array[2] = 0;
        array[3] = 0;
        array[4] = 0;
        array[5] = v.y;
        array[6] = 0;
        array[7] = 0;
        array[8] = 0;
        array[9] = 0;
        array[10] = v.z;
        array[11] = 0;
    }

    // Constructor that takes three columns of the matrix
    CUDA_HOST_DEVICE Mat33(const real3& col1, const real3& col2, const real3& col3) {
        array[0] = col1.x;
        array[1] = col1.y;
        array[2] = col1.z;
        array[3] = 0;
        array[4] = col2.x;
        array[5] = col2.y;
        array[6] = col2.z;
        array[7] = 0;
        array[8] = col3.x;
        array[9] = col3.y;
        array[10] = col3.z;
        array[11] = 0;
    }

    // Constructor that takes individial elements
    CUDA_HOST_DEVICE Mat33(const real& v11,
                           const real& v21,
                           const real& v31,
                           const real& v12,
                           const real& v22,
                           const real& v32,
                           const real& v13,
                           const real& v23,
                           const real& v33) {
        array[0] = v11;
        array[1] = v21;
        array[2] = v31;
        array[3] = 0;
        array[4] = v12;
        array[5] = v22;
        array[6] = v32;
        array[7] = 0;
        array[8] = v13;
        array[9] = v23;
        array[10] = v33;
        array[11] = 0;
    }
    // Copy constructor
    CUDA_HOST_DEVICE Mat33(const Mat33& M) { memcpy(array, M.array, 12 * sizeof(real)); }

    // Constructor that takes a quaternion and generates a rotation matrix
    CUDA_HOST_DEVICE Mat33(const quaternion& q) {
        array[0] = real(1.0) - real(2.0) * q.y * q.y - real(2.0) * q.z * q.z;
        array[1] = real(2.0) * q.x * q.y + real(2.0) * q.z * q.w;
        array[2] = real(2.0) * q.x * q.z - real(2.0) * q.y * q.w;

        array[4] = real(2.0) * q.x * q.y - real(2.0) * q.z * q.w;
        array[5] = real(1.0) - real(2.0) * q.x * q.x - real(2.0) * q.z * q.z;
        array[6] = real(2.0) * q.y * q.z + real(2.0) * q.x * q.w;

        array[8] = real(2.0) * q.x * q.z + real(2.0) * q.y * q.w;
        array[9] = real(2.0) * q.y * q.z - real(2.0) * q.x * q.w;
        array[10] = real(1.0) - real(2.0) * q.x * q.x - real(2.0) * q.y * q.y;
    }

    CUDA_HOST_DEVICE inline real operator[](unsigned int i) const { return array[i]; }
    CUDA_HOST_DEVICE inline real& operator[](unsigned int i) { return array[i]; }
    CUDA_HOST_DEVICE inline real operator()(int i, int j) const { return array[j * 4 + i]; }
    CUDA_HOST_DEVICE inline real& operator()(int i, int j) { return array[j * 4 + i]; }
    CUDA_HOST_DEVICE inline real3 col(unsigned int i) const {
        return real3(array[i * 4], array[i * 4 + 1], array[i * 4 + 2]);
    }
    CUDA_HOST_DEVICE inline real3 row(unsigned int i) const {
        return real3(array[0 * 4 + i], array[1 * 4 + i], array[2 * 4 + i]);
    }

    CUDA_HOST_DEVICE inline Mat33& operator=(const Mat33& M) {
        memcpy(array, M.array, 12 * sizeof(real));
        return *this;
    }

    real array[12];

    // c1 c2 c3
    // 0  4  8
    // 1  5  9
    // 2  6  10

    // cols[0] 0 1 2
    // cols[1] 3 4 5
    // cols[2] 6 7 8
};

// ========================================================================================

CUDA_HOST_DEVICE ChApi real3 operator*(const Mat33& M, const real3& v);
CUDA_HOST_DEVICE ChApi Mat33 operator*(const Mat33& N, const real scale);
CUDA_HOST_DEVICE ChApi Mat33 operator*(const Mat33& M, const Mat33& N);
CUDA_HOST_DEVICE ChApi Mat33 operator+(const Mat33& M, const Mat33& N);
CUDA_HOST_DEVICE ChApi Mat33 operator-(const Mat33& M, const Mat33& N);

CUDA_HOST_DEVICE ChApi OPERATOR_EQUALSALT_PROTO(*, real, Mat33)       //
    CUDA_HOST_DEVICE ChApi OPERATOR_EQUALSALT_PROTO(*, Mat33, Mat33)  //
    CUDA_HOST_DEVICE ChApi OPERATOR_EQUALSALT_PROTO(+, Mat33, Mat33)  //
    CUDA_HOST_DEVICE ChApi OPERATOR_EQUALSALT_PROTO(-, Mat33, Mat33)  //

    CUDA_HOST_DEVICE ChApi Mat33
    operator-(const Mat33& M);
CUDA_HOST_DEVICE ChApi Mat33 operator*(const real s, const Mat33& a);

CUDA_HOST_DEVICE ChApi Mat33 SkewSymmetric(const real3& r);
// This form generates a skew symmetric structure without flipping signs
CUDA_HOST_DEVICE ChApi Mat33 SkewSymmetricAlt(const real3& r);
CUDA_HOST_DEVICE ChApi real Determinant(const Mat33& m);
CUDA_HOST_DEVICE ChApi Mat33 Abs(const Mat33& m);
CUDA_HOST_DEVICE ChApi Mat33 Transpose(const Mat33& a);
CUDA_HOST_DEVICE ChApi Mat33 MultTranspose(const Mat33& M, const Mat33& N);  // M * N^T
CUDA_HOST_DEVICE ChApi Mat33 TransposeMult(const Mat33& M, const Mat33& N);  // M^T * N
CUDA_HOST_DEVICE ChApi real Trace(const Mat33& m);
CUDA_HOST_DEVICE ChApi Mat33 OuterProduct(const real3& a,
                                          const real3& b);  // Multiply a 3x1 by a 1x3 to get a 3x3
CUDA_HOST_DEVICE ChApi real InnerProduct(const Mat33& A, const Mat33& B);
CUDA_HOST_DEVICE ChApi Mat33 Adjoint(const Mat33& A);
CUDA_HOST_DEVICE ChApi Mat33 AdjointTranspose(const Mat33& A);
CUDA_HOST_DEVICE ChApi Mat33 Inverse(const Mat33& A);
CUDA_HOST_DEVICE ChApi Mat33 InverseTranspose(const Mat33& A);
CUDA_HOST_DEVICE ChApi real Norm(const Mat33& A);    // normalized double dot product of a matrix
CUDA_HOST_DEVICE ChApi real NormSq(const Mat33& A);  // normalized double dot product of a matrix
CUDA_HOST_DEVICE ChApi real DoubleDot(const Mat33& A, const Mat33& B);
CUDA_HOST_DEVICE ChApi real3 LargestColumnNormalized(const Mat33& A);

// ========================================================================================

struct ChApi DiagMat33 {
    CUDA_HOST_DEVICE DiagMat33() {}
    CUDA_HOST_DEVICE DiagMat33(const DiagMat33& M) : x11(M.x11), x22(M.x22), x33(M.x33) {}
    CUDA_HOST_DEVICE DiagMat33(const real v11, const real v22, const real v33) : x11(v11), x22(v22), x33(v33) {}
    CUDA_HOST_DEVICE DiagMat33(const real3 v) : x11(v.x), x22(v.y), x33(v.z) {}

    real x11, x22, x33;
};

CUDA_HOST_DEVICE ChApi Mat33 operator*(const DiagMat33& M, const Mat33& N);
CUDA_HOST_DEVICE ChApi real3 operator*(const DiagMat33& M, const real3& v);

// ========================================================================================

struct ChApi SymMat33 {
    CUDA_HOST_DEVICE SymMat33() : x11(0), x21(0), x31(0), x22(0), x32(0), x33(0) {}

    CUDA_HOST_DEVICE SymMat33(const real y11,
                              const real y21,
                              const real y31,
                              const real y22,
                              const real y32,
                              const real y33)
        : x11(y11), x21(y21), x31(y31), x22(y22), x32(y32), x33(y33) {}
    CUDA_HOST_DEVICE inline real operator[](unsigned int i) const { return array[i]; }
    CUDA_HOST_DEVICE inline real& operator[](unsigned int i) { return array[i]; }
    CUDA_HOST_DEVICE void operator=(const SymMat33& N) {
        x11 = N.x11;
        x21 = N.x21;
        x31 = N.x31;
        x22 = N.x22;
        x32 = N.x32;
        x33 = N.x33;
    }
    union {
        real array[8];
        struct {
            real x11, x21, x31, x22, x32, x33;
        };
    };
};

CUDA_HOST_DEVICE ChApi SymMat33 operator-(const SymMat33& M, const real& v);

// ========================================================================================

struct ChApi Mat32 {
    CUDA_HOST_DEVICE Mat32() {}
    CUDA_HOST_DEVICE Mat32(const real3 col1, const real3 col2) {
        array[0] = col1.x;
        array[1] = col1.y;
        array[2] = col1.z;
        array[3] = 0;
        array[4] = col2.x;
        array[5] = col2.y;
        array[6] = col2.z;
        array[7] = 0;
    }
    CUDA_HOST_DEVICE inline real operator[](unsigned int i) const { return array[i]; }
    CUDA_HOST_DEVICE inline real& operator[](unsigned int i) { return array[i]; }
    real array[8];
    // 0 4
    // 1 5
    // 2 6
    // 3 7 //Not used
};

CUDA_HOST_DEVICE ChApi real3 operator*(const Mat32& M, const real2& v);

// ========================================================================================

struct ChApi Mat23 {
    CUDA_HOST_DEVICE Mat23() {}
    CUDA_HOST_DEVICE Mat23(const real3 row1, const real3 row2) {
        array[0] = row1.x;
        array[1] = row1.y;
        array[2] = row1.z;
        array[3] = 0;
        array[4] = row2.x;
        array[5] = row2.y;
        array[6] = row2.z;
        array[7] = 0;
    }
    real array[8];
    // 0 1 2 3
    // 4 5 6 7
};

// ========================================================================================

struct ChApi SymMat22 {
    CUDA_HOST_DEVICE SymMat22() : x11(0), x21(0), x22(0) {}
    CUDA_HOST_DEVICE SymMat22(const real v11, const real v21, const real v22) : x11(v11), x21(v21), x22(v22) {}

    real x11, x21, x22;
};

CUDA_HOST_DEVICE ChApi SymMat22 operator-(const SymMat22& M, const real& v);
CUDA_HOST_DEVICE ChApi SymMat22 CofactorMatrix(const SymMat22& A);
CUDA_HOST_DEVICE ChApi real2 LargestColumnNormalized(const SymMat22& A);

// ========================================================================================

// Compute the normal equations matrix - 18 mults, 12 adds
CUDA_HOST_DEVICE ChApi SymMat33 NormalEquationsMatrix(const Mat33& A);
CUDA_HOST_DEVICE ChApi SymMat33 CofactorMatrix(const SymMat33& A);
CUDA_HOST_DEVICE ChApi real3 LargestColumnNormalized(const SymMat33& A);

// ========================================================================================

CUDA_HOST_DEVICE ChApi Mat32 operator*(const SymMat33& M, const Mat32& N);

// A^T*B
CUDA_HOST_DEVICE ChApi SymMat22 TransposeTimesWithSymmetricResult(const Mat32& A, const Mat32& B);
//
CUDA_HOST_DEVICE ChApi SymMat22 ConjugateWithTranspose(const Mat32& A, const SymMat33& B);

CUDA_HOST_DEVICE ChApi void Print(const Mat33& A, const char* name);
CUDA_HOST_DEVICE ChApi void Print(const Mat32& A, const char* name);
CUDA_HOST_DEVICE ChApi void Print(const SymMat33& A, const char* name);
CUDA_HOST_DEVICE ChApi void Print(const SymMat22& A, const char* name);

CUDA_HOST_DEVICE ChApi void PrintLine(const Mat33& A, const char* name);
CUDA_HOST_DEVICE ChApi void PrintLine(const Mat32& A, const char* name);
CUDA_HOST_DEVICE ChApi void PrintLine(const SymMat33& A, const char* name);
CUDA_HOST_DEVICE ChApi void PrintLine(const SymMat22& A, const char* name);

//[U.x,V.x,W.x]
//[U.y,V.y,W.y]
//[U.z,V.z,W.z]
// transposed:
//[U.x,U.y,U.z]
//[V.x,V.y,V.z]
//[W.x,W.y,W.z]

//[U.x,V.x,W.x][x]
//[U.y,V.y,W.y][y]
//[U.z,V.z,W.z][z]

/// @} chrono_mc_math

}  // end namespace chrono
