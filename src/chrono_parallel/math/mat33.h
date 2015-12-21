// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: definition of a 3x3 matrix class
// =============================================================================

#pragma once

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real2.h"
#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/math/real4.h"
namespace chrono {

class Mat33 {
  public:
    // Zero constructor
    Mat33() {
        cols[0] = real3(0.0);
        cols[1] = real3(0.0);
        cols[2] = real3(0.0);
    }
    // diagonal matrix constructor
    Mat33(real v) : array{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0} {}

    // Constructor that takes three columns of the matrix
    Mat33(const real3& col1, const real3& col2, const real3& col3) {
        cols[0] = col1;
        cols[1] = col2;
        cols[2] = col3;
    }
    Mat33(const real& v11,
          const real& v21,
          const real& v31,
          const real& v12,
          const real& v22,
          const real& v32,
          const real& v13,
          const real& v23,
          const real& v33)
        : array{v11, v21, v31, 0, v12, v22, v32, 0, v13, v23, v33, 0} {}

    Mat33(const Mat33& M) {}

    // Mat33 from quaternion
    //    Mat33(const quaternion& quat) {
    //        cols[0] = Rotate(real3(1.0, 0.0, 0.0), quat);
    //        cols[1] = Rotate(real3(0.0, 1.0, 0.0), quat);
    //        cols[2] = Rotate(real3(0.0, 0.0, 1.0), quat);
    //    }
    Mat33(const quaternion& q) {
        cols[0] = real3(real(1.0) - real(2.0) * q.y * q.y - real(2.0) * q.z * q.z,
                        real(2.0) * q.x * q.y + real(2.0) * q.z * q.w, real(2.0) * q.x * q.z - real(2.0) * q.y * q.w);
        cols[1] = real3(real(2.0) * q.x * q.y - real(2.0) * q.z * q.w,
                        real(1.0) - real(2.0) * q.x * q.x - real(2.0) * q.z * q.z,
                        real(2.0) * q.y * q.z + real(2.0) * q.x * q.w);
        cols[2] = real3(real(2.0) * q.x * q.z + real(2.0) * q.y * q.w, real(2.0) * q.y * q.z - real(2.0) * q.x * q.w,
                        real(1.0) - real(2.0) * q.x * q.x - real(2.0) * q.y * q.y);
    }

    inline real operator[](unsigned int i) const { return array[i]; }
    inline real& operator[](unsigned int i) { return array[i]; }

    inline Mat33& operator=(const Mat33& M) {
        cols[0] = M.cols[0];
        cols[1] = M.cols[1];
        cols[2] = M.cols[2];
        return *this;
    }
    //
    //    real operator()(int i, int j) const { return static_cast<const real*>(cols[j])[i]; }
    //    real& operator()(int i, int j) { return static_cast<real*>(cols[j])[i]; }
    union {
        real array[12];
        real3 cols[3];
    };
    // cols[0] 0 1 2
    // cols[1] 3 4 5
    // cols[2] 6 7 8
};
// ========================================================================================

real3 operator*(const Mat33& M, const real3& v);

static inline Mat33 operator*(const Mat33& N, const real scale) {
    Mat33 M(N);
    M.cols[0] *= scale;
    M.cols[1] *= scale;
    M.cols[2] *= scale;
    return M;
}

Mat33 operator*(const Mat33& M, const Mat33& N);

static inline Mat33 operator+(const Mat33& M, const Mat33& N) {
    return Mat33(M.cols[0] + N.cols[0], M.cols[1] + N.cols[1], M.cols[2] + N.cols[2]);  //
}

static inline Mat33 operator-(const Mat33& M, const Mat33& N) {
    return Mat33(M.cols[0] - N.cols[0], M.cols[1] - N.cols[1], M.cols[2] - N.cols[2]);  //
}

OPERATOR_EQUALSALT(*, real, Mat33)

OPERATOR_EQUALSALT(*, Mat33, Mat33)
OPERATOR_EQUALSALT(+, Mat33, Mat33)
OPERATOR_EQUALSALT(-, Mat33, Mat33)

static inline Mat33 operator-(const Mat33& M) {
    return Mat33(-M.cols[0], -M.cols[1], -M.cols[2]);
}

static inline Mat33 operator*(const real s, const Mat33& a) {
    return a * s;
}

static inline Mat33 Identity() {
    return Mat33(1.0);
}

Mat33 SkewSymmetric(const real3& r);

static inline real Determinant(const Mat33& m) {
    return Dot(m.cols[0], Cross(m.cols[1], m.cols[2]));
}

Mat33 Abs(const Mat33& m);

// static inline Mat33 Transpose(const Mat33& a) {
//    Mat33 result;
//    for (unsigned int i = 0; i < 3; ++i)
//        for (unsigned int j = 0; j < 3; ++j)
//            result(i, j) = a(j, i);
//
//    return result;
//}
// M * N^T
Mat33 MultTranspose(const Mat33& M, const Mat33& N);
// M^T * N
Mat33 TransposeMult(const Mat33& M, const Mat33& N);

real Trace(const Mat33& m);
// Multiply a 3x1 by a 1x3 to get a 3x3
Mat33 OuterProduct(const real3& a, const real3& b);

static inline real InnerProduct(const Mat33& A, const Mat33& B) {
    return Dot(A.cols[0], B.cols[0]) + Dot(A.cols[1], B.cols[1]) + Dot(A.cols[0], B.cols[0]);
}

//
// static inline Mat33 Inverse(const Mat33& A) {
//    real s = Determinant(A);
//
//    if (s > 0.0f) {
//        Mat33 B;
//
//        B(0, 0) = A(1, 1) * A(2, 2) - A(1, 2) * A(2, 1);
//        B(0, 1) = A(0, 2) * A(2, 1) - A(0, 1) * A(2, 2);
//        B(0, 2) = A(0, 1) * A(1, 2) - A(0, 2) * A(1, 1);
//        B(1, 0) = A(1, 2) * A(2, 0) - A(1, 0) * A(2, 2);
//        B(1, 1) = A(0, 0) * A(2, 2) - A(0, 2) * A(2, 0);
//        B(1, 2) = A(0, 2) * A(1, 0) - A(0, 0) * A(1, 2);
//        B(2, 0) = A(1, 0) * A(2, 1) - A(1, 1) * A(2, 0);
//        B(2, 1) = A(0, 1) * A(2, 0) - A(0, 0) * A(2, 1);
//        B(2, 2) = A(0, 0) * A(1, 1) - A(0, 1) * A(1, 0);
//
//        // pass = true;
//
//        return (1.0f / s) * B;
//    } else {
//        // pass = false;
//        return Mat33();
//    }
//}
// Same as inverse but we store it transposed
// static inline Mat33 InverseTranspose(const Mat33& a) {
//    real s = Determinant(a);
//
//    if (s > 0.0f) {
//        Mat33 b;
//
//        b(0, 0) = a(1, 1) * a(2, 2) - a(1, 2) * a(2, 1);
//        b(1, 0) = a(0, 2) * a(2, 1) - a(0, 1) * a(2, 2);
//        b(2, 0) = a(0, 1) * a(1, 2) - a(0, 2) * a(1, 1);
//
//        b(0, 1) = a(1, 2) * a(2, 0) - a(1, 0) * a(2, 2);
//        b(1, 1) = a(0, 0) * a(2, 2) - a(0, 2) * a(2, 0);
//        b(2, 1) = a(0, 2) * a(1, 0) - a(0, 0) * a(1, 2);
//
//        b(0, 2) = a(1, 0) * a(2, 1) - a(1, 1) * a(2, 0);
//        b(1, 2) = a(0, 1) * a(2, 0) - a(0, 0) * a(2, 1);
//        b(2, 2) = a(0, 0) * a(1, 1) - a(0, 1) * a(1, 0);
//
//        // pass = true;
//
//        return (1.0f / s) * b;
//    } else {
//        // pass = false;
//        return Mat33();
//    }
//}
// double dot product of a matrix
// static real Norm(const Mat33& A) {
//    return Sqrt(Trace(A * Transpose(A)));
//}
//
// static Mat33 Adjoint(const Mat33& A) {
//    Mat33 T;
//    T.cols[0].x = A.cols[1].y * A.cols[2].z - A.cols[2].y * A.cols[1].z;
//    T.cols[0].y = -A.cols[0].y * A.cols[2].z + A.cols[2].y * A.cols[0].z;
//    T.cols[0].z = A.cols[0].y * A.cols[1].z - A.cols[1].y * A.cols[0].z;
//
//    T.cols[1].x = -A.cols[1].x * A.cols[2].z + A.cols[2].x * A.cols[1].z;
//    T.cols[1].y = A.cols[0].x * A.cols[2].z - A.cols[2].x * A.cols[0].z;
//    T.cols[1].z = -A.cols[0].x * A.cols[1].z + A.cols[1].x * A.cols[0].z;
//
//    T.cols[2].x = A.cols[1].x * A.cols[2].y - A.cols[2].x * A.cols[1].y;
//    T.cols[2].y = -A.cols[0].x * A.cols[2].y + A.cols[2].x * A.cols[0].y;
//    T.cols[2].z = A.cols[0].x * A.cols[1].y - A.cols[1].x * A.cols[0].y;
//    return T;
//}
//
// static real3 LargestColumnNormalized(const Mat33& A) {
//    real scale1 = Length2((A.cols[0]));
//    real scale2 = Length2((A.cols[1]));
//    real scale3 = Length2((A.cols[2]));
//    if (scale1 > scale2) {
//        if (scale1 > scale3) {
//            return A.cols[0] / sqrt(scale1);
//        }
//    } else if (scale2 > scale3) {
//        return A.cols[1] / sqrt(scale2);
//    }
//    return A.cols[2] / sqrt(scale3);
//}

// ========================================================================================

static void Print(Mat33 A, const char* name) {
    //    printf("%s\n", name);
    //    printf("%f %f %f\n", A.cols[0].x, A.cols[1].x, A.cols[2].x);
    //    printf("%f %f %f\n", A.cols[0].y, A.cols[1].y, A.cols[2].y);
    //    printf("%f %f %f\n", A.cols[0].z, A.cols[1].z, A.cols[2].z);
}

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
}
