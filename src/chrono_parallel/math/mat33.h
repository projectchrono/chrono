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

struct Mat33 {
    // Zero constructor
    Mat33() {
        cols[0] = real3(0.0f);
        cols[1] = real3(0.0f);
        cols[2] = real3(0.0f);
    }
    // diagonal matrix constructor
    Mat33(real v) {
        cols[0] = real3(v, 0, 0);
        cols[1] = real3(0, v, 0);
        cols[2] = real3(0, 0, v);
    }

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
          const real& v33) {
        cols[0] = real3(v11, v21, v31);
        cols[1] = real3(v12, v22, v32);
        cols[2] = real3(v13, v23, v33);
    }
    // Mat33 from quaternion
    Mat33(const quaternion& quat) {
        cols[0] = Rotate(real3(1.0f, 0.0f, 0.0f), quat);
        cols[1] = Rotate(real3(0.0f, 1.0f, 0.0f), quat);
        cols[2] = Rotate(real3(0.0f, 0.0f, 1.0f), quat);
    }

    real operator()(int i, int j) const { return static_cast<const real*>(cols[j])[i]; }
    real& operator()(int i, int j) { return static_cast<real*>(cols[j])[i]; }

    Mat33 operator*(const real scale) const {
        Mat33 M(*this);
        M.cols[0] *= scale;
        M.cols[1] *= scale;
        M.cols[2] *= scale;
        return M;
    }

    real3 operator*(const real3& v) const {
        return cols[0] * v.x + cols[1] * v.y + cols[2] * v.z;  //
    }

    Mat33 operator*(const Mat33& N) const {
        Mat33 M(*this);
        return Mat33(M * N.cols[0], M * N.cols[1], M * N.cols[2]);
    }

    Mat33 operator+(const Mat33& N) const {
        return Mat33(cols[0] + N.cols[0], cols[1] + N.cols[1], cols[2] + N.cols[2]);  //
    }

    Mat33 operator-(const Mat33& N) const {
        return Mat33(cols[0] - N.cols[0], cols[1] - N.cols[1], cols[2] - N.cols[2]);  //
    }

    Mat33& operator*=(real s) {
        cols[0] *= s;
        cols[1] *= s;
        cols[2] *= s;
        return *this;
    }

    Mat33& operator*=(const Mat33& N) {
        *this = *this * N;
        return *this;
    }

    Mat33& operator+=(const Mat33& N) {
        *this = *this + N;
        return *this;
    }

    Mat33& operator-=(const Mat33& N) {
        *this = *this - N;
        return *this;
    }

    inline Mat33 operator-() const { return Mat33(-cols[0], -cols[1], -cols[2]); }
    real3 cols[3];
    // cols[0] 0 1 2
    // cols[1] 3 4 5
    // cols[2] 6 7 8
};
// ========================================================================================

struct DiagMat33 {
    DiagMat33() {}
    DiagMat33(const DiagMat33& M) : x11(M.x11), x22(M.x22), x33(M.x33) {}
    DiagMat33(const real v11, const real v22, const real v33) : x11(v11), x22(v22), x33(v33) {}
    DiagMat33(const real3 v) : x11(v.x), x22(v.y), x33(v.z) {}

    Mat33 operator*(const Mat33& N) const {
        return Mat33(real3(x11 * N.cols[0].x, x22 * N.cols[0].y, x33 * N.cols[0].z),
                     real3(x11 * N.cols[1].x, x22 * N.cols[1].y, x33 * N.cols[1].z),
                     real3(x11 * N.cols[2].x, x22 * N.cols[2].y, x33 * N.cols[2].z));
    }

    real3 operator*(const real3& v) const {
        real3 result;
        result.x = x11 * v.x;
        result.y = x22 * v.y;
        result.z = x33 * v.z;
        return result;
    }

    real x11, x22, x33;
};
//// ========================================================================================
//

struct SymMat33 {
    SymMat33() : x11(0), x21(0), x31(0), x22(0), x32(0), x33(0) {}

    SymMat33(const real y11, const real y21, const real y31, const real y22, const real y32, const real y33)
        : x11(y11), x21(y21), x31(y31), x22(y22), x32(y32), x33(y33) {}

    void operator=(const SymMat33& N) {
        x11 = N.x11;
        x21 = N.x21;
        x31 = N.x31;
        x22 = N.x22;
        x32 = N.x32;
        x33 = N.x33;
    }
    SymMat33 operator-(const real& v) const {
        return SymMat33(x11 - v, x21, x31, x22 - v, x32, x33 - v);  // only subtract diagonal
    }

    real x11, x21, x31, x22, x32, x33;
};
//// ========================================================================================

struct Mat32 {
    Mat32() {}
    Mat32(const real3 col1, const real3 col2) {
        cols[0] = col1;
        cols[1] = col2;
    }

    inline real3 operator*(const real2& v) {
        real3 result;
        result.x = cols[0].x * v.x + cols[1].x * v.y;
        result.y = cols[0].y * v.x + cols[1].y * v.y;
        result.z = cols[0].z * v.x + cols[1].z * v.y;

        return result;
    }
    real3 cols[2];
};
// ========================================================================================

struct Mat23 {
    Mat23() {}
    Mat23(const real3 row1, const real3 row2) {
        rows[0] = row1;
        rows[1] = row2;
    }

    real3 rows[2];
};
// ========================================================================================

struct SymMat22 {
    SymMat22() : x11(0), x21(0), x22(0) {}
    SymMat22(const real v11, const real v21, const real v22) : x11(v11), x21(v21), x22(v22) {}

    SymMat22 operator-(const real& v) const {
        return SymMat22(x11 - v, x21, x22 - v);  //
    }

    real x11, x21, x22;
};
//// ========================================================================================
//

static inline Mat33 operator*(const real s, const Mat33& a) {
    return a * s;
}

static inline Mat33 Identity() {
    return Mat33(1.0);
}

static inline Mat33 SkewSymmetric(const real3& r) {
    return Mat33(real3(0, -r.z, r.y), real3(r.z, 0, -r.x), real3(-r.y, r.x, 0));
}

static inline real Determinant(const Mat33& m) {
    return Dot(m.cols[0], Cross(m.cols[1], m.cols[2]));
}

static inline Mat33 Abs(const Mat33& m) {
    return Mat33(Abs(m.cols[0].x), Abs(m.cols[0].y), Abs(m.cols[0].z), Abs(m.cols[1].x), Abs(m.cols[1].y),
                 Abs(m.cols[1].z), Abs(m.cols[2].x), Abs(m.cols[2].y), Abs(m.cols[2].z));
}

static inline Mat33 Transpose(const Mat33& a) {
    Mat33 result;
    for (unsigned int i = 0; i < 3; ++i)
        for (unsigned int j = 0; j < 3; ++j)
            result(i, j) = a(j, i);

    return result;
}
// M * N^T
static Mat33 MultTranspose(const Mat33& M, const Mat33& N) {
    return Mat33(M * real3(N.cols[0].x, N.cols[1].x, N.cols[2].x),  //
                 M * real3(N.cols[0].y, N.cols[1].y, N.cols[2].y),  //
                 M * real3(N.cols[0].z, N.cols[1].z, N.cols[2].z));
}

// M^T * N
static Mat33 TransposeMult(const Mat33& M, const Mat33& N) {
	Mat33 result;

	result.cols[0].x = M.cols[0].x * N.cols[0].x + M.cols[0].y * N.cols[0].y + M.cols[0].z * N.cols[0].z;  // row1 * col1
	result.cols[1].x = M.cols[0].x * N.cols[1].x + M.cols[0].y * N.cols[1].y + M.cols[0].z * N.cols[1].z;  // row1 * col2
	result.cols[2].x = M.cols[0].x * N.cols[2].x + M.cols[0].y * N.cols[2].y + M.cols[0].z * N.cols[2].z;  // row1 * col3

	result.cols[0].y = M.cols[1].x * N.cols[0].x + M.cols[1].y * N.cols[0].y + M.cols[1].z * N.cols[0].z;  // row2 * col1
	result.cols[1].y = M.cols[1].x * N.cols[1].x + M.cols[1].y * N.cols[1].y + M.cols[1].z * N.cols[1].z;  // row2 * col2
	result.cols[2].y = M.cols[1].x * N.cols[2].x + M.cols[1].y * N.cols[2].y + M.cols[1].z * N.cols[2].z;  // row2 * col3

	result.cols[0].z = M.cols[2].x * N.cols[0].x + M.cols[2].y * N.cols[0].y + M.cols[2].z * N.cols[0].z;  // row3 * col1
	result.cols[1].z = M.cols[2].x * N.cols[1].x + M.cols[2].y * N.cols[1].y + M.cols[2].z * N.cols[1].z;  // row3 * col2
	result.cols[2].z = M.cols[2].x * N.cols[2].x + M.cols[2].y * N.cols[2].y + M.cols[2].z * N.cols[2].z;  // row3 * col3

    return result;
}

static Mat33 MultTranspose(const DiagMat33& M, const Mat33& N) {
    return Mat33(M * real3(N.cols[0].x, N.cols[1].x, N.cols[2].x),  //
                 M * real3(N.cols[0].y, N.cols[1].y, N.cols[2].y),  //
                 M * real3(N.cols[0].z, N.cols[1].z, N.cols[2].z));
}

static inline real Trace(const Mat33& m) {
    return m(0, 0) + m(1, 1) + m(2, 2);
}
// Multiply a 3x1 by a 1x3 to get a 3x3
static inline Mat33 OuterProduct(const real3& a, const real3& b) {
    return Mat33(a * b.x, a * b.y, a * b.z);
}

static inline real InnerProduct(const Mat33& A, const Mat33& B) {
    return Dot(A.cols[0], B.cols[0]) + Dot(A.cols[1], B.cols[1]) + Dot(A.cols[0], B.cols[0]);
}

static inline Mat33 SkewMatrix(const real3& v) {
    return Mat33(real3(0.0, v.z, -v.y), real3(-v.z, 0.0, v.x), real3(v.y, -v.x, 0.0));
}

static inline Mat33 Inverse(const Mat33& A) {
    real s = Determinant(A);

    if (s > 0.0f) {
        Mat33 B;

        B(0, 0) = A(1, 1) * A(2, 2) - A(1, 2) * A(2, 1);
        B(0, 1) = A(0, 2) * A(2, 1) - A(0, 1) * A(2, 2);
        B(0, 2) = A(0, 1) * A(1, 2) - A(0, 2) * A(1, 1);
        B(1, 0) = A(1, 2) * A(2, 0) - A(1, 0) * A(2, 2);
        B(1, 1) = A(0, 0) * A(2, 2) - A(0, 2) * A(2, 0);
        B(1, 2) = A(0, 2) * A(1, 0) - A(0, 0) * A(1, 2);
        B(2, 0) = A(1, 0) * A(2, 1) - A(1, 1) * A(2, 0);
        B(2, 1) = A(0, 1) * A(2, 0) - A(0, 0) * A(2, 1);
        B(2, 2) = A(0, 0) * A(1, 1) - A(0, 1) * A(1, 0);

        // pass = true;

        return (1.0f / s) * B;
    } else {
        // pass = false;
        return Mat33();
    }
}
// Same as inverse but we store it transposed
static inline Mat33 InverseTranspose(const Mat33& a) {
    real s = Determinant(a);

    if (s > 0.0f) {
        Mat33 b;

        b(0, 0) = a(1, 1) * a(2, 2) - a(1, 2) * a(2, 1);
        b(1, 0) = a(0, 2) * a(2, 1) - a(0, 1) * a(2, 2);
        b(2, 0) = a(0, 1) * a(1, 2) - a(0, 2) * a(1, 1);

        b(0, 1) = a(1, 2) * a(2, 0) - a(1, 0) * a(2, 2);
        b(1, 1) = a(0, 0) * a(2, 2) - a(0, 2) * a(2, 0);
        b(2, 1) = a(0, 2) * a(1, 0) - a(0, 0) * a(1, 2);

        b(0, 2) = a(1, 0) * a(2, 1) - a(1, 1) * a(2, 0);
        b(1, 2) = a(0, 1) * a(2, 0) - a(0, 0) * a(2, 1);
        b(2, 2) = a(0, 0) * a(1, 1) - a(0, 1) * a(1, 0);

        // pass = true;

        return (1.0f / s) * b;
    } else {
        // pass = false;
        return Mat33();
    }
}
// double dot product of a matrix
static real Norm(const Mat33& A) {
    return Sqrt(Trace(A * Transpose(A)));
}

// Compute the normal equations matrix - 18 mults, 12 adds
static SymMat33 NormalEquationsMatrix(const Mat33& A) {
    SymMat33 T;
    T.x11 = Dot(A.cols[0], A.cols[0]);
    T.x21 = Dot(A.cols[0], A.cols[1]);
    T.x31 = Dot(A.cols[0], A.cols[2]);
    T.x22 = Dot(A.cols[1], A.cols[1]);
    T.x32 = Dot(A.cols[1], A.cols[2]);
    T.x33 = Dot(A.cols[2], A.cols[2]);
    return T;
}
// static  Mat33 Cofactor_Matrix(const Mat33& A) {
//    Mat33 T;
//    T.cols[0].x = A.cols[1].y * A.cols[2].z - A.cols[2].y * A.cols[1].z;
//    T.cols[0].y = -A.cols[1].x * A.cols[2].z + A.cols[2].x * A.cols[1].z;
//    T.cols[0].z = A.cols[1].x * A.cols[2].y - A.cols[2].x * A.cols[1].y;
//
//    T.cols[1].x = -A.cols[0].y * A.cols[2].z + A.cols[2].y * A.cols[0].z;
//    T.cols[1].y = A.cols[0].x * A.cols[2].z - A.cols[2].x * A.cols[0].z;
//    T.cols[1].z = -A.cols[0].x * A.cols[2].y + A.cols[2].x * A.cols[0].y;
//
//    T.cols[2].x = A.cols[0].y * A.cols[1].z - A.cols[1].y * A.cols[0].z;
//    T.cols[2].y = -A.cols[0].x * A.cols[1].z + A.cols[1].x * A.cols[0].z;
//    T.cols[2].z = A.cols[0].x * A.cols[1].y - A.cols[1].x * A.cols[0].y;
//    return T;
//}
//
static Mat33 Adjoint(const Mat33& A) {
    Mat33 T;
    T.cols[0].x = A.cols[1].y * A.cols[2].z - A.cols[2].y * A.cols[1].z;
    T.cols[0].y = -A.cols[0].y * A.cols[2].z + A.cols[2].y * A.cols[0].z;
    T.cols[0].z = A.cols[0].y * A.cols[1].z - A.cols[1].y * A.cols[0].z;

    T.cols[1].x = -A.cols[1].x * A.cols[2].z + A.cols[2].x * A.cols[1].z;
    T.cols[1].y = A.cols[0].x * A.cols[2].z - A.cols[2].x * A.cols[0].z;
    T.cols[1].z = -A.cols[0].x * A.cols[1].z + A.cols[1].x * A.cols[0].z;

    T.cols[2].x = A.cols[1].x * A.cols[2].y - A.cols[2].x * A.cols[1].y;
    T.cols[2].y = -A.cols[0].x * A.cols[2].y + A.cols[2].x * A.cols[0].y;
    T.cols[2].z = A.cols[0].x * A.cols[1].y - A.cols[1].x * A.cols[0].y;
    return T;
}

static real3 LargestColumnNormalized(const Mat33& A) {
    real scale1 = Length2((A.cols[0]));
    real scale2 = Length2((A.cols[1]));
    real scale3 = Length2((A.cols[2]));
    if (scale1 > scale2) {
        if (scale1 > scale3) {
            return A.cols[0] / sqrt(scale1);
        }
    } else if (scale2 > scale3) {
        return A.cols[1] / sqrt(scale2);
    }
    return A.cols[2] / sqrt(scale3);
}

// ========================================================================================

static SymMat33 CofactorMatrix(const SymMat33& A) {
    SymMat33 T;
    T.x11 = A.x22 * A.x33 - A.x32 * A.x32;   //
    T.x21 = -A.x21 * A.x33 + A.x32 * A.x31;  //
    T.x22 = A.x11 * A.x33 - A.x31 * A.x31;   //
    T.x31 = A.x21 * A.x32 - A.x22 * A.x31;   //
    T.x32 = -A.x11 * A.x32 + A.x21 * A.x31;  //
    T.x33 = A.x11 * A.x22 - A.x21 * A.x21;   //
    return T;
}

static SymMat22 CofactorMatrix(const SymMat22& A) {
    SymMat22 T;
    T.x11 = A.x22;   //
    T.x21 = -A.x21;  //
    T.x22 = A.x11;   //
    return T;
}

static real3 LargestColumnNormalized(const SymMat33& A) {
    real scale1 = Length2(real3(A.x11, A.x21, A.x31));
    real scale2 = Length2(real3(A.x21, A.x22, A.x32));
    real scale3 = Length2(real3(A.x31, A.x32, A.x33));
    if (scale1 > scale2) {
        if (scale1 > scale3) {
            return real3(A.x11, A.x21, A.x31) / sqrt(scale1);
        }
    } else if (scale2 > scale3) {
        return real3(A.x21, A.x22, A.x32) / sqrt(scale2);
    }
    if (scale3 > 0)
        return real3(A.x31, A.x32, A.x33) / sqrt(scale3);
    else {
        return (real3(1, 0, 0));
    }
}

static real2 LargestColumnNormalized(const SymMat22& A) {
    real scale1 = Length2(real2(A.x11, A.x21));
    real scale2 = Length2(real2(A.x21, A.x22));
    if (scale1 > scale2) {
        return real2(A.x11, A.x21) / sqrt(scale1);
    } else if (scale2 > 0) {
        return real2(A.x21, A.x22) / sqrt(scale2);
    } else {
        return real2(1, 0);
    }
}

static inline Mat32 operator*(const SymMat33& M, const Mat32& N) {
    Mat32 result;

    // x11 x21 x31  c11 c12
    // x21 x22 x32  c21 c22
    // x31 x32 x33  c31 c32
    result.cols[0].x = M.x11 * N.cols[0].x + M.x21 * N.cols[0].y + M.x31 * N.cols[0].z;
    result.cols[0].y = M.x21 * N.cols[0].x + M.x22 * N.cols[0].y + M.x32 * N.cols[0].z;
    result.cols[0].z = M.x31 * N.cols[0].x + M.x32 * N.cols[0].y + M.x33 * N.cols[0].z;

    result.cols[1].x = M.x11 * N.cols[1].x + M.x21 * N.cols[1].y + M.x31 * N.cols[1].z;
    result.cols[1].y = M.x21 * N.cols[1].x + M.x22 * N.cols[1].y + M.x32 * N.cols[1].z;
    result.cols[1].z = M.x31 * N.cols[1].x + M.x32 * N.cols[1].y + M.x33 * N.cols[1].z;

    return result;
}
// A^T*B
static inline SymMat22 TransposeTimesWithSymmetricResult(const Mat32& A, const Mat32& B) {
    SymMat22 T;
    T.x11 = Dot(A.cols[0], B.cols[0]);
    T.x21 = Dot(A.cols[1], B.cols[0]);
    T.x22 = Dot(A.cols[1], B.cols[1]);
    return T;
}

static inline SymMat22 ConjugateWithTranspose(const Mat32& A, const SymMat33& B) {
    return TransposeTimesWithSymmetricResult(B * A, A);
}

static void Print(Mat33 A, const char* name) {
    printf("%s\n", name);
    printf("%f %f %f\n", A.cols[0].x, A.cols[1].x, A.cols[2].x);
    printf("%f %f %f\n", A.cols[0].y, A.cols[1].y, A.cols[2].y);
    printf("%f %f %f\n", A.cols[0].z, A.cols[1].z, A.cols[2].z);
}
static void Print(Mat32 A, const char* name) {
    printf("%s\n", name);
    printf("%f %f\n", A.cols[0].x, A.cols[1].x);
    printf("%f %f\n", A.cols[0].y, A.cols[1].y);
    printf("%f %f\n", A.cols[0].z, A.cols[1].z);
}
static void Print(SymMat33 A, const char* name) {
    printf("%s\n", name);

    printf("%f %f %f\n", A.x11, A.x21, A.x31);
    printf("%f %f %f\n", A.x21, A.x22, A.x32);
    printf("%f %f %f\n", A.x31, A.x32, A.x33);
}
static void Print(SymMat22 A, const char* name) {
    printf("%s\n", name);

    printf("%f %f\n", A.x11, A.x21);
    printf("%f %f\n", A.x21, A.x22);
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
