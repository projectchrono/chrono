// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
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

#include "chrono_parallel/ChCudaDefines.h"
//#include "chrono_parallel/math/float.h"
namespace chrono {

#define OPERATOR_EQUALSALT(op, tin, tout)                            \
    static inline tout& operator op##=(tout & a, const tin& scale) { \
        a = a op scale;                                              \
        return a;                                                    \
    }

CUDA_HOST_DEVICE static inline float3 operator+(const float3& a, float b) {
    return make_float3(a.x + b, a.y + b, a.z + b);
}
CUDA_HOST_DEVICE static inline float3 operator-(const float3& a, float b) {
    return make_float3(a.x - b, a.y - b, a.z - b);
}
CUDA_HOST_DEVICE static inline float3 operator*(const float3& a, float b) {
    return make_float3(a.x * b, a.y * b, a.z * b);
}
CUDA_HOST_DEVICE static inline float3 operator*(float b, const float3& a) {
    return make_float3(a.x * b, a.y * b, a.z * b);
}
CUDA_HOST_DEVICE static inline float3 operator/(const float3& a, float b) {
    return make_float3(a.x / b, a.y / b, a.z / b);
}
CUDA_HOST_DEVICE static inline float3 operator/(float b, const float3& a) {
    return make_float3(a.x / b, a.y / b, a.z / b);
}
CUDA_HOST_DEVICE static inline float3 operator+(const float3& a, const float3& b) {
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}
CUDA_HOST_DEVICE static inline float3 operator-(const float3& a, const float3& b) {
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}
CUDA_HOST_DEVICE static inline float Dot(const float3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}
CUDA_HOST_DEVICE static inline float Dot(const float3& v, const float3& w) {
    return v.x * w.x + v.y * w.y + v.z * w.z;
}
CUDA_HOST_DEVICE static inline float Length2(const float3& v1) {
    return Dot(v1);
}

CUDA_HOST_DEVICE static inline float Length(const float3& v) {
    return sqrtf(Dot(v));
}
CUDA_HOST_DEVICE static inline float3 Sqrt(const float3& v) {
    return make_float3(sqrtf(v.x), sqrtf(v.y), sqrtf(v.z));
}
CUDA_HOST_DEVICE static inline float3 Abs(const float3& v) {
    return make_float3(fabsf(v.x), fabsf(v.y), fabsf(v.z));
}
CUDA_HOST_DEVICE static inline float3 Normalize(const float3& v) {
    return v / sqrtf(Dot(v));
}

CUDA_HOST_DEVICE static inline float3 Cross(const float3& a, const float3& b) {
    float3 result;
    result.x = (a.y * b.z) - (a.z * b.y);
    result.y = (a.z * b.x) - (a.x * b.z);
    result.z = (a.x * b.y) - (a.y * b.x);
    return result;
}

CUDA_HOST_DEVICE static inline float3 Max(const float3& a, const float3& b) {
    return make_float3(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z));
}
CUDA_HOST_DEVICE static inline float3 Max(const float3& a, float b) {
    return make_float3(fmaxf(a.x, b), fmaxf(a.y, b), fmaxf(a.z, b));
}
CUDA_HOST_DEVICE static inline float3 Min(const float3& a, const float3& b) {
    return make_float3(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z));
}
CUDA_HOST_DEVICE static inline float3 Min(const float3& a, float b) {
    return make_float3(fminf(a.x, b), fminf(a.y, b), fminf(a.z, b));
}

CUDA_HOST_DEVICE static inline float HorizontalAdd(const float3& a) {
    return a.x + a.y + a.z;
}

CUDA_HOST_DEVICE static inline float3 OrthogonalVector(const float3& v) {
    float3 abs = Abs(v);
    if (abs.x < abs.y) {
        return abs.x < abs.z ? make_float3(0, v.z, -v.y) : make_float3(v.y, -v.x, 0);
    } else {
        return abs.y < abs.z ? make_float3(-v.z, 0, v.x) : make_float3(v.y, -v.x, 0);
    }
}
CUDA_HOST_DEVICE static inline float3 UnitOrthogonalVector(const float3& v) {
    return Normalize(OrthogonalVector(v));
}

//=======
CUDA_HOST_DEVICE static inline float2 operator/(const float2& a, float b) {
    return make_float2(a.x / b, a.y / b);
}
CUDA_HOST_DEVICE static inline float Dot(const float2& v) {
    return v.x * v.x + v.y * v.y;
}
CUDA_HOST_DEVICE static inline float Length2(const float2& v1) {
    return Dot(v1);
}
CUDA_HOST_DEVICE static inline float2 Normalize(const float2& v) {
    return v / sqrtf(Dot(v));
}
class Mat33f {
  public:
    // Zero constructor
    CUDA_HOST_DEVICE Mat33f() : array{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} {}
    // Diagonal matrix constructor
    CUDA_HOST_DEVICE Mat33f(float v) : array{v, 0, 0, 0, 0, v, 0, 0, 0, 0, v, 0} {}
    // Diagonal matrix constructor
    CUDA_HOST_DEVICE Mat33f(float3 v) : array{v.x, 0, 0, 0, 0, v.y, 0, 0, 0, 0, v.z, 0} {}

    // Constructor that takes three columns of the matrix
    CUDA_HOST_DEVICE Mat33f(const float3& col1, const float3& col2, const float3& col3)
        : array{col1.x, col1.y, col1.z, 0, col2.x, col2.y, col2.z, 0, col3.x, col3.y, col3.z, 0} {}

    // Constructor that takes individial elements
    CUDA_HOST_DEVICE Mat33f(const float& v11,
                            const float& v21,
                            const float& v31,
                            const float& v12,
                            const float& v22,
                            const float& v32,
                            const float& v13,
                            const float& v23,
                            const float& v33)
        : array{v11, v21, v31, 0, v12, v22, v32, 0, v13, v23, v33, 0} {}
    // Copy constructor
    CUDA_HOST_DEVICE Mat33f(const Mat33f& M) { memcpy(array, M.array, 12 * sizeof(float)); }

    // Constructor that takes a quaternion and generates a rotation matrix
    //    CUDA_HOST_DEVICE Mat33f(const quaternion& q) {
    //        array[0] = float(1.0) - float(2.0) * q.y * q.y - float(2.0) * q.z * q.z;
    //        array[1] = float(2.0) * q.x * q.y + float(2.0) * q.z * q.w;
    //        array[2] = float(2.0) * q.x * q.z - float(2.0) * q.y * q.w;
    //
    //        array[4] = float(2.0) * q.x * q.y - float(2.0) * q.z * q.w;
    //        array[5] = float(1.0) - float(2.0) * q.x * q.x - float(2.0) * q.z * q.z;
    //        array[6] = float(2.0) * q.y * q.z + float(2.0) * q.x * q.w;
    //
    //        array[8] = float(2.0) * q.x * q.z + float(2.0) * q.y * q.w;
    //        array[9] = float(2.0) * q.y * q.z - float(2.0) * q.x * q.w;
    //        array[10] = float(1.0) - float(2.0) * q.x * q.x - float(2.0) * q.y * q.y;
    //    }

    CUDA_HOST_DEVICE inline float operator[](unsigned int i) const { return array[i]; }
    CUDA_HOST_DEVICE inline float& operator[](unsigned int i) { return array[i]; }
    CUDA_HOST_DEVICE inline float operator()(int i, int j) const { return array[j * 4 + i]; }
    CUDA_HOST_DEVICE inline float& operator()(int i, int j) { return array[j * 4 + i]; }
    CUDA_HOST_DEVICE inline float3 col(unsigned int i) const {
        return make_float3(array[i * 4], array[i * 4 + 1], array[i * 4 + 2]);
    }
    CUDA_HOST_DEVICE inline float3 row(unsigned int i) const {
        return make_float3(array[0 * 4 + i], array[1 * 4 + i], array[2 * 4 + i]);
    }

    CUDA_HOST_DEVICE inline Mat33f& operator=(const Mat33f& M) {
        array[0] = M.array[0];
        array[1] = M.array[1];
        array[2] = M.array[2];
        array[3] = M.array[3];
        array[4] = M.array[4];
        array[5] = M.array[5];
        array[6] = M.array[6];
        array[7] = M.array[7];
        array[8] = M.array[8];
        array[9] = M.array[9];
        array[10] = M.array[10];
        array[11] = M.array[11];
        return *this;
    }

    float array[12];

    // c1 c2 c3
    // 0  4  8
    // 1  5  9
    // 2  6  10

    // cols[0] 0 1 2
    // cols[1] 3 4 5
    // cols[2] 6 7 8
};
// ========================================================================================

CUDA_HOST_DEVICE float3 operator*(const Mat33f& M, const float3& v);
CUDA_HOST_DEVICE Mat33f operator*(const Mat33f& N, const float scale);
CUDA_HOST_DEVICE Mat33f operator*(const Mat33f& M, const Mat33f& N);
CUDA_HOST_DEVICE Mat33f operator+(const Mat33f& M, const Mat33f& N);
CUDA_HOST_DEVICE Mat33f operator-(const Mat33f& M, const Mat33f& N);

CUDA_HOST_DEVICE OPERATOR_EQUALSALT(*, float, Mat33f) CUDA_HOST_DEVICE
    OPERATOR_EQUALSALT(*, Mat33f, Mat33f) CUDA_HOST_DEVICE OPERATOR_EQUALSALT(+, Mat33f, Mat33f) CUDA_HOST_DEVICE
    OPERATOR_EQUALSALT(-, Mat33f, Mat33f)

        CUDA_HOST_DEVICE Mat33f
        operator-(const Mat33f& M);

CUDA_HOST_DEVICE static inline Mat33f operator*(const float s, const Mat33f& a) {
    return a * s;
}
CUDA_HOST_DEVICE Mat33f SkewSymmetric(const float3& r);
// This form generates a skew symmetric structure without flipping signs
CUDA_HOST_DEVICE Mat33f SkewSymmetricAlt(const float3& r);
CUDA_HOST_DEVICE float Determinant(const Mat33f& m);
CUDA_HOST_DEVICE Mat33f Abs(const Mat33f& m);
CUDA_HOST_DEVICE Mat33f Transpose(const Mat33f& a);
CUDA_HOST_DEVICE Mat33f MultTranspose(const Mat33f& M, const Mat33f& N);  // M * N^T
CUDA_HOST_DEVICE Mat33f TransposeMult(const Mat33f& M, const Mat33f& N);  // M^T * N
CUDA_HOST_DEVICE float Trace(const Mat33f& m);
CUDA_HOST_DEVICE Mat33f OuterProduct(const float3& a, const float3& b);  // Multiply a 3x1 by a 1x3 to get a 3x3
CUDA_HOST_DEVICE float InnerProduct(const Mat33f& A, const Mat33f& B);
CUDA_HOST_DEVICE Mat33f Adjoint(const Mat33f& A);
CUDA_HOST_DEVICE Mat33f AdjointTranspose(const Mat33f& A);
CUDA_HOST_DEVICE Mat33f Inverse(const Mat33f& A);
CUDA_HOST_DEVICE Mat33f InverseTranspose(const Mat33f& A);
CUDA_HOST_DEVICE float Norm(const Mat33f& A);  // normalized double dot product of a matrix
CUDA_HOST_DEVICE float DoubleDot(const Mat33f& A, const Mat33f& B);
CUDA_HOST_DEVICE float3 LargestColumnNormalized(const Mat33f& A);

// ========================================================================================
//
//// ========================================================================================
//
struct DiagMat33f {
    CUDA_HOST_DEVICE DiagMat33f() {}
    CUDA_HOST_DEVICE DiagMat33f(const DiagMat33f& M) : x11(M.x11), x22(M.x22), x33(M.x33) {}
    CUDA_HOST_DEVICE DiagMat33f(const float v11, const float v22, const float v33) : x11(v11), x22(v22), x33(v33) {}
    CUDA_HOST_DEVICE DiagMat33f(const float3 v) : x11(v.x), x22(v.y), x33(v.z) {}

    float x11, x22, x33;
};
CUDA_HOST_DEVICE Mat33f operator*(const DiagMat33f& M, const Mat33f& N);
CUDA_HOST_DEVICE float3 operator*(const DiagMat33f& M, const float3& v);

//// ========================================================================================
//

struct SymMat33f {
    CUDA_HOST_DEVICE SymMat33f() : x11(0), x21(0), x31(0), x22(0), x32(0), x33(0) {}

    CUDA_HOST_DEVICE SymMat33f(const float y11,
                               const float y21,
                               const float y31,
                               const float y22,
                               const float y32,
                               const float y33)
        : x11(y11), x21(y21), x31(y31), x22(y22), x32(y32), x33(y33) {}
    CUDA_HOST_DEVICE inline float operator[](unsigned int i) const { return array[i]; }
    CUDA_HOST_DEVICE inline float& operator[](unsigned int i) { return array[i]; }
    CUDA_HOST_DEVICE void operator=(const SymMat33f& N) {
        x11 = N.x11;
        x21 = N.x21;
        x31 = N.x31;
        x22 = N.x22;
        x32 = N.x32;
        x33 = N.x33;
    }
    union {
        float array[8];
        struct {
            float x11, x21, x31, x22, x32, x33;
        };
    };
};
CUDA_HOST_DEVICE SymMat33f operator-(const SymMat33f& M, const float& v);

//// ========================================================================================

struct Mat32f {
    CUDA_HOST_DEVICE Mat32f() {}
    CUDA_HOST_DEVICE Mat32f(const float3 col1, const float3 col2)
        : array{col1.x, col1.y, col1.z, 0, col2.x, col2.y, col2.z, 0} {}
    CUDA_HOST_DEVICE inline float operator[](unsigned int i) const { return array[i]; }
    CUDA_HOST_DEVICE inline float& operator[](unsigned int i) { return array[i]; }
    float array[8];
    // 0 4
    // 1 5
    // 2 6
    // 3 7 //Not used
};
CUDA_HOST_DEVICE float3 operator*(const Mat32f& M, const float2& v);

//// ========================================================================================

struct Mat23f {
    CUDA_HOST_DEVICE Mat23f() {}
    CUDA_HOST_DEVICE Mat23f(const float3 row1, const float3 row2)
        : array{row1.x, row1.y, row1.z, 0, row2.x, row2.y, row2.z, 0} {}
    float array[8];
    // 0 1 2 3
    // 4 5 6 7
};
// ========================================================================================

struct SymMat22f {
    CUDA_HOST_DEVICE SymMat22f() : x11(0), x21(0), x22(0) {}
    CUDA_HOST_DEVICE SymMat22f(const float v11, const float v21, const float v22) : x11(v11), x21(v21), x22(v22) {}

    float x11, x21, x22;
};

CUDA_HOST_DEVICE SymMat22f operator-(const SymMat22f& M, const float& v);
CUDA_HOST_DEVICE SymMat22f CofactorMatrix(const SymMat22f& A);
CUDA_HOST_DEVICE float2 LargestColumnNormalized(const SymMat22f& A);
//// ========================================================================================
//

// Compute the normal equations matrix - 18 mults, 12 adds
CUDA_HOST_DEVICE SymMat33f NormalEquationsMatrix(const Mat33f& A);
CUDA_HOST_DEVICE SymMat33f CofactorMatrix(const SymMat33f& A);
CUDA_HOST_DEVICE float3 LargestColumnNormalized(const SymMat33f& A);
// ========================================================================================
CUDA_HOST_DEVICE Mat32f operator*(const SymMat33f& M, const Mat32f& N);

//// A^T*B
CUDA_HOST_DEVICE SymMat22f TransposeTimesWithSymmetricResult(const Mat32f& A, const Mat32f& B);
//
CUDA_HOST_DEVICE SymMat22f ConjugateWithTranspose(const Mat32f& A, const SymMat33f& B);

CUDA_HOST_DEVICE void Print(const Mat33f& A, const char* name);
CUDA_HOST_DEVICE void Print(const Mat32f& A, const char* name);
CUDA_HOST_DEVICE void Print(const SymMat33f& A, const char* name);
CUDA_HOST_DEVICE void Print(const SymMat22f& A, const char* name);

CUDA_HOST_DEVICE void PrintLine(const Mat33f& A, const char* name);
CUDA_HOST_DEVICE void PrintLine(const Mat32f& A, const char* name);
CUDA_HOST_DEVICE void PrintLine(const SymMat33f& A, const char* name);
CUDA_HOST_DEVICE void PrintLine(const SymMat22f& A, const char* name);


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
