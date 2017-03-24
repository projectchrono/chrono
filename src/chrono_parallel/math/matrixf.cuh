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
#include <iostream>

//#include "chrono_parallel/math/float.h"
namespace chrono {

#if !defined(_WIN32)
#define FLT_EPSILON 1.19209290E-07F
#define FLT_MAX 3.40282347E+38F
#endif

#define OPERATOR_EQUALSALT(op, tin, tout)                           \
    static inline tout& operator op##=(tout& a, const tin& scale) { \
        a = a op scale;                                             \
        return a;                                                   \
    }

template <typename T>
CUDA_HOST_DEVICE static inline T Sign(const T& x) {
    if (x < 0) {
        return T(-1.0f);
    } else if (x > 0.0f) {
        return T(1.0f);
    } else {
        return T(0.0f);
    }
}
template <typename T>
CUDA_HOST_DEVICE static inline T Sqr(const T x) {
    return x * x;
}
template <typename T>
CUDA_HOST_DEVICE static inline T Cube(const T x) {
    return x * x * x;
}

template <typename T>
CUDA_HOST_DEVICE inline void Swap(T& a, T& b) {
    T temp = a;
    a = b;
    b = temp;
}

template <typename T>
CUDA_HOST_DEVICE void Sort(T& a, T& b, T& c) {
    if (a > b)
        Swap(a, b);
    if (b > c)
        Swap(b, c);
    if (a > b)
        Swap(a, b);
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
CUDA_HOST_DEVICE static inline float3 operator*(const float b, const float3& a) {
    return make_float3(a.x * b, a.y * b, a.z * b);
}
CUDA_HOST_DEVICE static inline float3 operator/(const float3& a, const float b) {
    return make_float3(a.x / b, a.y / b, a.z / b);
}
CUDA_HOST_DEVICE static inline float3 operator/(const float b, const float3& a) {
    return make_float3(b / a.x, b / a.y, b / a.z);
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
CUDA_HOST_DEVICE static inline float3 Max(const float3& a, const float b) {
    return make_float3(fmaxf(a.x, b), fmaxf(a.y, b), fmaxf(a.z, b));
}
CUDA_HOST_DEVICE static inline float3 Min(const float3& a, const float3& b) {
    return make_float3(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z));
}
CUDA_HOST_DEVICE static inline float3 Min(const float3& a, const float b) {
    return make_float3(fminf(a.x, b), fminf(a.y, b), fminf(a.z, b));
}

CUDA_HOST_DEVICE static inline float HorizontalAdd(const float3& a) {
    return a.x + a.y + a.z;
}

CUDA_HOST_DEVICE static inline float3 OrthogonalVector(const float3& v) {
    float3 abs = Abs(v);
    if (abs.x < abs.y) {
        return abs.x < abs.z ? make_float3(0.0f, v.z, -v.y) : make_float3(v.y, -v.x, 0.0f);
    } else {
        return abs.y < abs.z ? make_float3(-v.z, 0.0f, v.x) : make_float3(v.y, -v.x, 0.0f);
    }
}
CUDA_HOST_DEVICE static inline float3 UnitOrthogonalVector(const float3& v) {
    return Normalize(OrthogonalVector(v));
}

//=======
CUDA_HOST_DEVICE static inline float2 operator/(const float2& a, const float b) {
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

CUDA_HOST_DEVICE inline float Clamp(float x, float low, float high) {
    if (low > high) {
        Swap(low, high);
    }
    return fmaxf(low, fminf(x, high));
}

class Mat33f {
  public:
    // Zero constructor
    CUDA_HOST_DEVICE inline Mat33f() {}
    // Diagonal matrix constructor

    CUDA_HOST_DEVICE inline explicit Mat33f(float v)  {
		array[0] = v;
		array[1] = 0.0f;
		array[2] = 0.0f;
		array[3] = 0.0f;
		array[4] = v;
		array[5] = 0.0f;
		array[6] = 0.0f;
		array[7] = 0.0f;
		array[8] = v;
	}

    // Diagonal matrix constructor
    CUDA_HOST_DEVICE inline explicit Mat33f(float3 v) {
		array[0] = v.x;
		array[1] = 0.0f;
		array[2] = 0.0f;
		array[3] = 0.0f;
		array[4] = v.y;
		array[5] = 0.0f;
		array[6] = 0.0f;
		array[7] = 0.0f;
		array[8] = v.z;
	
	}
    // Coalesced load, each thread p loads a value with a stride
    CUDA_HOST_DEVICE inline explicit Mat33f(const float* N, const int p, const int s){	
		array[0] = N[p + s * 0];
		array[1] = N[p + s * 1];
		array[2] = N[p + s * 2];
		array[3] = N[p + s * 3];
		array[4] = N[p + s * 4];
		array[5] = N[p + s * 5];
		array[6] = N[p + s * 6];
		array[7] = N[p + s * 7];
		array[8] = N[p + s * 8];
	}

    // Constructor that takes three columns of the matrix
    CUDA_HOST_DEVICE inline Mat33f(const float3& col1, const float3& col2, const float3& col3){
		array[0] = col1.x;
		array[1] = col1.y;
		array[2] = col1.z;
		array[3] = col2.x;
		array[4] = col2.y;
		array[5] = col2.z;
		array[6] = col3.x;
		array[7] = col3.y;
		array[8] = col3.z;
	}

    // Constructor that takes individial elements
    CUDA_HOST_DEVICE inline Mat33f(const float& v11,
                                   const float& v21,
                                   const float& v31,
                                   const float& v12,
                                   const float& v22,
                                   const float& v32,
                                   const float& v13,
                                   const float& v23,
                                   const float& v33){
		array[0] = v11;
		array[1] = v21;
		array[2] = v31;
		array[3] = v12;
		array[4] = v22;
		array[5] = v32;
		array[6] = v13;
		array[7] = v23;
		array[8] = v33;
	}
    // Copy constructor
    CUDA_HOST_DEVICE inline Mat33f(const Mat33f& M) {
        array[0] = M.array[0];
        array[1] = M.array[1];
        array[2] = M.array[2];
        array[3] = M.array[3];
        array[4] = M.array[4];
        array[5] = M.array[5];
        array[6] = M.array[6];
        array[7] = M.array[7];
        array[8] = M.array[8];
    }

    CUDA_HOST_DEVICE inline float operator[](const unsigned int i) const { return array[i]; }
    CUDA_HOST_DEVICE inline float& operator[](const unsigned int i) { return array[i]; }
    CUDA_HOST_DEVICE inline float operator()(const int i, const int j) const { return array[j * 3 + i]; }
    CUDA_HOST_DEVICE inline float& operator()(const int i, const int j) { return array[j * 3 + i]; }
    CUDA_HOST_DEVICE inline float3 col(const unsigned int i) const {
        return make_float3(array[i * 3], array[i * 3 + 1], array[i * 3 + 2]);
    }
    CUDA_HOST_DEVICE inline float3 row(const unsigned int i) const {
        return make_float3(array[0 * 3 + i], array[1 * 3 + i], array[2 * 3 + i]);
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

        return *this;
    }

    CUDA_HOST_DEVICE inline void Store(float* N, const int p, const int s) {
        N[p + 0 * s] = array[0];
        N[p + 1 * s] = array[1];
        N[p + 2 * s] = array[2];
        N[p + 3 * s] = array[3];
        N[p + 4 * s] = array[4];
        N[p + 5 * s] = array[5];
        N[p + 6 * s] = array[6];
        N[p + 7 * s] = array[7];
        N[p + 8 * s] = array[8];
    }

    float array[9];

    // c1 c2 c3
    // 0  4  8
    // 1  5  9
    // 2  6  10

    // cols[0] 0 1 2
    // cols[1] 3 4 5
    // cols[2] 6 7 8
};
// ========================================================================================

// ========================================================================================
//
//// ========================================================================================
//
struct DiagMat33f {
    CUDA_HOST_DEVICE DiagMat33f() {}
    CUDA_HOST_DEVICE DiagMat33f(const DiagMat33f& M) : x11(M.x11), x22(M.x22), x33(M.x33) {}
    CUDA_HOST_DEVICE DiagMat33f(const float v11, const float v22, const float v33) : x11(v11), x22(v22), x33(v33) {}
    CUDA_HOST_DEVICE DiagMat33f(const float3& v) : x11(v.x), x22(v.y), x33(v.z) {}

    float x11, x22, x33;
};

//// ========================================================================================
//

struct SymMat33f {
    CUDA_HOST_DEVICE SymMat33f() {}
    CUDA_HOST_DEVICE SymMat33f(const float f){
		array[0] = f;
		array[1] = 0.0f;
		array[2] = 0.0f;
		array[3] = f;
		array[4] = 0.0f;
		array[5] = f;
	}
    CUDA_HOST_DEVICE SymMat33f(const float* N) {
		array[0] = N[0];
		array[1] = N[1];
		array[2] = N[2];
		array[3] = N[3];
		array[4] = N[4];
		array[5] = N[5];
	}
    CUDA_HOST_DEVICE SymMat33f(const float y11,
                               const float y21,
                               const float y31,
                               const float y22,
                               const float y32,
                               const float y33)
        : x11(y11), x21(y21), x31(y31), x22(y22), x32(y32), x33(y33) {}
    CUDA_HOST_DEVICE inline float operator[](const unsigned int i) const { return array[i]; }
    CUDA_HOST_DEVICE inline float& operator[](const unsigned int i) { return array[i]; }
    CUDA_HOST_DEVICE void operator=(const SymMat33f& N) {
        x11 = N.x11;
        x21 = N.x21;
        x31 = N.x31;
        x22 = N.x22;
        x32 = N.x32;
        x33 = N.x33;
    }
    CUDA_HOST_DEVICE void operator=(const Mat33f& N) {
        x11 = N[0];
        x21 = N[1];
        x31 = N[2];
        x22 = N[4];
        x32 = N[5];
        x33 = N[8];
    }
    CUDA_HOST_DEVICE void Load(const float* N) {
        x11 = N[0];
        x21 = N[1];
        x31 = N[2];
        x22 = N[3];
        x32 = N[4];
        x33 = N[5];
    }
    // 0--
    // 13-
    // 245
    union {
        float array[6];
        struct {
            float x11, x21, x31, x22, x32, x33;
            //    0     1    2    3    4    5
        };
    };
};

//// ========================================================================================

struct Mat32f {
    CUDA_HOST_DEVICE Mat32f() {}
    CUDA_HOST_DEVICE Mat32f(const float3& col1, const float3& col2){
		array[0] = col1.x;
		array[1] = col1.y;
		array[2] = col1.z;
		array[3] = col2.x;
		array[4] = col2.y;
		array[5] = col2.z;
	}
    CUDA_HOST_DEVICE inline float operator[](const unsigned int i) const { return array[i]; }
    CUDA_HOST_DEVICE inline float& operator[](const unsigned int i) { return array[i]; }
    float array[6];
    // 0 3
    // 1 4
    // 2 5
};

//// ========================================================================================

struct Mat23f {
    CUDA_HOST_DEVICE Mat23f() {}
    CUDA_HOST_DEVICE Mat23f(const float3 row1, const float3 row2){
		array[0] = row1.x;
		array[1] = row1.y;
		array[2] = row1.z;
		array[3] = row2.x;
		array[4] = row2.y;
		array[5] = row2.z;
	}
    float array[6];
    // 0 1 2
    // 3 4 5
};
// ========================================================================================

struct SymMat22f {
    CUDA_HOST_DEVICE SymMat22f() {}
    CUDA_HOST_DEVICE SymMat22f(const float f) : x11(f), x21(f), x22(f) {}
    CUDA_HOST_DEVICE SymMat22f(const float v11, const float v21, const float v22) : x11(v11), x21(v21), x22(v22) {}

    float x11, x21, x22;
};

// dot product of each column of a matrix with itself
CUDA_HOST_DEVICE static inline float3 DotMM(const Mat33f& M) {
    float3 result;
    result.x = M[0] * M[0] + M[1] * M[1] + M[2] * M[2];
    result.y = M[3] * M[3] + M[4] * M[4] + M[5] * M[5];
    result.z = M[6] * M[6] + M[7] * M[7] + M[8] * M[8];
    return result;
}  // dot product of each column of a matrix with another matrix
CUDA_HOST_DEVICE static inline float3 DotMM(const Mat33f& M, const Mat33f& N) {
    float3 result;
    result.x = M[0] * N[0] + M[1] * N[1] + M[2] * N[2];
    result.y = M[3] * N[3] + M[4] * N[4] + M[5] * N[5];
    result.z = M[6] * N[6] + M[7] * N[7] + M[8] * N[8];
    return result;
}

// CUDA_HOST_DEVICE inline Mat33f OuterProductVV(const float* A, const float* B) {
//    return Mat33f(A[0] * B[0], A[1] * B[0], A[2] * B[0], A[0] * B[1], A[1] * B[1], A[2] * B[1], A[0] * B[2],
//                  A[1] * B[2], A[2] * B[2]);
//}

//[0,4,8 ]
//[1,5,9 ]
//[2,6,10]
//[3,7,11]
CUDA_HOST_DEVICE static inline float3 operator*(const Mat33f& M, const float3& N) {
    float3 r;
    r.x = M[0] * N.x + M[3] * N.y + M[6] * N.z;
    r.y = M[1] * N.x + M[4] * N.y + M[7] * N.z;
    r.z = M[2] * N.x + M[5] * N.y + M[8] * N.z;
    return r;
}

CUDA_HOST_DEVICE static inline Mat33f operator*(const Mat33f& M, const float b) {
    Mat33f r;
    r[0] = M[0] * b;
    r[1] = M[1] * b;
    r[2] = M[2] * b;
    r[3] = M[3] * b;
    r[4] = M[4] * b;
    r[5] = M[5] * b;
    r[6] = M[6] * b;
    r[7] = M[7] * b;
    r[8] = M[8] * b;
    return r;
}

CUDA_HOST_DEVICE static inline Mat33f operator*(const Mat33f& M, const Mat33f& N) {
    Mat33f r;
    r[0] = M[0] * N[0] + M[3] * N[1] + M[6] * N[2];
    r[1] = M[1] * N[0] + M[4] * N[1] + M[7] * N[2];
    r[2] = M[2] * N[0] + M[5] * N[1] + M[8] * N[2];

    r[3] = M[0] * N[3] + M[3] * N[4] + M[6] * N[5];
    r[4] = M[1] * N[3] + M[4] * N[4] + M[7] * N[5];
    r[5] = M[2] * N[3] + M[5] * N[4] + M[8] * N[5];

    r[6] = M[0] * N[6] + M[3] * N[7] + M[6] * N[8];
    r[7] = M[1] * N[6] + M[4] * N[7] + M[7] * N[8];
    r[8] = M[2] * N[6] + M[5] * N[7] + M[8] * N[8];
    return r;
}
// Multiply by the skew symmetric form of N
CUDA_HOST_DEVICE static inline Mat33f MultSkew(const Mat33f& M, const float3& N) {
    Mat33f r;
    r[0] = M[3] * N.z + M[6] * -N.y;
    r[1] = M[4] * N.z + M[7] * -N.y;
    r[2] = M[5] * N.z + M[8] * -N.y;

    r[3] = M[0] * -N.z + M[6] * N.x;
    r[4] = M[1] * -N.z + M[7] * N.x;
    r[5] = M[2] * -N.z + M[8] * N.x;

    r[6] = M[0] * N.y + M[3] * -N.x;
    r[7] = M[1] * N.y + M[4] * -N.x;
    r[8] = M[2] * N.y + M[5] * -N.x;
    return r;
}

CUDA_HOST_DEVICE static inline Mat33f operator+(const Mat33f& M, const Mat33f& N) {
    return Mat33f(M[0] + N[0], M[1] + N[1], M[2] + N[2], M[3] + N[3], M[4] + N[4], M[5] + N[5], M[6] + N[6],
                  M[7] + N[7], M[8] + N[8]);
}

CUDA_HOST_DEVICE static inline Mat33f operator-(const Mat33f& M, const Mat33f& N) {
    return Mat33f(M[0] - N[0], M[1] - N[1], M[2] - N[2], M[3] - N[3], M[4] - N[4], M[5] - N[5], M[6] - N[6],
                  M[7] - N[7], M[8] - N[8]);
}
CUDA_HOST_DEVICE static inline Mat33f operator-(const Mat33f& M) {
    return Mat33f(-M[0], -M[1], -M[2], -M[3], -M[4], -M[5], -M[6], -M[7], -M[8]);
}
CUDA_HOST_DEVICE static inline Mat33f Abs(const Mat33f& M) {
    return Mat33f(fabsf(M[0]), fabsf(M[1]), fabsf(M[2]), fabsf(M[3]), fabsf(M[4]), fabsf(M[5]), fabsf(M[6]),
                  fabsf(M[7]), fabsf(M[8]));
}
CUDA_HOST_DEVICE static inline Mat33f SkewSymmetric(const float3& r) {
    return Mat33f(0, r.z, -r.y, -r.z, 0, r.x, r.y, -r.x, 0);
}
CUDA_HOST_DEVICE static inline Mat33f SkewSymmetricAlt(const float3& r) {
    return Mat33f(0, r.z, r.y, r.z, 0, r.x, r.y, r.x, 0);
}

CUDA_HOST_DEVICE static inline Mat33f Transpose(const Mat33f& a) {
    return Mat33f(a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]);
}

CUDA_HOST_DEVICE static inline Mat33f MultTranspose(const Mat33f& M, const Mat33f& N) {
    Mat33f r;
    r[0] = M[0] * N[0] + M[3] * N[3] + M[6] * N[6];
    r[1] = M[1] * N[0] + M[4] * N[3] + M[7] * N[6];
    r[2] = M[2] * N[0] + M[5] * N[3] + M[8] * N[6];

    r[3] = M[0] * N[1] + M[3] * N[4] + M[6] * N[7];
    r[4] = M[1] * N[1] + M[4] * N[4] + M[7] * N[7];
    r[5] = M[2] * N[1] + M[5] * N[4] + M[8] * N[7];

    r[6] = M[0] * N[2] + M[3] * N[5] + M[6] * N[8];
    r[7] = M[1] * N[2] + M[4] * N[5] + M[7] * N[8];
    r[8] = M[2] * N[2] + M[5] * N[5] + M[8] * N[8];
    return r;
}

// Multiply by diagonal matrix
CUDA_HOST_DEVICE static inline Mat33f MultTranspose(const float3& M, const Mat33f& N) {
    Mat33f r;
    r[0] = M.x * N[0];
    r[1] = M.y * N[3];
    r[2] = M.z * N[6];

    r[3] = M.x * N[1];
    r[4] = M.y * N[4];
    r[5] = M.z * N[7];

    r[6] = M.x * N[2];
    r[7] = M.y * N[5];
    r[8] = M.z * N[8];
    return r;
}

CUDA_HOST_DEVICE static inline Mat33f TransposeMult(const Mat33f& M, const Mat33f& N) {
    // c1 c2 c3    // c1 c2 c3
    // 0  1  2     // 0  4  8
    // 4  5  6     // 1  5  9
    // 8  9  10    // 2  6  10

    Mat33f r;
    r[0] = M[0] * N[0] + M[1] * N[1] + M[2] * N[2];
    r[1] = M[3] * N[0] + M[4] * N[1] + M[5] * N[2];
    r[2] = M[6] * N[0] + M[7] * N[1] + M[8] * N[2];

    r[3] = M[0] * N[3] + M[1] * N[4] + M[2] * N[5];
    r[4] = M[3] * N[3] + M[4] * N[4] + M[5] * N[5];
    r[5] = M[6] * N[3] + M[7] * N[4] + M[8] * N[5];

    r[6] = M[0] * N[6] + M[1] * N[7] + M[2] * N[8];
    r[7] = M[3] * N[6] + M[4] * N[7] + M[5] * N[8];
    r[8] = M[6] * N[6] + M[7] * N[7] + M[8] * N[8];
    return r;
}

CUDA_HOST_DEVICE static inline float Trace(const Mat33f& m) {
    return m[0] + m[4] + m[8];
}
// Multiply a 3x1 by a 1x3 to get a 3x3
CUDA_HOST_DEVICE static inline Mat33f OuterProduct(const float3& A, const float3& B) {
    return Mat33f(A.x * B.x, A.y * B.x, A.z * B.x, A.x * B.y, A.y * B.y, A.z * B.y, A.x * B.z, A.y * B.z, A.z * B.z);
}

CUDA_HOST_DEVICE static inline float InnerProduct(const Mat33f& A, const Mat33f& B) {
    return HorizontalAdd(DotMM(A, B));
}

CUDA_HOST_DEVICE static inline Mat33f Adjoint(const Mat33f& A) {
    Mat33f T;
    T[0] = A[4] * A[8] - A[7] * A[5];
    T[1] = -A[1] * A[8] + A[7] * A[2];
    T[2] = A[1] * A[5] - A[4] * A[2];

    T[3] = -A[3] * A[8] + A[6] * A[5];
    T[4] = A[0] * A[8] - A[6] * A[2];
    T[5] = -A[0] * A[5] + A[3] * A[2];

    T[6] = A[3] * A[7] - A[6] * A[4];
    T[7] = -A[0] * A[7] + A[6] * A[1];
    T[8] = A[0] * A[4] - A[3] * A[1];
    return T;
}

CUDA_HOST_DEVICE static inline Mat33f AdjointTranspose(const Mat33f& A) {
    Mat33f T;
    T[0] = A[4] * A[8] - A[7] * A[5];
    T[1] = -A[3] * A[8] + A[6] * A[5];
    T[2] = A[3] * A[7] - A[6] * A[4];

    T[3] = -A[1] * A[8] + A[7] * A[2];
    T[4] = A[0] * A[8] - A[6] * A[2];
    T[5] = -A[0] * A[7] + A[6] * A[1];

    T[6] = A[1] * A[5] - A[4] * A[2];
    T[7] = -A[0] * A[5] + A[3] * A[2];
    T[8] = A[0] * A[4] - A[3] * A[1];

    return T;
}

CUDA_HOST_DEVICE static inline float Determinant(const Mat33f& m) {
    return m[0] * (m[4] * m[8] - m[7] * m[5]) - m[3] * (m[1] * m[8] - m[7] * m[2]) + m[6] * (m[1] * m[5] - m[4] * m[2]);
}

CUDA_HOST_DEVICE static inline Mat33f Inverse(const Mat33f& A) {
    float s = Determinant(A);
    if (s > 0.0f) {
        return Adjoint(A) * (1.0f / s);
    } else {
        return Mat33f(0.0f);
    }
}

// Same as inverse but we store it transposed
CUDA_HOST_DEVICE static inline Mat33f InverseTranspose(const Mat33f& A) {
    float s = Determinant(A);
    if (s > 0.0f) {
        return AdjointTranspose(A) * (1.0f / s);
    } else {
        return Mat33f(0.0f);
    }
}
CUDA_HOST_DEVICE static inline Mat33f InverseUnsafe(const Mat33f& A) {
    float s = Determinant(A);
    return Adjoint(A) * float(1.0f / s);
}

// Same as inverse but we store it transposed
CUDA_HOST_DEVICE static inline Mat33f InverseTransposeUnsafe(const Mat33f& A) {
    float s = Determinant(A);
    return AdjointTranspose(A) * float(1.0f / s);
}
CUDA_HOST_DEVICE static inline float Norm(const Mat33f& A) {
    float x = A[0] * A[0] + A[3] * A[3] + A[6] * A[6];
    float y = A[1] * A[1] + A[4] * A[4] + A[7] * A[7];
    float z = A[2] * A[2] + A[5] * A[5] + A[8] * A[8];
    return sqrtf(x + y + z);
}
// 9add
CUDA_HOST_DEVICE static inline float DoubleDot(const Mat33f& A, const Mat33f& B) {
    return A[0] * B[0] + A[1] * B[1] + A[2] * B[2] + A[3] * B[3] + A[4] * B[4] + A[5] * B[5] + A[6] * B[6] +
           A[7] * B[7] + A[8] * B[8];
}

CUDA_HOST_DEVICE static inline float3 LargestColumnNormalized(const Mat33f& A) {
    float3 scale = DotMM(A);
    float3 sqrt_scale = Sqrt(scale);
    if (scale.x > scale.y) {
        if (scale.x > scale.z) {
            return A.col(0) / sqrt_scale.x;
        }
    } else if (scale.y > scale.z) {
        return A.col(1) / sqrt_scale.y;
    }
    return A.col(2) / sqrt_scale.z;
}
//// ========================================================================================

CUDA_HOST_DEVICE static inline Mat33f operator*(const DiagMat33f& M, const Mat33f& N) {
    return Mat33f(M.x11 * N[0], M.x22 * N[1], M.x33 * N[2], M.x11 * N[3], M.x22 * N[4], M.x33 * N[5], M.x11 * N[6],
                  M.x22 * N[7], M.x33 * N[8]);
}
CUDA_HOST_DEVICE static inline float3 operator*(const DiagMat33f& M, const float3& v) {
    float3 result;
    result.x = M.x11 * v.x;
    result.y = M.x22 * v.y;
    result.z = M.x33 * v.z;
    return result;
}
//// ========================================================================================
CUDA_HOST_DEVICE static inline SymMat33f operator-(const SymMat33f& M, const float v) {
    return SymMat33f(M.x11 - v, M.x21, M.x31, M.x22 - v, M.x32, M.x33 - v);  // only subtract diagonal
}
CUDA_HOST_DEVICE static inline SymMat33f CofactorMatrix(const SymMat33f& A) {
    SymMat33f T;
    T.x11 = A.x22 * A.x33 - A.x32 * A.x32;   //
    T.x21 = -A.x21 * A.x33 + A.x32 * A.x31;  //
    T.x22 = A.x11 * A.x33 - A.x31 * A.x31;   //
    T.x31 = A.x21 * A.x32 - A.x22 * A.x31;   //
    T.x32 = -A.x11 * A.x32 + A.x21 * A.x31;  //
    T.x33 = A.x11 * A.x22 - A.x21 * A.x21;   //
    return T;
}
CUDA_HOST_DEVICE static inline float3 LargestColumnNormalized(const SymMat33f& A) {
    float scale1 = Length2(make_float3(A.x11, A.x21, A.x31));
    float scale2 = Length2(make_float3(A.x21, A.x22, A.x32));
    float scale3 = Length2(make_float3(A.x31, A.x32, A.x33));
    if (scale1 > scale2) {
        if (scale1 > scale3) {
            return make_float3(A.x11, A.x21, A.x31) / sqrtf(scale1);
        }
    } else if (scale2 > scale3) {
        return make_float3(A.x21, A.x22, A.x32) / sqrtf(scale2);
    }
    if (scale3 > 0.0f)
        return make_float3(A.x31, A.x32, A.x33) / sqrtf(scale3);
    else {
        return (make_float3(1.0f, 0.0f, 0.0f));
    }
}
CUDA_HOST_DEVICE static inline SymMat33f NormalEquationsMatrix(const Mat33f& A) {
    SymMat33f T;

    T.x11 = A[0] * A[0] + A[1] * A[1] + A[2] * A[2];
    T.x21 = A[0] * A[3] + A[1] * A[4] + A[2] * A[5];
    T.x31 = A[0] * A[6] + A[1] * A[7] + A[2] * A[8];
    T.x22 = A[3] * A[3] + A[4] * A[4] + A[5] * A[5];
    T.x32 = A[3] * A[6] + A[4] * A[7] + A[5] * A[8];
    T.x33 = A[6] * A[6] + A[7] * A[7] + A[8] * A[8];

    return T;
}
//// ========================================================================================

CUDA_HOST_DEVICE static inline float3 operator*(const Mat32f& M, const float2& v) {
    float3 result;
    result.x = M[0] * v.x + M[3] * v.y;
    result.y = M[1] * v.x + M[4] * v.y;
    result.z = M[2] * v.x + M[5] * v.y;

    return result;
}
CUDA_HOST_DEVICE static inline Mat32f operator*(const SymMat33f& M, const Mat32f& N) {
    Mat32f result;
    // x11 x21 x31  c11 c12
    // x21 x22 x32  c21 c22
    // x31 x32 x33  c31 c32
    result[0] = M.x11 * N[0] + M.x21 * N[1] + M.x31 * N[2];
    result[1] = M.x21 * N[0] + M.x22 * N[1] + M.x32 * N[2];
    result[2] = M.x31 * N[0] + M.x32 * N[1] + M.x33 * N[2];

    result[3] = M.x11 * N[3] + M.x21 * N[4] + M.x31 * N[5];
    result[4] = M.x21 * N[3] + M.x22 * N[4] + M.x32 * N[5];
    result[5] = M.x31 * N[3] + M.x32 * N[4] + M.x33 * N[5];

    return result;
}
//// ========================================================================================
CUDA_HOST_DEVICE static inline SymMat22f operator-(const SymMat22f& M, const float v) {
    return SymMat22f(M.x11 - v, M.x21, M.x22 - v);  //
}
CUDA_HOST_DEVICE static inline SymMat22f CofactorMatrix(const SymMat22f& A) {
    SymMat22f T;
    T.x11 = A.x22;   //
    T.x21 = -A.x21;  //
    T.x22 = A.x11;   //
    return T;
}
CUDA_HOST_DEVICE static inline float2 LargestColumnNormalized(const SymMat22f& A) {
    float scale1 = Length2(make_float2(A.x11, A.x21));
    float scale2 = Length2(make_float2(A.x21, A.x22));
    if (scale1 > scale2) {
        return make_float2(A.x11, A.x21) / sqrtf(scale1);
    } else if (scale2 > 0.0f) {
        return make_float2(A.x21, A.x22) / sqrtf(scale2);
    } else {
        return make_float2(1.0f, 0.0f);
    }
}

// A^T*B
CUDA_HOST_DEVICE static inline SymMat22f TransposeTimesWithSymmetricResult(const Mat32f& A, const Mat32f& B) {
    SymMat22f T;
    T.x11 = A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
    T.x21 = A[3] * B[0] + A[4] * B[1] + A[5] * B[2];
    T.x22 = A[3] * B[3] + A[4] * B[4] + A[5] * B[5];

    return T;
}

CUDA_HOST_DEVICE static inline SymMat22f ConjugateWithTranspose(const Mat32f& A, const SymMat33f& B) {
    return TransposeTimesWithSymmetricResult(B * A, A);
}

CUDA_HOST_DEVICE void static Print(const Mat33f& A, const char* name) {
    printf("%s\n", name);
    printf("%f %f %f\n", A[0], A[3], A[6]);
    printf("%f %f %f\n", A[1], A[4], A[7]);
    printf("%f %f %f\n", A[2], A[5], A[8]);
}
CUDA_HOST_DEVICE void static Print(const Mat32f& A, const char* name) {
    printf("%s\n", name);
    printf("%f %f\n", A[0], A[3]);
    printf("%f %f\n", A[1], A[4]);
    printf("%f %f\n", A[2], A[5]);
}
CUDA_HOST_DEVICE void static Print(const SymMat33f& A, const char* name) {
    printf("%s\n", name);

    printf("%f %f %f\n", A.x11, A.x21, A.x31);
    printf("%f %f %f\n", A.x21, A.x22, A.x32);
    printf("%f %f %f\n", A.x31, A.x32, A.x33);
}
CUDA_HOST_DEVICE void static Print(const SymMat22f& A, const char* name) {
    printf("%s\n", name);

    printf("%f %f\n", A.x11, A.x21);
    printf("%f %f\n", A.x21, A.x22);
}

CUDA_HOST_DEVICE void static PrintLine(const Mat33f& A, const char* name) {
    printf("%s: [%f,%f,%f,%f,%f,%f,%f,%f,%f]\n", name, A[0], A[1], A[2], A[3], A[4], A[5], A[6], A[7], A[8]);
}
CUDA_HOST_DEVICE void static PrintLine(const Mat32f& A, const char* name) {
    printf("%s: [%f,%f,%f,%f,%f,%f]\n", name, A[0], A[1], A[2], A[3], A[4], A[5]);
}
CUDA_HOST_DEVICE void static PrintLine(const SymMat33f& A, const char* name) {
    printf("%s: [%f,%f,%f,%f,%f,%f,%f,%f,%f]\n", name, A.x11, A.x21, A.x31, A.x21, A.x22, A.x32, A.x31, A.x32, A.x33);
}
CUDA_HOST_DEVICE void static PrintLine(const SymMat22f& A, const char* name) {
    printf("%s: [%f,%f,%f,%f]\n", name, A.x11, A.x21, A.x21, A.x22);
}

CUDA_HOST_DEVICE OPERATOR_EQUALSALT(*, float, Mat33f)       //
    CUDA_HOST_DEVICE OPERATOR_EQUALSALT(*, Mat33f, Mat33f)  //
    CUDA_HOST_DEVICE OPERATOR_EQUALSALT(+, Mat33f, Mat33f)  //
    CUDA_HOST_DEVICE OPERATOR_EQUALSALT(-, Mat33f, Mat33f)

        CUDA_HOST_DEVICE static inline Mat33f
        operator*(const float s, const Mat33f& a) {
    return a * s;
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
