
#include "chrono_parallel/math/matrixf.cuh"

#include <iostream>

namespace chrono {

// dot product of each column of a matrix with itself
CUDA_HOST_DEVICE inline float3 DotMM(const float* M) {
    float3 result;
    result.x = M[0] * M[0] + M[1] * M[1] + M[2] * M[2];
    result.y = M[3] * M[3] + M[4] * M[4] + M[5] * M[5];
    result.z = M[6] * M[6] + M[7] * M[7] + M[8] * M[8];
    return result;
}  // dot product of each column of a matrix with another matrix
CUDA_HOST_DEVICE inline float3 DotMM(const float* M, const float* N) {
    float3 result;
    result.x = M[0] * N[0] + M[1] * N[1] + M[2] * N[2];
    result.y = M[3] * N[3] + M[4] * N[4] + M[5] * N[5];
    result.z = M[6] * N[6] + M[7] * N[7] + M[8] * N[8];
    return result;
}
CUDA_HOST_DEVICE inline Mat33f MulMM(const float* M, const float* N) {
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

CUDA_HOST_DEVICE inline Mat33f MulM_TM(const float* M, const float* N) {
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

CUDA_HOST_DEVICE inline float3 MulMV(const float* M, const float* N) {
    float3 r;
    r.x = M[0] * N[0] + M[3] * N[1] + M[6] * N[2];
    r.y = M[1] * N[0] + M[4] * N[1] + M[7] * N[2];
    r.z = M[2] * N[0] + M[5] * N[1] + M[8] * N[2];

    return r;
}

// CUDA_HOST_DEVICE inline Mat33f OuterProductVV(const float* A, const float* B) {
//    return Mat33f(A[0] * B[0], A[1] * B[0], A[2] * B[0], A[0] * B[1], A[1] * B[1], A[2] * B[1], A[0] * B[2],
//                  A[1] * B[2], A[2] * B[2]);
//}

CUDA_HOST_DEVICE inline Mat33f ScaleMat(const float* M, const float b) {
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

CUDA_HOST_DEVICE inline Mat33f MAbs(const float* M) {
    return Mat33f(fabsf(M[0]), fabsf(M[1]), fabsf(M[2]), fabsf(M[3]), fabsf(M[4]), fabsf(M[5]), fabsf(M[6]),
                  fabsf(M[7]), fabsf(M[8]));
}

//[0,4,8 ]
//[1,5,9 ]
//[2,6,10]
//[3,7,11]
CUDA_HOST_DEVICE float3 operator*(const Mat33f& M, const float3& N) {
    float3 r;
    r.x = M[0] * N.x + M[3] * N.y + M[6] * N.z;
    r.y = M[1] * N.x + M[4] * N.y + M[7] * N.z;
    r.z = M[2] * N.x + M[5] * N.y + M[8] * N.z;
    return r;
}

CUDA_HOST_DEVICE Mat33f operator*(const Mat33f& N, const float scale) {
    return ScaleMat(N.array, scale);
}

CUDA_HOST_DEVICE Mat33f operator*(const Mat33f& M, const Mat33f& N) {
    return MulMM(M.array, N.array);
}
CUDA_HOST_DEVICE Mat33f operator+(const Mat33f& M, const Mat33f& N) {
    return Mat33f(M[0] + N[0], M[1] + N[1], M[2] + N[2], M[3] + N[3], M[4] + N[4], M[5] + N[5], M[6] + N[6],
                  M[7] + N[7], M[8] + N[8]);
}

CUDA_HOST_DEVICE Mat33f operator-(const Mat33f& M, const Mat33f& N) {
    return Mat33f(M[0] - N[0], M[1] - N[1], M[2] - N[2], M[3] - N[3], M[4] - N[4], M[5] - N[5], M[6] - N[6],
                  M[7] - N[7], M[8] - N[8]);
}
CUDA_HOST_DEVICE Mat33f operator-(const Mat33f& M) {
    return Mat33f(-M[0], -M[1], -M[2], -M[3], -M[4], -M[5], -M[6], -M[7], -M[8]);
}
CUDA_HOST_DEVICE Mat33f Abs(const Mat33f& m) {
    return MAbs(m.array);
}
CUDA_HOST_DEVICE Mat33f SkewSymmetric(const float3& r) {
    return Mat33f(0, r.z, -r.y, -r.z, 0, r.x, r.y, -r.x, 0);
}
CUDA_HOST_DEVICE Mat33f SkewSymmetricAlt(const float3& r) {
    return Mat33f(0, r.z, r.y, r.z, 0, r.x, r.y, r.x, 0);
}
CUDA_HOST_DEVICE Mat33f MultTranspose(const Mat33f& M, const Mat33f& N) {
    // Not a clean way to write this in AVX, might as well transpose first and then multiply
    return M * Transpose(N);
}

CUDA_HOST_DEVICE Mat33f TransposeMult(const Mat33f& M, const Mat33f& N) {
    return MulM_TM(M.array, N.array);
}

CUDA_HOST_DEVICE Mat33f Transpose(const Mat33f& a) {
    return Mat33f(a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]);
}

CUDA_HOST_DEVICE float Trace(const Mat33f& m) {
    return m[0] + m[4] + m[8];
}
// Multiply a 3x1 by a 1x3 to get a 3x3
CUDA_HOST_DEVICE Mat33f OuterProduct(const float3& A, const float3& B) {
    return Mat33f(A.x * B.x, A.y * B.x, A.z * B.x, A.x * B.y, A.y * B.y, A.z * B.y, A.x * B.z, A.y * B.z, A.z * B.z);
}

CUDA_HOST_DEVICE float InnerProduct(const Mat33f& A, const Mat33f& B) {
    return HorizontalAdd(DotMM(A.array, B.array));
}

CUDA_HOST_DEVICE Mat33f Adjoint(const Mat33f& A) {
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

CUDA_HOST_DEVICE Mat33f AdjointTranspose(const Mat33f& A) {
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

CUDA_HOST_DEVICE float Determinant(const Mat33f& m) {
    return m[0] * (m[4] * m[8] - m[7] * m[5]) - m[3] * (m[1] * m[8] - m[7] * m[2]) + m[6] * (m[1] * m[5] - m[4] * m[2]);
}

CUDA_HOST_DEVICE Mat33f Inverse(const Mat33f& A) {
    float s = Determinant(A);
    if (s > 0.0) {
        return Adjoint(A) * float(1.0 / s);
    } else {
        return Mat33f(0);
    }
}

// Same as inverse but we store it transposed
CUDA_HOST_DEVICE Mat33f InverseTranspose(const Mat33f& A) {
    float s = Determinant(A);
    if (s > 0.0) {
        return AdjointTranspose(A) * float(1.0 / s);
    } else {
        return Mat33f(0);
    }
}

CUDA_HOST_DEVICE float Norm(const Mat33f& A) {
    return sqrtf(Trace(A * Transpose(A)));
}

CUDA_HOST_DEVICE float DoubleDot(const Mat33f& A, const Mat33f& B) {
    return A[0] * B[0] + A[1] * B[1] + A[2] * B[2] + A[3] * B[3] + A[4] * B[4] + A[5] * B[5] + A[6] * B[6] +
           A[7] * B[7] + A[8] * B[8];
}

CUDA_HOST_DEVICE float3 LargestColumnNormalized(const Mat33f& A) {
    float3 scale = DotMM(A.array);
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

CUDA_HOST_DEVICE Mat33f operator*(const DiagMat33f& M, const Mat33f& N) {
    return Mat33f(M.x11 * N[0], M.x22 * N[1], M.x33 * N[2], M.x11 * N[3], M.x22 * N[4], M.x33 * N[5], M.x11 * N[6],
                  M.x22 * N[7], M.x33 * N[8]);
}
CUDA_HOST_DEVICE float3 operator*(const DiagMat33f& M, const float3& v) {
    float3 result;
    result.x = M.x11 * v.x;
    result.y = M.x22 * v.y;
    result.z = M.x33 * v.z;
    return result;
}
//// ========================================================================================
CUDA_HOST_DEVICE SymMat33f operator-(const SymMat33f& M, const float& v) {
    return SymMat33f(M.x11 - v, M.x21, M.x31, M.x22 - v, M.x32, M.x33 - v);  // only subtract diagonal
}
CUDA_HOST_DEVICE SymMat33f CofactorMatrix(const SymMat33f& A) {
    SymMat33f T;
    T.x11 = A.x22 * A.x33 - A.x32 * A.x32;   //
    T.x21 = -A.x21 * A.x33 + A.x32 * A.x31;  //
    T.x22 = A.x11 * A.x33 - A.x31 * A.x31;   //
    T.x31 = A.x21 * A.x32 - A.x22 * A.x31;   //
    T.x32 = -A.x11 * A.x32 + A.x21 * A.x31;  //
    T.x33 = A.x11 * A.x22 - A.x21 * A.x21;   //
    return T;
}
CUDA_HOST_DEVICE float3 LargestColumnNormalized(const SymMat33f& A) {
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
    if (scale3 > 0)
        return make_float3(A.x31, A.x32, A.x33) / sqrtf(scale3);
    else {
        return (make_float3(1, 0, 0));
    }
}
CUDA_HOST_DEVICE SymMat33f NormalEquationsMatrix(const Mat33f& A) {
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

CUDA_HOST_DEVICE float3 operator*(const Mat32f& M, const float2& v) {
    float3 result;
    result.x = M[0] * v.x + M[3] * v.y;
    result.y = M[1] * v.x + M[4] * v.y;
    result.z = M[2] * v.x + M[5] * v.y;

    return result;
}
CUDA_HOST_DEVICE Mat32f operator*(const SymMat33f& M, const Mat32f& N) {
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
CUDA_HOST_DEVICE SymMat22f operator-(const SymMat22f& M, const float& v) {
    return SymMat22f(M.x11 - v, M.x21, M.x22 - v);  //
}
CUDA_HOST_DEVICE SymMat22f CofactorMatrix(const SymMat22f& A) {
    SymMat22f T;
    T.x11 = A.x22;   //
    T.x21 = -A.x21;  //
    T.x22 = A.x11;   //
    return T;
}
CUDA_HOST_DEVICE float2 LargestColumnNormalized(const SymMat22f& A) {
    float scale1 = Length2(make_float2(A.x11, A.x21));
    float scale2 = Length2(make_float2(A.x21, A.x22));
    if (scale1 > scale2) {
        return make_float2(A.x11, A.x21) / sqrtf(scale1);
    } else if (scale2 > 0) {
        return make_float2(A.x21, A.x22) / sqrtf(scale2);
    } else {
        return make_float2(1, 0);
    }
}

// A^T*B
CUDA_HOST_DEVICE SymMat22f TransposeTimesWithSymmetricResult(const Mat32f& A, const Mat32f& B) {
    SymMat22f T;
    T.x11 = A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
    T.x21 = A[3] * B[0] + A[4] * B[1] + A[5] * B[2];
    T.x22 = A[3] * B[3] + A[4] * B[4] + A[5] * B[5];

    return T;
}

CUDA_HOST_DEVICE SymMat22f ConjugateWithTranspose(const Mat32f& A, const SymMat33f& B) {
    return TransposeTimesWithSymmetricResult(B * A, A);
}

CUDA_HOST_DEVICE void Print(const Mat33f& A, const char* name) {
    printf("%s\n", name);
    printf("%f %f %f\n", A[0], A[3], A[6]);
    printf("%f %f %f\n", A[1], A[4], A[7]);
    printf("%f %f %f\n", A[2], A[5], A[8]);
}
CUDA_HOST_DEVICE void Print(const Mat32f& A, const char* name) {
    printf("%s\n", name);
    printf("%f %f\n", A[0], A[3]);
    printf("%f %f\n", A[1], A[4]);
    printf("%f %f\n", A[2], A[5]);
}
CUDA_HOST_DEVICE void Print(const SymMat33f& A, const char* name) {
    printf("%s\n", name);

    printf("%f %f %f\n", A.x11, A.x21, A.x31);
    printf("%f %f %f\n", A.x21, A.x22, A.x32);
    printf("%f %f %f\n", A.x31, A.x32, A.x33);
}
CUDA_HOST_DEVICE void Print(const SymMat22f& A, const char* name) {
    printf("%s\n", name);

    printf("%f %f\n", A.x11, A.x21);
    printf("%f %f\n", A.x21, A.x22);
}

CUDA_HOST_DEVICE void PrintLine(const Mat33f& A, const char* name) {
    printf("%s: [%f,%f,%f,%f,%f,%f,%f,%f,%f]\n", name, A[0], A[1], A[2], A[3], A[4], A[5], A[6], A[7], A[8]);
}
CUDA_HOST_DEVICE void PrintLine(const Mat32f& A, const char* name) {
    printf("%s: [%f,%f,%f,%f,%f,%f]\n", name, A[0], A[1], A[2], A[3], A[4], A[5]);
}
CUDA_HOST_DEVICE void PrintLine(const SymMat33f& A, const char* name) {
    printf("%s: [%f,%f,%f,%f,%f,%f,%f,%f,%f]\n", name, A.x11, A.x21, A.x31, A.x21, A.x22, A.x32, A.x31, A.x32, A.x33);
}
CUDA_HOST_DEVICE void PrintLine(const SymMat22f& A, const char* name) {
    printf("%s: [%f,%f,%f,%f]\n", name, A.x11, A.x21, A.x21, A.x22);
}
}
