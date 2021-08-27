#if defined(__CUDA_ARCH__)
#include "chrono_multicore/math/simd_non.h"
#include "chrono_multicore/math/matrix.h"
#include "chrono_multicore/math/real3.h"

#include <iostream>

namespace chrono {

// dot product of each column of a matrix with itself
CUDA_HOST_DEVICE inline real3 DotMM(const real* M) {
    real3 result;
    result.x = M[0] * M[0] + M[1] * M[1] + M[2] * M[2];
    result.y = M[4] * M[4] + M[5] * M[5] + M[6] * M[6];
    result.z = M[8] * M[8] + M[9] * M[9] + M[10] * M[10];
    return result;
}  // dot product of each column of a matrix with another matrix
CUDA_HOST_DEVICE inline real3 DotMM(const real* M, const real* N) {
    real3 result;
    result.x = M[0] * N[0] + M[1] * N[1] + M[2] * N[2];
    result.y = M[4] * N[4] + M[5] * N[5] + M[6] * N[6];
    result.z = M[8] * N[8] + M[9] * N[9] + M[10] * N[10];
    return result;
}
CUDA_HOST_DEVICE inline Mat33 MulMM(const real* M, const real* N) {
    Mat33 r;
    r[0] = M[0] * N[0] + M[4] * N[1] + M[8] * N[2];
    r[1] = M[1] * N[0] + M[5] * N[1] + M[9] * N[2];
    r[2] = M[2] * N[0] + M[6] * N[1] + M[10] * N[2];

    r[4] = M[0] * N[4] + M[4] * N[5] + M[8] * N[6];
    r[5] = M[1] * N[4] + M[5] * N[5] + M[9] * N[6];
    r[6] = M[2] * N[4] + M[6] * N[5] + M[10] * N[6];

    r[8] = M[0] * N[8] + M[4] * N[9] + M[8] * N[10];
    r[9] = M[1] * N[8] + M[5] * N[9] + M[9] * N[10];
    r[10] = M[2] * N[8] + M[6] * N[9] + M[10] * N[10];
    return r;
}

CUDA_HOST_DEVICE inline Mat33 MulM_TM(const real* M, const real* N) {
    // c1 c2 c3    // c1 c2 c3
    // 0  1  2     // 0  4  8
    // 4  5  6     // 1  5  9
    // 8  9  10    // 2  6  10

    Mat33 r;
    r[0] = M[0] * N[0] + M[1] * N[1] + M[2] * N[2];
    r[1] = M[4] * N[0] + M[5] * N[1] + M[6] * N[2];
    r[2] = M[8] * N[0] + M[9] * N[1] + M[10] * N[2];

    r[4] = M[0] * N[4] + M[1] * N[5] + M[2] * N[6];
    r[5] = M[4] * N[4] + M[5] * N[5] + M[6] * N[6];
    r[6] = M[8] * N[4] + M[9] * N[5] + M[10] * N[6];

    r[8] = M[0] * N[8] + M[1] * N[9] + M[2] * N[10];
    r[9] = M[4] * N[8] + M[5] * N[9] + M[6] * N[10];
    r[10] = M[8] * N[8] + M[9] * N[9] + M[10] * N[10];
    return r;
}

CUDA_HOST_DEVICE inline real3 MulMV(const real* M, const real* N) {
    real3 r;
    r[0] = M[0] * N[0] + M[4] * N[1] + M[8] * N[2];
    r[1] = M[1] * N[0] + M[5] * N[1] + M[9] * N[2];
    r[2] = M[2] * N[0] + M[6] * N[1] + M[10] * N[2];

    return r;
}

CUDA_HOST_DEVICE inline Mat33 OuterProductVV(const real* A, const real* B) {
    return Mat33(A[0] * B[0], A[1] * B[0], A[2] * B[0], A[0] * B[1], A[1] * B[1], A[2] * B[1], A[0] * B[2], A[1] * B[2],
                 A[2] * B[2]);
}

CUDA_HOST_DEVICE inline Mat33 ScaleMat(const real* M, const real b) {
    Mat33 r;
    r[0] = M[0] * b;
    r[1] = M[1] * b;
    r[2] = M[2] * b;
    r[4] = M[4] * b;
    r[5] = M[5] * b;
    r[6] = M[6] * b;
    r[8] = M[8] * b;
    r[9] = M[9] * b;
    r[10] = M[10] * b;
    return r;
}

CUDA_HOST_DEVICE inline SymMat33 NormalEquations(const real* A) {
    SymMat33 T;

    T.x11 = A[0] * A[0] + A[1] * A[1] + A[2] * A[2];
    T.x21 = A[0] * A[4] + A[1] * A[5] + A[2] * A[6];
    T.x31 = A[0] * A[8] + A[1] * A[9] + A[2] * A[10];
    T.x22 = A[4] * A[4] + A[5] * A[5] + A[6] * A[6];
    T.x32 = A[4] * A[8] + A[5] * A[9] + A[6] * A[10];
    T.x33 = A[8] * A[8] + A[9] * A[9] + A[10] * A[10];

    return T;
}

CUDA_HOST_DEVICE inline Mat33 MAbs(const real* M) {
    return Mat33(Abs(M[0]), Abs(M[1]), Abs(M[2]), Abs(M[4]), Abs(M[5]), Abs(M[6]), Abs(M[8]), Abs(M[9]), Abs(M[10]));
}

//[0,4,8 ]
//[1,5,9 ]
//[2,6,10]
//[3,7,11]
CUDA_HOST_DEVICE real3 operator*(const Mat33& M, const real3& v) {
    return MulMV(M.array, v.array);
}

CUDA_HOST_DEVICE Mat33 operator*(const Mat33& N, const real scale) {
    return ScaleMat(N.array, scale);
}

CUDA_HOST_DEVICE Mat33 operator*(const Mat33& M, const Mat33& N) {
    return MulMM(M.array, N.array);
}
CUDA_HOST_DEVICE Mat33 operator+(const Mat33& M, const Mat33& N) {
    return Mat33(M[0] + N[0], M[1] + N[1], M[2] + N[2], M[4] + N[4], M[5] + N[5], M[6] + N[6], M[8] + N[8], M[9] + N[9],
                 M[10] + N[10]);
}

CUDA_HOST_DEVICE Mat33 operator-(const Mat33& M, const Mat33& N) {
    return Mat33(M[0] - N[0], M[1] - N[1], M[2] - N[2], M[4] - N[4], M[5] - N[5], M[6] - N[6], M[8] - N[8], M[9] - N[9],
                 M[10] - N[10]);
}
CUDA_HOST_DEVICE Mat33 operator-(const Mat33& M) {
    return Mat33(-M[0], -M[1], -M[2], -M[4], -M[5], -M[6], -M[8], -M[9], -M[10]);
}
CUDA_HOST_DEVICE Mat33 Abs(const Mat33& m) {
    return MAbs(m.array);
}
CUDA_HOST_DEVICE Mat33 SkewSymmetric(const real3& r) {
    return Mat33(0, r[2], -r[1], -r[2], 0, r[0], r[1], -r[0], 0);
}
CUDA_HOST_DEVICE Mat33 SkewSymmetricAlt(const real3& r) {
    return Mat33(0, r[2], r[1], r[2], 0, r[0], r[1], r[0], 0);
}
CUDA_HOST_DEVICE Mat33 MultTranspose(const Mat33& M, const Mat33& N) {
    // Not a clean way to write this in AVX, might as well transpose first and then multiply
    return M * Transpose(N);
}

CUDA_HOST_DEVICE Mat33 TransposeMult(const Mat33& M, const Mat33& N) {
    return MulM_TM(M.array, N.array);
}

CUDA_HOST_DEVICE Mat33 Transpose(const Mat33& a) {
    return Mat33(a[0], a[4], a[8], a[1], a[5], a[9], a[2], a[6], a[10]);
}

CUDA_HOST_DEVICE real Trace(const Mat33& m) {
    return m[0] + m[5] + m[10];
}
// Multiply a 3x1 by a 1x3 to get a 3x3
CUDA_HOST_DEVICE Mat33 OuterProduct(const real3& a, const real3& b) {
    return OuterProductVV(a.array, b.array);
}

CUDA_HOST_DEVICE real InnerProduct(const Mat33& A, const Mat33& B) {
    return simd::HorizontalAdd(DotMM(A.array, B.array));
}

CUDA_HOST_DEVICE Mat33 Adjoint(const Mat33& A) {
    Mat33 T;
    T[0] = A[5] * A[10] - A[9] * A[6];
    T[1] = -A[1] * A[10] + A[9] * A[2];
    T[2] = A[1] * A[6] - A[5] * A[2];

    T[4] = -A[4] * A[10] + A[8] * A[6];
    T[5] = A[0] * A[10] - A[8] * A[2];
    T[6] = -A[0] * A[6] + A[4] * A[2];

    T[8] = A[4] * A[9] - A[8] * A[5];
    T[9] = -A[0] * A[9] + A[8] * A[1];
    T[10] = A[0] * A[5] - A[4] * A[1];
    return T;
}

CUDA_HOST_DEVICE Mat33 AdjointTranspose(const Mat33& A) {
    Mat33 T;
    T[0] = A[5] * A[10] - A[9] * A[6];
    T[1] = -A[4] * A[10] + A[8] * A[6];
    T[2] = A[4] * A[9] - A[8] * A[5];

    T[4] = -A[1] * A[10] + A[9] * A[2];
    T[5] = A[0] * A[10] - A[8] * A[2];
    T[6] = -A[0] * A[9] + A[8] * A[1];

    T[8] = A[1] * A[6] - A[5] * A[2];
    T[9] = -A[0] * A[6] + A[4] * A[2];
    T[10] = A[0] * A[5] - A[4] * A[1];

    return T;
}

CUDA_HOST_DEVICE real Determinant(const Mat33& m) {
    return m[0] * (m[5] * m[10] - m[9] * m[6]) - m[4] * (m[1] * m[10] - m[9] * m[2]) +
           m[8] * (m[1] * m[6] - m[5] * m[2]);
}

CUDA_HOST_DEVICE Mat33 Inverse(const Mat33& A) {
    real s = Determinant(A);
    if (s > 0.0) {
        return Adjoint(A) * real(1.0 / s);
    } else {
        return Mat33(0);
    }
}

// Same as inverse but we store it transposed
CUDA_HOST_DEVICE Mat33 InverseTranspose(const Mat33& A) {
    real s = Determinant(A);
    if (s > 0.0) {
        return AdjointTranspose(A) * real(1.0 / s);
    } else {
        return Mat33(0);
    }
}

CUDA_HOST_DEVICE real Norm(const Mat33& A) {
    return Sqrt(Trace(A * Transpose(A)));
}
CUDA_HOST_DEVICE real NormSq(const Mat33& A) {
    return Trace(A * Transpose(A));
}
CUDA_HOST_DEVICE real DoubleDot(const Mat33& A, const Mat33& B) {
    return A[0] * B[0] + A[1] * B[1] + A[2] * B[2] + A[4] * B[4] + A[5] * B[5] + A[6] * B[6] + A[8] * B[8] +
           A[9] * B[9] + A[10] * B[10];
}

CUDA_HOST_DEVICE real3 LargestColumnNormalized(const Mat33& A) {
    real3 scale = DotMM(A.array);
    real3 sqrt_scale = simd::SquareRoot(scale);
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

CUDA_HOST_DEVICE Mat33 operator*(const DiagMat33& M, const Mat33& N) {
    return Mat33(M.x11 * N[0], M.x22 * N[1], M.x33 * N[2], M.x11 * N[4], M.x22 * N[5], M.x33 * N[6], M.x11 * N[8],
                 M.x22 * N[9], M.x33 * N[10]);
}
CUDA_HOST_DEVICE real3 operator*(const DiagMat33& M, const real3& v) {
    real3 result;
    result.x = M.x11 * v.x;
    result.y = M.x22 * v.y;
    result.z = M.x33 * v.z;
    return result;
}
//// ========================================================================================
CUDA_HOST_DEVICE SymMat33 operator-(const SymMat33& M, const real& v) {
    return SymMat33(M.x11 - v, M.x21, M.x31, M.x22 - v, M.x32, M.x33 - v);  // only subtract diagonal
}
CUDA_HOST_DEVICE SymMat33 CofactorMatrix(const SymMat33& A) {
    SymMat33 T;
    T.x11 = A.x22 * A.x33 - A.x32 * A.x32;   //
    T.x21 = -A.x21 * A.x33 + A.x32 * A.x31;  //
    T.x22 = A.x11 * A.x33 - A.x31 * A.x31;   //
    T.x31 = A.x21 * A.x32 - A.x22 * A.x31;   //
    T.x32 = -A.x11 * A.x32 + A.x21 * A.x31;  //
    T.x33 = A.x11 * A.x22 - A.x21 * A.x21;   //
    return T;
}
CUDA_HOST_DEVICE real3 LargestColumnNormalized(const SymMat33& A) {
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
CUDA_HOST_DEVICE SymMat33 NormalEquationsMatrix(const Mat33& A) {
    return NormalEquations(A.array);
}
//// ========================================================================================

CUDA_HOST_DEVICE real3 operator*(const Mat32& M, const real2& v) {
    real3 result;
    result.x = M[0] * v.x + M[4] * v.y;
    result.y = M[1] * v.x + M[5] * v.y;
    result.z = M[2] * v.x + M[6] * v.y;

    return result;
}
CUDA_HOST_DEVICE Mat32 operator*(const SymMat33& M, const Mat32& N) {
    Mat32 result;
    // x11 x21 x31  c11 c12
    // x21 x22 x32  c21 c22
    // x31 x32 x33  c31 c32
    result[0] = M.x11 * N[0] + M.x21 * N[1] + M.x31 * N[2];
    result[1] = M.x21 * N[0] + M.x22 * N[1] + M.x32 * N[2];
    result[2] = M.x31 * N[0] + M.x32 * N[1] + M.x33 * N[2];

    result[4] = M.x11 * N[4] + M.x21 * N[5] + M.x31 * N[6];
    result[5] = M.x21 * N[4] + M.x22 * N[5] + M.x32 * N[6];
    result[6] = M.x31 * N[4] + M.x32 * N[5] + M.x33 * N[6];

    return result;
}
//// ========================================================================================
CUDA_HOST_DEVICE SymMat22 operator-(const SymMat22& M, const real& v) {
    return SymMat22(M.x11 - v, M.x21, M.x22 - v);  //
}
CUDA_HOST_DEVICE SymMat22 CofactorMatrix(const SymMat22& A) {
    SymMat22 T;
    T.x11 = A.x22;   //
    T.x21 = -A.x21;  //
    T.x22 = A.x11;   //
    return T;
}
CUDA_HOST_DEVICE real2 LargestColumnNormalized(const SymMat22& A) {
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

// A^T*B
CUDA_HOST_DEVICE SymMat22 TransposeTimesWithSymmetricResult(const Mat32& A, const Mat32& B) {
    SymMat22 T;
    T.x11 = A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
    T.x21 = A[4] * B[0] + A[5] * B[1] + A[6] * B[2];
    T.x22 = A[4] * B[4] + A[5] * B[5] + A[6] * B[6];

    return T;
}

CUDA_HOST_DEVICE SymMat22 ConjugateWithTranspose(const Mat32& A, const SymMat33& B) {
    return TransposeTimesWithSymmetricResult(B * A, A);
}

CUDA_HOST_DEVICE void Print(const Mat33& A, const char* name) {
    printf("%s\n", name);
    printf("%f %f %f\n", A[0], A[4], A[8]);
    printf("%f %f %f\n", A[1], A[5], A[9]);
    printf("%f %f %f\n", A[2], A[6], A[10]);
}
CUDA_HOST_DEVICE void Print(const Mat32& A, const char* name) {
    printf("%s\n", name);
    printf("%f %f\n", A[0], A[4]);
    printf("%f %f\n", A[1], A[5]);
    printf("%f %f\n", A[2], A[6]);
}
CUDA_HOST_DEVICE void Print(const SymMat33& A, const char* name) {
    printf("%s\n", name);

    printf("%f %f %f\n", A.x11, A.x21, A.x31);
    printf("%f %f %f\n", A.x21, A.x22, A.x32);
    printf("%f %f %f\n", A.x31, A.x32, A.x33);
}
CUDA_HOST_DEVICE void Print(const SymMat22& A, const char* name) {
    printf("%s\n", name);

    printf("%f %f\n", A.x11, A.x21);
    printf("%f %f\n", A.x21, A.x22);
}

CUDA_HOST_DEVICE void PrintLine(const Mat33& A, const char* name) {
    printf("%s: [%f,%f,%f,%f,%f,%f,%f,%f,%f]\n", name, A[0], A[1], A[2], A[4], A[5], A[6], A[8], A[9], A[10]);
}
CUDA_HOST_DEVICE void PrintLine(const Mat32& A, const char* name) {
    printf("%s: [%f,%f,%f,%f,%f,%f]\n", name, A[0], A[1], A[2], A[4], A[5], A[6]);
}
CUDA_HOST_DEVICE void PrintLine(const SymMat33& A, const char* name) {
    printf("%s: [%f,%f,%f,%f,%f,%f,%f,%f,%f]\n", name, A.x11, A.x21, A.x31, A.x21, A.x22, A.x32, A.x31, A.x32, A.x33);
}
CUDA_HOST_DEVICE void PrintLine(const SymMat22& A, const char* name) {
    printf("%s: [%f,%f,%f,%f]\n", name, A.x11, A.x21, A.x21, A.x22);
}
}
#endif
