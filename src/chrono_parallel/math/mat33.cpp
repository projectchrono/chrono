#include "chrono_parallel/math/mat33.h"
#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/math/simd.h"
namespace chrono {

//[0,4,8 ]
//[1,5,9 ]
//[2,6,10]
//[3,7,11]
real3 operator*(const Mat33& M, const real3& v) {
    return simd::MulMV(M.array, v.array);
}

Mat33 operator*(const Mat33& N, const real scale) {
    return simd::ScaleMat(N.array, scale);
}

Mat33 operator*(const Mat33& M, const Mat33& N) {
    return simd::MulMM(M.array, N.array);
}
Mat33 operator+(const Mat33& M, const Mat33& N) {
    return Mat33(M[0] + N[0], M[1] + N[1], M[2] + N[2], M[4] + N[4], M[5] + N[5], M[6] + N[6], M[8] + N[8], M[9] + N[9],
                 M[10] + N[10]);
}

Mat33 operator-(const Mat33& M, const Mat33& N) {
    return Mat33(M[0] - N[0], M[1] - N[1], M[2] - N[2], M[4] - N[4], M[5] - N[5], M[6] - N[6], M[8] - N[8], M[9] - N[9],
                 M[10] - N[10]);
}
Mat33 operator-(const Mat33& M) {
    return Mat33(-M[0], -M[1], -M[2], -M[4], -M[5], -M[6], -M[8], -M[9], -M[10]);
}
Mat33 Abs(const Mat33& m) {
    return Mat33(simd::Abs(m.cols[0]), simd::Abs(m.cols[1]), simd::Abs(m.cols[2]));
}
Mat33 SkewSymmetric(const real3& r) {
    return Mat33(real3(0, r[2], -r[1]), real3(-r[2], 0, r[0]), real3(r[1], -r[0], 0));
}

Mat33 MultTranspose(const Mat33& M, const Mat33& N) {
    return Mat33(M * real3(N[0], N[4], N[8]),  //
                 M * real3(N[1], N[5], N[9]),  //
                 M * real3(N[2], N[6], N[10]));
}

Mat33 TransposeMult(const Mat33& M, const Mat33& N) {
    return simd::MulM_TM(M.array, N.array);
}

Mat33 Transpose(const Mat33& a) {
    return Mat33(a[0], a[4], a[8], a[1], a[5], a[9], a[2], a[6], a[10]);
}

real Trace(const Mat33& m) {
    return m[0] + m[5] + m[10];
}
// Multiply a 3x1 by a 1x3 to get a 3x3
Mat33 OuterProduct(const real3& a, const real3& b) {
    return simd::OuterProductVV(a.array, b.array);
}

real InnerProduct(const Mat33& A, const Mat33& B) {
    return Dot(A.cols[0], B.cols[0]) + Dot(A.cols[1], B.cols[1]) + Dot(A.cols[0], B.cols[0]);
}

Mat33 Adjoint(const Mat33& A) {
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

Mat33 AdjointTranspose(const Mat33& A) {
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

real Determinant(const Mat33& m) {
    real d;
    d = m[0] * (m[5] * m[10] - m[9] * m[6]);
    d -= m[4] * (m[1] * m[10] - m[9] * m[2]);
    d += m[8] * (m[1] * m[6] - m[5] * m[2]);
    return d;  // Dot(m.cols[0], Cross(m.cols[1], m.cols[2]));
}

Mat33 Inverse(const Mat33& A) {
    real s = Determinant(A);
    // if (s > 0.0) {
    return Adjoint(A) * real(1.0 / s);
    //} else {
    //    return Mat33(0);
    //}
}

// Same as inverse but we store it transposed
Mat33 InverseTranspose(const Mat33& A) {
    real s = Determinant(A);
    if (s > 0.0) {
        return AdjointTranspose(A) * real(1.0 / s);
    } else {
        return Mat33(0);
    }
}

real Norm(const Mat33& A) {
    return Sqrt(Trace(A * Transpose(A)));
}

real3 LargestColumnNormalized(const Mat33& A) {
    real scale1 = Length2((A.cols[0]));
    real scale2 = Length2((A.cols[1]));
    real scale3 = Length2((A.cols[2]));
    if (scale1 > scale2) {
        if (scale1 > scale3) {
            return A.cols[0] / Sqrt(scale1);
        }
    } else if (scale2 > scale3) {
        return A.cols[1] / Sqrt(scale2);
    }
    return A.cols[2] / Sqrt(scale3);
}
//// ========================================================================================

Mat33 operator*(const DiagMat33& M, const Mat33& N) {
    return Mat33(real3(M.x11 * N.cols[0].x, M.x22 * N.cols[0].y, M.x33 * N.cols[0].z),
                 real3(M.x11 * N.cols[1].x, M.x22 * N.cols[1].y, M.x33 * N.cols[1].z),
                 real3(M.x11 * N.cols[2].x, M.x22 * N.cols[2].y, M.x33 * N.cols[2].z));
}
real3 operator*(const DiagMat33& M, const real3& v) {
    real3 result;
    result.x = M.x11 * v.x;
    result.y = M.x22 * v.y;
    result.z = M.x33 * v.z;
    return result;
}
//// ========================================================================================
SymMat33 operator-(const SymMat33& M, const real& v) {
    return SymMat33(M.x11 - v, M.x21, M.x31, M.x22 - v, M.x32, M.x33 - v);  // only subtract diagonal
}
SymMat33 CofactorMatrix(const SymMat33& A) {
    SymMat33 T;
    T.x11 = A.x22 * A.x33 - A.x32 * A.x32;   //
    T.x21 = -A.x21 * A.x33 + A.x32 * A.x31;  //
    T.x22 = A.x11 * A.x33 - A.x31 * A.x31;   //
    T.x31 = A.x21 * A.x32 - A.x22 * A.x31;   //
    T.x32 = -A.x11 * A.x32 + A.x21 * A.x31;  //
    T.x33 = A.x11 * A.x22 - A.x21 * A.x21;   //
    return T;
}
real3 LargestColumnNormalized(const SymMat33& A) {
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
//// ========================================================================================

real3 operator*(const Mat32& M, const real2& v) {
    real3 result;
    result.x = M[0] * v.x + M[4] * v.y;
    result.y = M[1] * v.x + M[5] * v.y;
    result.z = M[2] * v.x + M[6] * v.y;

    return result;
}
Mat32 operator*(const SymMat33& M, const Mat32& N) {
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
SymMat22 operator-(const SymMat22& M, const real& v) {
    return SymMat22(M.x11 - v, M.x21, M.x22 - v);  //
}
SymMat22 CofactorMatrix(const SymMat22& A) {
    SymMat22 T;
    T.x11 = A.x22;   //
    T.x21 = -A.x21;  //
    T.x22 = A.x11;   //
    return T;
}
real2 LargestColumnNormalized(const SymMat22& A) {
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
SymMat22 TransposeTimesWithSymmetricResult(const Mat32& A, const Mat32& B) {
    SymMat22 T;
    T.x11 = Dot(real3(A[0], A[1], A[2]), real3(B[0], A[1], A[2]));
    T.x21 = Dot(real3(A[4], A[5], A[6]), real3(B[0], A[1], A[2]));
    T.x22 = Dot(real3(A[4], A[5], A[6]), real3(B[4], A[5], A[6]));
    return T;
}

SymMat22 ConjugateWithTranspose(const Mat32& A, const SymMat33& B) {
    return TransposeTimesWithSymmetricResult(B * A, A);
}
}
