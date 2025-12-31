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

#include "chrono/multicore_math/simd.h"
#include "chrono/multicore_math/matrix.h"
#include "chrono/multicore_math/real3.h"

#if defined(USE_SSE)
    #include "chrono/multicore_math/simd_sse.h"
#elif defined(USE_AVX)
    #include "chrono/multicore_math/simd_avx.h"
#else
    #include "chrono/multicore_math/simd_non.h"
#endif

#include <iostream>

namespace chrono {

#if defined(USE_AVX)

// dot product of each column of a matrix with itself
inline __m256d DotMM(const real* M) {
    __m256d a = _mm256_loadu_pd(&M[0]);  // Load first column of M
    __m256d b = _mm256_loadu_pd(&M[4]);  // Load second column of M
    __m256d c = _mm256_loadu_pd(&M[8]);  // Load third column of M

    __m256d xy0 = _mm256_mul_pd(a, a);
    __m256d xy1 = _mm256_mul_pd(b, b);
    __m256d xy2 = _mm256_mul_pd(c, c);
    __m256d xy3 = _mm256_set1_pd(0);  // last dot prod is a dud

    // low to high: xy00+xy01 xy10+xy11 xy02+xy03 xy12+xy13
    __m256d temp01 = _mm256_hadd_pd(xy0, xy1);

    // low to high: xy20+xy21 xy30+xy31 xy22+xy23 xy32+xy33
    __m256d temp23 = _mm256_hadd_pd(xy2, xy3);

    // low to high: xy02+xy03 xy12+xy13 xy20+xy21 xy30+xy31
    __m256d swapped = _mm256_permute2f128_pd(temp01, temp23, 0x21);

    // low to high: xy00+xy01 xy10+xy11 xy22+xy23 xy32+xy33
    __m256d blended = _mm256_blend_pd(temp01, temp23, 0xC);

    __m256d dotproduct = _mm256_add_pd(swapped, blended);
    return dotproduct;
}  // dot product of each column of a matrix with itself
inline __m256d DotMM(const real* M, const real* N) {
    __m256d a = _mm256_loadu_pd(&M[0]);  // Load first column of M
    __m256d b = _mm256_loadu_pd(&M[4]);  // Load second column of M
    __m256d c = _mm256_loadu_pd(&M[8]);  // Load third column of M

    __m256d x = _mm256_loadu_pd(&N[0]);  // Load first column of M
    __m256d y = _mm256_loadu_pd(&N[4]);  // Load second column of M
    __m256d z = _mm256_loadu_pd(&N[8]);  // Load third column of M

    __m256d xy0 = _mm256_mul_pd(a, x);
    __m256d xy1 = _mm256_mul_pd(b, y);
    __m256d xy2 = _mm256_mul_pd(c, z);
    __m256d xy3 = _mm256_set1_pd(0);  // last dot prod is a dud

    // low to high: xy00+xy01 xy10+xy11 xy02+xy03 xy12+xy13
    __m256d temp01 = _mm256_hadd_pd(xy0, xy1);

    // low to high: xy20+xy21 xy30+xy31 xy22+xy23 xy32+xy33
    __m256d temp23 = _mm256_hadd_pd(xy2, xy3);

    // low to high: xy02+xy03 xy12+xy13 xy20+xy21 xy30+xy31
    __m256d swapped = _mm256_permute2f128_pd(temp01, temp23, 0x21);

    // low to high: xy00+xy01 xy10+xy11 xy22+xy23 xy32+xy33
    __m256d blended = _mm256_blend_pd(temp01, temp23, 0xC);

    __m256d dotproduct = _mm256_add_pd(swapped, blended);
    return dotproduct;
}
// http://fhtr.blogspot.com/2010/02/4x4-float-matrix-multiplication-using.html
inline Mat33 MulMM(const real* M, const real* N) {
    Mat33 r;
    __m256d r_line;
    int i;
    __m256d a = _mm256_loadu_pd(&M[0]);  // Load first column of M
    __m256d b = _mm256_loadu_pd(&M[4]);  // Load second column of M
    __m256d c = _mm256_loadu_pd(&M[8]);  // Load third column of M

    for (i = 0; i < 3; i++) {
        r_line = _mm256_mul_pd(a, _mm256_set1_pd(N[i * 4 + 0]));
        r_line = _mm256_add_pd(_mm256_mul_pd(b, _mm256_set1_pd(N[i * 4 + 1])), r_line);
        r_line = _mm256_add_pd(_mm256_mul_pd(c, _mm256_set1_pd(N[i * 4 + 2])), r_line);
        _mm256_storeu_pd(&r.array[i * 4], r_line);
    }
    return r;
}

inline Mat33 MulM_TM(const real* M, const real* N) {
    Mat33 result;
    __m256d a = _mm256_loadu_pd(&M[0]);  // Load first column of M
    __m256d b = _mm256_loadu_pd(&M[4]);  // Load second column of M
    __m256d c = _mm256_loadu_pd(&M[8]);  // Load third column of M

    for (int i = 0; i < 3; i++) {
        __m256d v = _mm256_loadu_pd(&N[i * 4]);  // load ith column column of N
        __m256d xy0 = _mm256_mul_pd(v, a);
        __m256d xy1 = _mm256_mul_pd(v, b);
        __m256d xy2 = _mm256_mul_pd(v, c);
        __m256d xy3 = _mm256_set1_pd(0);  // last dot prod is not used

        __m256d temp01 = _mm256_hadd_pd(xy0, xy1);  // low to high: xy00+xy01 xy10+xy11 xy02+xy03 xy12+xy13
        __m256d temp23 = _mm256_hadd_pd(xy2, xy3);  // low to high: xy20+xy21 xy30+xy31 xy22+xy23 xy32+xy33
        // low to high: xy02+xy03 xy12+xy13 xy20+xy21 xy30+xy31
        __m256d swapped = _mm256_permute2f128_pd(temp01, temp23, 0x21);
        // low to high: xy00+xy01 xy10+xy11 xy22+xy23 xy32+xy33
        __m256d blended = _mm256_blend_pd(temp01, temp23, 0xC);
        __m256d dotproduct = _mm256_add_pd(swapped, blended);
        _mm256_storeu_pd(&result.array[i * 4], dotproduct);
    }
    return result;
}

inline real3 MulMV(const real* a, const real* b) {
    real3 r;
    __m256d v1 = _mm256_mul_pd(_mm256_loadu_pd(&a[0]), _mm256_set1_pd(b[0]));
    __m256d v2 = _mm256_mul_pd(_mm256_loadu_pd(&a[4]), _mm256_set1_pd(b[1]));
    __m256d v3 = _mm256_mul_pd(_mm256_loadu_pd(&a[8]), _mm256_set1_pd(b[2]));
    __m256d out = _mm256_add_pd(_mm256_add_pd(v1, v2), v3);
    _mm256_storeu_pd(&r.array[0], out);

    return r;
}

inline Mat33 OuterProductVV(const real* a, const real* b) {
    Mat33 r;
    __m256d u = _mm256_loadu_pd(a);  // Load the first vector
    __m256d col;
    int i;
    for (i = 0; i < 3; i++) {
        col = _mm256_mul_pd(u, _mm256_set1_pd(b[i]));
        _mm256_storeu_pd(&r.array[i * 4], col);
    }
    return r;
}

inline Mat33 ScaleMat(const real* a, const real b) {
    Mat33 r;
    __m256d s = _mm256_set1_pd(b);
    __m256d c, col;
    int i;
    for (i = 0; i < 3; i++) {
        c = _mm256_loadu_pd(&a[i * 4]);
        col = _mm256_mul_pd(c, s);
        _mm256_storeu_pd(&r.array[i * 4], col);
    }
    return r;
}

inline SymMat33 NormalEquations(const real* M) {
    SymMat33 result;
    __m256d a = _mm256_loadu_pd(&M[0]);  // Load first column of M
    __m256d b = _mm256_loadu_pd(&M[4]);  // Load second column of M
    __m256d c = _mm256_loadu_pd(&M[8]);  // Load third column of M

    __m256d xy0 = _mm256_mul_pd(a, a);
    __m256d xy1 = _mm256_mul_pd(a, b);
    __m256d xy2 = _mm256_mul_pd(a, c);
    __m256d xy3 = _mm256_mul_pd(b, b);

    __m256d xy4 = _mm256_mul_pd(b, c);
    __m256d xy5 = _mm256_mul_pd(c, c);
    __m256d xy6 = _mm256_set1_pd(0);
    __m256d xy7 = _mm256_set1_pd(0);

    __m256d temp01 = _mm256_hadd_pd(xy0, xy1);
    __m256d temp23 = _mm256_hadd_pd(xy2, xy3);
    __m256d swapped = _mm256_permute2f128_pd(temp01, temp23, 0x21);
    __m256d blended = _mm256_blend_pd(temp01, temp23, 0xC);
    __m256d dotproduct = _mm256_add_pd(swapped, blended);

    _mm256_storeu_pd(&result.array[0], dotproduct);

    temp01 = _mm256_hadd_pd(xy4, xy5);
    temp23 = _mm256_hadd_pd(xy6, xy7);  // This should be zero
    swapped = _mm256_permute2f128_pd(temp01, temp23, 0x21);
    blended = _mm256_blend_pd(temp01, temp23, 0xC);
    dotproduct = _mm256_add_pd(swapped, blended);
    _mm256_storeu_pd(&result.array[4], dotproduct);
    return result;
}

inline Mat33 MAbs(const real* M) {
    Mat33 result;
    __m256d a = _mm256_loadu_pd(&M[0]);  // Load first column of M
    __m256d b = _mm256_loadu_pd(&M[4]);  // Load second column of M
    __m256d c = _mm256_loadu_pd(&M[8]);  // Load third column of M
    a = _mm256_and_pd(a, simd::ABSMASK);
    b = _mm256_and_pd(b, simd::ABSMASK);
    c = _mm256_and_pd(c, simd::ABSMASK);
    _mm256_storeu_pd(&result.array[0 * 4], a);
    _mm256_storeu_pd(&result.array[1 * 4], b);
    _mm256_storeu_pd(&result.array[2 * 4], c);
    return result;
}

#elif defined(USE_SSE)
inline real3 DotMM(const real* M) {
    real3 result;
    result.x = M[0] * M[0] + M[1] * M[1] + M[2] * M[2];
    result.y = M[4] * M[4] + M[5] * M[5] + M[6] * M[6];
    result.z = M[8] * M[8] + M[9] * M[9] + M[10] * M[10];
    return result;
}  // dot product of each column of a matrix with another matrix
inline real3 DotMM(const real* M, const real* N) {
    real3 result;
    result.x = M[0] * N[0] + M[1] * N[1] + M[2] * N[2];
    result.y = M[4] * N[4] + M[5] * N[5] + M[6] * N[6];
    result.z = M[8] * N[8] + M[9] * N[9] + M[10] * N[10];
    return result;
}
// http://fhtr.blogspot.com/2010/02/4x4-float-matrix-multiplication-using.html
inline Mat33 MulMM(const real* M, const real* N) {
    Mat33 r;
    __m128 r_line;
    int i;
    __m128 a = _mm_loadu_ps(&M[0]);  // Load first column of M
    __m128 b = _mm_loadu_ps(&M[4]);  // Load second column of M
    __m128 c = _mm_loadu_ps(&M[8]);  // Load third column of M

    for (i = 0; i < 3; i++) {
        r_line = _mm_mul_ps(a, _mm_set1_ps(N[i * 4 + 0]));
        r_line = _mm_add_ps(_mm_mul_ps(b, _mm_set1_ps(N[i * 4 + 1])), r_line);
        r_line = _mm_add_ps(_mm_mul_ps(c, _mm_set1_ps(N[i * 4 + 2])), r_line);
        _mm_storeu_ps(&r.array[i * 4], r_line);
    }
    return r;
}

inline Mat33 MulM_TM(const real* M, const real* N) {
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

inline real3 MulMV(const real* a, const real* b) {
    real3 r;
    __m128 v1 = _mm_mul_ps(_mm_loadu_ps(&a[0]), _mm_set1_ps(b[0]));
    __m128 v2 = _mm_mul_ps(_mm_loadu_ps(&a[4]), _mm_set1_ps(b[1]));
    __m128 v3 = _mm_mul_ps(_mm_loadu_ps(&a[8]), _mm_set1_ps(b[2]));
    __m128 out = _mm_add_ps(_mm_add_ps(v1, v2), v3);
    _mm_storeu_ps(&r.array[0], out);

    return r;
}

inline Mat33 OuterProductVV(const real* a, const real* b) {
    Mat33 r;
    __m128 u = _mm_loadu_ps(a);  // Load the first vector
    __m128 col;
    int i;
    for (i = 0; i < 3; i++) {
        col = _mm_mul_ps(u, _mm_set1_ps(b[i]));
        _mm_storeu_ps(&r.array[i * 4], col);
    }
    return r;
}

inline Mat33 ScaleMat(const real* a, const real b) {
    Mat33 r;
    __m128 s = _mm_set1_ps(b);
    __m128 c, col;
    int i;
    for (i = 0; i < 3; i++) {
        c = _mm_loadu_ps(&a[i * 4]);
        col = _mm_mul_ps(c, s);
        _mm_storeu_ps(&r.array[i * 4], col);
    }
    return r;
}

inline SymMat33 NormalEquations(const real* A) {
    SymMat33 T;

    T.x11 = A[0] * A[0] + A[1] * A[1] + A[2] * A[2];
    T.x21 = A[0] * A[4] + A[1] * A[5] + A[2] * A[6];
    T.x31 = A[0] * A[8] + A[1] * A[9] + A[2] * A[10];
    T.x22 = A[4] * A[4] + A[5] * A[5] + A[6] * A[6];
    T.x32 = A[4] * A[8] + A[5] * A[9] + A[6] * A[10];
    T.x33 = A[8] * A[8] + A[9] * A[9] + A[10] * A[10];

    return T;
}

inline Mat33 MAbs(const real* M) {
    Mat33 result;
    __m128 a = _mm_loadu_ps(&M[0]);  // Load first column of M
    __m128 b = _mm_loadu_ps(&M[4]);  // Load second column of M
    __m128 c = _mm_loadu_ps(&M[8]);  // Load third column of M
    a = _mm_and_ps(a, simd::ABSMASK);
    b = _mm_and_ps(b, simd::ABSMASK);
    c = _mm_and_ps(c, simd::ABSMASK);
    _mm_storeu_ps(&result.array[0 * 4], a);
    _mm_storeu_ps(&result.array[1 * 4], b);
    _mm_storeu_ps(&result.array[2 * 4], c);
    return result;
}
#else

// dot product of each column of a matrix with itself
ChApi inline real3 DotMM(const real* M) {
    real3 result;
    result.x = M[0] * M[0] + M[1] * M[1] + M[2] * M[2];
    result.y = M[4] * M[4] + M[5] * M[5] + M[6] * M[6];
    result.z = M[8] * M[8] + M[9] * M[9] + M[10] * M[10];
    return result;
}  // dot product of each column of a matrix with another matrix
ChApi inline real3 DotMM(const real* M, const real* N) {
    real3 result;
    result.x = M[0] * N[0] + M[1] * N[1] + M[2] * N[2];
    result.y = M[4] * N[4] + M[5] * N[5] + M[6] * N[6];
    result.z = M[8] * N[8] + M[9] * N[9] + M[10] * N[10];
    return result;
}
ChApi inline Mat33 MulMM(const real* M, const real* N) {
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

ChApi inline Mat33 MulM_TM(const real* M, const real* N) {
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

ChApi inline real3 MulMV(const real* M, const real* N) {
    real3 r;
    r[0] = M[0] * N[0] + M[4] * N[1] + M[8] * N[2];
    r[1] = M[1] * N[0] + M[5] * N[1] + M[9] * N[2];
    r[2] = M[2] * N[0] + M[6] * N[1] + M[10] * N[2];

    return r;
}

ChApi inline Mat33 OuterProductVV(const real* A, const real* B) {
    return Mat33(A[0] * B[0], A[1] * B[0], A[2] * B[0], A[0] * B[1], A[1] * B[1], A[2] * B[1], A[0] * B[2], A[1] * B[2],
                 A[2] * B[2]);
}

ChApi inline Mat33 ScaleMat(const real* M, const real b) {
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

ChApi inline SymMat33 NormalEquations(const real* A) {
    SymMat33 T;

    T.x11 = A[0] * A[0] + A[1] * A[1] + A[2] * A[2];
    T.x21 = A[0] * A[4] + A[1] * A[5] + A[2] * A[6];
    T.x31 = A[0] * A[8] + A[1] * A[9] + A[2] * A[10];
    T.x22 = A[4] * A[4] + A[5] * A[5] + A[6] * A[6];
    T.x32 = A[4] * A[8] + A[5] * A[9] + A[6] * A[10];
    T.x33 = A[8] * A[8] + A[9] * A[9] + A[10] * A[10];

    return T;
}

ChApi inline Mat33 MAbs(const real* M) {
    return Mat33(Abs(M[0]), Abs(M[1]), Abs(M[2]), Abs(M[4]), Abs(M[5]), Abs(M[6]), Abs(M[8]), Abs(M[9]), Abs(M[10]));
}

#endif
//[0,4,8 ]
//[1,5,9 ]
//[2,6,10]
//[3,7,11]
ChApi real3 operator*(const Mat33& M, const real3& v) {
    return MulMV(M.array, v.array);
}

ChApi Mat33 operator*(const Mat33& N, const real scale) {
    return ScaleMat(N.array, scale);
}

ChApi Mat33 operator*(const Mat33& M, const Mat33& N) {
    return MulMM(M.array, N.array);
}
ChApi Mat33 operator+(const Mat33& M, const Mat33& N) {
    return Mat33(M[0] + N[0], M[1] + N[1], M[2] + N[2], M[4] + N[4], M[5] + N[5], M[6] + N[6], M[8] + N[8], M[9] + N[9],
                 M[10] + N[10]);
}

ChApi Mat33 operator-(const Mat33& M, const Mat33& N) {
    return Mat33(M[0] - N[0], M[1] - N[1], M[2] - N[2], M[4] - N[4], M[5] - N[5], M[6] - N[6], M[8] - N[8], M[9] - N[9],
                 M[10] - N[10]);
}

ChApi OPERATOR_EQUALSALT(*, real, Mat33)       //
    ChApi OPERATOR_EQUALSALT(*, Mat33, Mat33)  //
    ChApi OPERATOR_EQUALSALT(+, Mat33, Mat33)  //
    ChApi OPERATOR_EQUALSALT(-, Mat33, Mat33)  //

    ChApi Mat33
    operator-(const Mat33& M) {
    return Mat33(-M[0], -M[1], -M[2], -M[4], -M[5], -M[6], -M[8], -M[9], -M[10]);
}

ChApi Mat33 operator*(const real s, const Mat33& a) {
    return a * s;
}
ChApi Mat33 Abs(const Mat33& m) {
    return MAbs(m.array);
}
ChApi Mat33 SkewSymmetric(const real3& r) {
    return Mat33(0, r[2], -r[1], -r[2], 0, r[0], r[1], -r[0], 0);
}
ChApi Mat33 SkewSymmetricAlt(const real3& r) {
    return Mat33(0, r[2], r[1], r[2], 0, r[0], r[1], r[0], 0);
}
ChApi Mat33 MultTranspose(const Mat33& M, const Mat33& N) {
    // Not a clean way to write this in AVX, might as well transpose first and then multiply
    return M * Transpose(N);
}

ChApi Mat33 TransposeMult(const Mat33& M, const Mat33& N) {
    return MulM_TM(M.array, N.array);
}

ChApi Mat33 Transpose(const Mat33& a) {
    return Mat33(a[0], a[4], a[8], a[1], a[5], a[9], a[2], a[6], a[10]);
}

ChApi real Trace(const Mat33& m) {
    return m[0] + m[5] + m[10];
}
// Multiply a 3x1 by a 1x3 to get a 3x3
ChApi Mat33 OuterProduct(const real3& a, const real3& b) {
    return OuterProductVV(a.array, b.array);
}

ChApi real InnerProduct(const Mat33& A, const Mat33& B) {
    return simd::HorizontalAdd(DotMM(A.array, B.array));
}

ChApi Mat33 Adjoint(const Mat33& A) {
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

ChApi Mat33 AdjointTranspose(const Mat33& A) {
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

ChApi real Determinant(const Mat33& m) {
    return m[0] * (m[5] * m[10] - m[9] * m[6]) - m[4] * (m[1] * m[10] - m[9] * m[2]) +
           m[8] * (m[1] * m[6] - m[5] * m[2]);
}

ChApi Mat33 Inverse(const Mat33& A) {
    real s = Determinant(A);
    if (s > 0.0) {
        return Adjoint(A) * real(1.0 / s);
    } else {
        return Mat33(0);
    }
}

// Same as inverse but we store it transposed
ChApi Mat33 InverseTranspose(const Mat33& A) {
    real s = Determinant(A);
    if (s > 0.0) {
        return AdjointTranspose(A) * real(1.0 / s);
    } else {
        return Mat33(0);
    }
}
ChApi Mat33 InverseUnsafe(const Mat33& A) {
    real s = Determinant(A);
    return Adjoint(A) * real(1.0 / s);
}

// Same as inverse but we store it transposed
ChApi Mat33 InverseTransposeUnsafe(const Mat33& A) {
    real s = Determinant(A);
    return AdjointTranspose(A) * real(1.0 / s);
}
ChApi real Norm(const Mat33& A) {
    return Sqrt(Trace(A * Transpose(A)));
}
ChApi real NormSq(const Mat33& A) {
    return Trace(A * Transpose(A));
}
ChApi real DoubleDot(const Mat33& A, const Mat33& B) {
    return A[0] * B[0] + A[1] * B[1] + A[2] * B[2] + A[4] * B[4] + A[5] * B[5] + A[6] * B[6] + A[8] * B[8] +
           A[9] * B[9] + A[10] * B[10];
}

ChApi real3 LargestColumnNormalized(const Mat33& A) {
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

ChApi Mat33 operator*(const DiagMat33& M, const Mat33& N) {
    return Mat33(M.x11 * N[0], M.x22 * N[1], M.x33 * N[2], M.x11 * N[4], M.x22 * N[5], M.x33 * N[6], M.x11 * N[8],
                 M.x22 * N[9], M.x33 * N[10]);
}
ChApi real3 operator*(const DiagMat33& M, const real3& v) {
    real3 result;
    result.x = M.x11 * v.x;
    result.y = M.x22 * v.y;
    result.z = M.x33 * v.z;
    return result;
}
//// ========================================================================================
ChApi SymMat33 operator-(const SymMat33& M, const real& v) {
    return SymMat33(M.x11 - v, M.x21, M.x31, M.x22 - v, M.x32, M.x33 - v);  // only subtract diagonal
}
ChApi SymMat33 CofactorMatrix(const SymMat33& A) {
    SymMat33 T;
    T.x11 = A.x22 * A.x33 - A.x32 * A.x32;   //
    T.x21 = -A.x21 * A.x33 + A.x32 * A.x31;  //
    T.x22 = A.x11 * A.x33 - A.x31 * A.x31;   //
    T.x31 = A.x21 * A.x32 - A.x22 * A.x31;   //
    T.x32 = -A.x11 * A.x32 + A.x21 * A.x31;  //
    T.x33 = A.x11 * A.x22 - A.x21 * A.x21;   //
    return T;
}
ChApi real3 LargestColumnNormalized(const SymMat33& A) {
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
ChApi SymMat33 NormalEquationsMatrix(const Mat33& A) {
    return NormalEquations(A.array);
}
//// ========================================================================================

ChApi real3 operator*(const Mat32& M, const real2& v) {
    real3 result;
    result.x = M[0] * v.x + M[4] * v.y;
    result.y = M[1] * v.x + M[5] * v.y;
    result.z = M[2] * v.x + M[6] * v.y;

    return result;
}
ChApi Mat32 operator*(const SymMat33& M, const Mat32& N) {
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
ChApi SymMat22 operator-(const SymMat22& M, const real& v) {
    return SymMat22(M.x11 - v, M.x21, M.x22 - v);  //
}
ChApi SymMat22 CofactorMatrix(const SymMat22& A) {
    SymMat22 T;
    T.x11 = A.x22;   //
    T.x21 = -A.x21;  //
    T.x22 = A.x11;   //
    return T;
}
ChApi real2 LargestColumnNormalized(const SymMat22& A) {
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
ChApi SymMat22 TransposeTimesWithSymmetricResult(const Mat32& A, const Mat32& B) {
    SymMat22 T;
    T.x11 = A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
    T.x21 = A[4] * B[0] + A[5] * B[1] + A[6] * B[2];
    T.x22 = A[4] * B[4] + A[5] * B[5] + A[6] * B[6];

    return T;
}

ChApi SymMat22 ConjugateWithTranspose(const Mat32& A, const SymMat33& B) {
    return TransposeTimesWithSymmetricResult(B * A, A);
}

ChApi void Print(const Mat33& A, const char* name) {
    printf("%s\n", name);
    printf("%f %f %f\n", A[0], A[4], A[8]);
    printf("%f %f %f\n", A[1], A[5], A[9]);
    printf("%f %f %f\n", A[2], A[6], A[10]);
}
ChApi void Print(const Mat32& A, const char* name) {
    printf("%s\n", name);
    printf("%f %f\n", A[0], A[4]);
    printf("%f %f\n", A[1], A[5]);
    printf("%f %f\n", A[2], A[6]);
}
ChApi void Print(const SymMat33& A, const char* name) {
    printf("%s\n", name);

    printf("%f %f %f\n", A.x11, A.x21, A.x31);
    printf("%f %f %f\n", A.x21, A.x22, A.x32);
    printf("%f %f %f\n", A.x31, A.x32, A.x33);
}
ChApi void Print(const SymMat22& A, const char* name) {
    printf("%s\n", name);

    printf("%f %f\n", A.x11, A.x21);
    printf("%f %f\n", A.x21, A.x22);
}

ChApi void PrintLine(const Mat33& A, const char* name) {
    printf("%s: [%f,%f,%f,%f,%f,%f,%f,%f,%f]\n", name, A[0], A[1], A[2], A[4], A[5], A[6], A[8], A[9], A[10]);
}
ChApi void PrintLine(const Mat32& A, const char* name) {
    printf("%s: [%f,%f,%f,%f,%f,%f]\n", name, A[0], A[1], A[2], A[4], A[5], A[6]);
}
ChApi void PrintLine(const SymMat33& A, const char* name) {
    printf("%s: [%f,%f,%f,%f,%f,%f,%f,%f,%f]\n", name, A.x11, A.x21, A.x31, A.x21, A.x22, A.x32, A.x31, A.x32, A.x33);
}
ChApi void PrintLine(const SymMat22& A, const char* name) {
    printf("%s: [%f,%f,%f,%f]\n", name, A.x11, A.x21, A.x21, A.x22);
}
}  // namespace chrono
