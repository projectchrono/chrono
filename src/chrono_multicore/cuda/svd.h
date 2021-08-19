// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: Fast Singular Value Decomposition (SVD)
// =============================================================================

#pragma once

#include "chrono_multicore/cuda/matrixf.cuh"

namespace chrono {

/// @addtogroup multicore_math
/// @{

// Oliver K. Smith. 1961. Eigenvalues of a symmetric 3 × 3 matrix. Commun. ACM 4, 4 (April 1961), 168-.
// DOI=http://dx.doi.org/10.1145/355578.366316
CUDA_HOST_DEVICE static float3 Fast_Eigenvalues(const SymMat33f& A)  // 24 mults, 20 adds, 1 atan2, 1 sincos, 2 sqrts
{
    const float m = (1.0f) / (3.0f) * (A.x11 + A.x22 + A.x33);
    const float a11 = A.x11 - m;
    const float a22 = A.x22 - m;
    const float a33 = A.x33 - m;
    const float a12_sqr = A.x21 * A.x21;
    const float a13_sqr = A.x31 * A.x31;
    const float a23_sqr = A.x32 * A.x32;
    const float p = (1.0f) / (6.0f) * (a11 * a11 + a22 * a22 + a33 * a33 + 2 * (a12_sqr + a13_sqr + a23_sqr));
    const float q = (0.5f) * (a11 * (a22 * a33 - a23_sqr) - a22 * a13_sqr - a33 * a12_sqr) + A.x21 * A.x31 * A.x32;
    const float sqrt_p = sqrtf(p);
    const float disc = p * p * p - q * q;
    const float phi = (1.0f) / (3.0f) * atan2f(sqrtf(fmaxf((0.0f), disc)), q);
    const float c = cosf(phi);
    const float s = sinf(phi);
    const float sqrt_p_cos = sqrt_p * c;
    const float root_three_sqrt_p_sin = sqrtf((3.0f)) * sqrt_p * s;
    float3 lambda = make_float3(m + (2.0f) * sqrt_p_cos, m - sqrt_p_cos - root_three_sqrt_p_sin,
                                m - sqrt_p_cos + root_three_sqrt_p_sin);
    Sort(lambda.z, lambda.y, lambda.x);
    return lambda;
}

CUDA_HOST_DEVICE static Mat33f Fast_Eigenvectors(const SymMat33f& A, float3& lambda) {
    // flip if necessary so that first eigenvalue is the most different
    bool flipped = false;
    float3 lambda_flip(lambda);
    if (lambda.x - lambda.y < lambda.y - lambda.z) {  // 2a
        Swap(lambda_flip.x, lambda_flip.z);
        flipped = true;
    }

    // get first eigenvector
    const float3 v1 =
        LargestColumnNormalized(CofactorMatrix(A - lambda_flip.x));  // 3a + 12m+6a + 9m+6a+1d+1s = 21m+15a+1d+1s
    // form basis for orthogonal complement to v1, and reduce A to this space
    const float3 v1_orthogonal = UnitOrthogonalVector(v1);           // 6m+2a+1d+1s (tweak: 5m+1a+1d+1s)
    const Mat32f other_v(v1_orthogonal, Cross(v1, v1_orthogonal));   // 6m+3a (tweak: 4m+1a)
    const SymMat22f A_reduced = ConjugateWithTranspose(other_v, A);  // 21m+12a (tweak: 18m+9a)
    // find third eigenvector from A_reduced, and fill in second via cross product

    // 6m+3a + 2a + 5m+2a+1d+1s = 11m+7a+1d+1s (tweak: 10m+6a+1d+1s)

    const float3 v3 = other_v * LargestColumnNormalized(CofactorMatrix(A_reduced - lambda_flip.z));

    const float3 v2 = Cross(v3, v1);  // 6m+3a
    // finish
    return flipped ? Mat33f(v3.x, v3.y, v3.z, v2.x, v2.y, v2.z, -v1.x, -v1.y, -v1.z)
                   : Mat33f(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, v3.x, v3.y, v3.z);
}

CUDA_HOST_DEVICE static void Fast_Solve_EigenProblem(const SymMat33f& A, float3& eigen_values, Mat33f& eigen_vectors) {
    eigen_values = Fast_Eigenvalues(A);
    eigen_vectors = Fast_Eigenvectors(A, eigen_values);
}

CUDA_HOST_DEVICE static void SVD(const Mat33f& A, Mat33f& U, float3& singular_values, Mat33f& V) {
    const SymMat33f ATA = NormalEquationsMatrix(A);
    float3 lambda;
    Fast_Solve_EigenProblem(ATA, lambda, V);

    if (lambda.z < 0) {
        lambda = Max(lambda, (0.0f));
    }
    singular_values = Sqrt(lambda);  // 3s
    if (Determinant(A) < 0) {
        singular_values.z = -singular_values.z;
    }

    // compute singular vectors
    const float3 c0 = Normalize(A * V.col(0));   // 15m+8a+1d+1s
    const float3 v1 = UnitOrthogonalVector(c0);  // 6m+2a+1d+1s
    const float3 v2 = Cross(c0, v1);             // 6m+3a

    // 6m+3a + 6m+4a + 9m+6a + 6m+2a+1d+1s = 27m+15a+1d+1s
    const float3 v3 = A * V.col(1);
    const float2 other_v = Normalize(make_float2(Dot(v1, v3), Dot(v2, v3)));
    const float3 c1 = make_float3(v1.x * other_v.x + v2.x * other_v.y, v1.y * other_v.x + v2.y * other_v.y,
                                  v1.z * other_v.x + v2.z * other_v.y);
    const float3 c2 = Cross(c0, c1);  // 6m+3a

    U = Mat33f(c0.x, c0.y, c0.z, c1.x, c1.y, c1.z, c2.x, c2.y, c2.z);
}

/// @} multicore_math

} // end namespace chrono
