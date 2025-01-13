// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
// Chrono::Multicore unit test for SVD decomposition
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono/multicore_math/matrix.h"
#include "chrono_multicore/cuda/svd.h"

#include "../ut_utils.h"

using namespace chrono;

void Assert_near(const float3& a, const float3& b, float COMPARE_EPS = FLT_EPSILON) {
    ASSERT_NEAR(a.x, b.x, COMPARE_EPS);
    ASSERT_NEAR(a.y, b.y, COMPARE_EPS);
    ASSERT_NEAR(a.z, b.z, COMPARE_EPS);
}

void Assert_near(const SymMat33f& a, const Mat33f& b, float COMPARE_EPS = FLT_EPSILON) {
    ASSERT_NEAR(a[0], b[0], COMPARE_EPS);   // x11
    ASSERT_NEAR(a[1], b[1], COMPARE_EPS);   // x21
    ASSERT_NEAR(a[2], b[2], COMPARE_EPS);   // x31
    ASSERT_NEAR(a[3], b[5], COMPARE_EPS);   // x22
    ASSERT_NEAR(a[4], b[6], COMPARE_EPS);   // x32
    ASSERT_NEAR(a[5], b[10], COMPARE_EPS);  // x33
}

void Assert_near(const Mat33f& a, const Mat33f& b, float COMPARE_EPS = FLT_EPSILON) {
    ASSERT_NEAR(a[0], b[0], COMPARE_EPS);
    ASSERT_NEAR(a[1], b[1], COMPARE_EPS);
    ASSERT_NEAR(a[2], b[2], COMPARE_EPS);
    ASSERT_NEAR(a[4], b[4], COMPARE_EPS);
    ASSERT_NEAR(a[5], b[5], COMPARE_EPS);
    ASSERT_NEAR(a[6], b[6], COMPARE_EPS);
    ASSERT_NEAR(a[8], b[8], COMPARE_EPS);
    ASSERT_NEAR(a[9], b[9], COMPARE_EPS);
    ASSERT_NEAR(a[10], b[10], COMPARE_EPS);
}

// Note that the values in this test were generated using PhysBAM, Matlab/Maple will return different answers (usually
// opposite signs or some columns switched)
TEST(svd, decomposition) {
    const Mat33f B(0.8147f, 0.9058f, 0.1270f, 0.9134f, 0.6324f, 0.0975f, 0.2785f, 0.5469f, 0.9575f);

    printf("NormalEquationsMatrix\n");
    SymMat33f ATA = NormalEquationsMatrix(B);
    Assert_near(ATA, TransposeMult(B, B));

    printf("Fast_Eigenvalues\n");
    float3 eigen_values = Fast_Eigenvalues(ATA);
    Assert_near(eigen_values,
                float3{3.3008110353074768816838969f, 0.7037860796418966558007924f, 0.0329452950506265440644427f},
                1e-7f);
    Mat33f eigen_vectors = Fast_Eigenvectors(ATA, eigen_values);
    // Print(eigen_vectors, "eigen_vectors");
    Assert_near(eigen_vectors,
                Mat33f(0.6556436385585946435838878f, 0.5847646613644273960730402f, 0.4776836924545294627009184f,
                       -0.3055753507677172464696014f, -0.3730222352637652116769118f, 0.8760582840211093014204380f,
                       0.6904745644995428088819267f, -0.7203504028028100414360324f, -0.0658799890785948943916495f),
                1e-7);

    Mat33f U, V;
    float3 SV;
    printf("SVD\n");
    SVD(B, U, SV, V);

    // Print(U, "U");
    printf("U\n");
    Assert_near(U,
                Mat33f(0.6612191451724509505538663f, 0.6742202427583622315054868f, 0.3289624694585512321154397f,
                       -0.4120639447417925316230480f, -0.0400194954888142551130414f, 0.9102756425526574712847605f,
                       0.6268911767613626340178712f, -0.7374452550770638215027475f, 0.2513601962584331994676745f),
                1e-7f);
    printf("V\n");
    Assert_near(V,
                Mat33f(0.6556436385585946435838878f, 0.5847646613644273960730402f, 0.4776836924545294627009184f,
                       -0.3055753507677172464696014f, -0.3730222352637652116769118f, 0.8760582840211093014204380f,
                       0.6904745644995428088819267f, -0.7203504028028100414360324f, -0.0658799890785948943916495f),
                1e-7f);
    printf("SV\n");
    Assert_near(SV, float3{1.8168134288659023578560436f, 0.8389195906890580811676728f, -0.1815083883753765836566174f},
                1e-7);

    const Mat33f A(1, 0, 5, 2, 1, 6, 3, 4, 0);

    printf("Polar Decomposition\n");
    SVD(A, U, SV, V);
    Mat33f R = MultTranspose(U, V);
    Mat33f S = V * Mat33f(SV) * Transpose(V);
    S = (S + Transpose(S)) * .5;
    printf("Check if  A=R*S \n");
    Assert_near(R * S, A, 1e-5f);
}
