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
// Authors: Hammad Mazhar
// =============================================================================
//
// ChronoParallel unit test for MPR collision detection
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>

#include "unit_testing.h"

#include "chrono_parallel/math/matrix.h"
#include "chrono_parallel/math/svd.h"

using namespace chrono;
// Note that the values in this test were generated using PhysBAM, Matlab/Maple will return different answers (usually
// opposite signs or some columns switched)
int main(int argc, char* argv[]) {
    const Mat33 B(0.8147, 0.9058, 0.1270, 0.9134, 0.6324, .0975, 0.2785, 0.5469, 0.9575);
    printf("NormalEquationsMatrix\n");
    SymMat33 ATA = NormalEquationsMatrix(B);
    WeakEqual(ATA, TransposeMult(B, B), C_EPSILON);
    printf("Fast_Eigenvalues\n");
    real3 eigen_values = Fast_Eigenvalues(ATA);
    WeakEqual(eigen_values,
              real3(3.3008110353074768816838969, 0.7037860796418966558007924, 0.0329452950506265440644427), 1e-7);
    Mat33 eigen_vectors = Fast_Eigenvectors(ATA, eigen_values);
    // Print(eigen_vectors, "eigen_vectors");
    WeakEqual(eigen_vectors,
              Mat33(0.6556436385585946435838878, 0.5847646613644273960730402, 0.4776836924545294627009184,
                    -0.3055753507677172464696014, -0.3730222352637652116769118, 0.8760582840211093014204380,
                    0.6904745644995428088819267, -0.7203504028028100414360324, -0.0658799890785948943916495),
              1e-7);
    Mat33 U, V;
    real3 SV;
    printf("SVD\n");
    chrono::SVD(B, U, SV, V);

    // Print(U, "U");
    printf("U\n");
    WeakEqual(U, Mat33(0.6612191451724509505538663, 0.6742202427583622315054868, 0.3289624694585512321154397,
                       -0.4120639447417925316230480, -0.0400194954888142551130414, 0.9102756425526574712847605,
                       0.6268911767613626340178712, -0.7374452550770638215027475, 0.2513601962584331994676745),
              1e-7);
    printf("V\n");
    WeakEqual(V, Mat33(0.6556436385585946435838878, 0.5847646613644273960730402, 0.4776836924545294627009184,
                       -0.3055753507677172464696014, -0.3730222352637652116769118, 0.8760582840211093014204380,
                       0.6904745644995428088819267, -0.7203504028028100414360324, -0.0658799890785948943916495),
              1e-7);
    printf("SV\n");
    WeakEqual(SV, real3(1.8168134288659023578560436, 0.8389195906890580811676728, -0.1815083883753765836566174), 1e-7);

    const Mat33 A(1, 0, 5, 2, 1, 6, 3, 4, 0);

    printf("Polar Decomposition\n");
    chrono::SVD(A, U, SV, V);
    Mat33 R = MultTranspose(U, V);
    Mat33 S = V * Mat33(SV) * Transpose(V);
    S = (S + Transpose(S)) * .5;
    printf("Check if  A=R*S \n");
    WeakEqual(R * S, A, 1e-5);
    return 0;
}
