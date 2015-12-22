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
// ChronoParallel unit test for MPR collision detection
// =============================================================================

#include <stdio.h>
#include <vector>
#include <cmath>

#include "unit_testing.h"

#include "chrono_parallel/constraints/ChConstraintRigidRigid.h"
#include "chrono_parallel/math/mat33.h"
#include "chrono_parallel/constraints/ChConstraintUtils.h"

#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/core/ChMathematics.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::utils;
int main(int argc, char* argv[]) {
    real3 n = Normalize(real3(rand(), rand(), rand()));
    quaternion R1 = Normalize(quaternion(rand(), rand(), rand(), rand()));
    quaternion R2 = Normalize(quaternion(rand(), rand(), rand(), rand()));

    const Mat33 A1(1, 2, 4, 5, 6, 7, 8, 9, 10);
    const Mat33 A2(10, 2, 4, 7, 2, 5, 8, 3, 1);
    const Mat33 A3(1, 0, 5, 2, 1, 6, 3, 4, 0);
    const Mat33 A4(-24, 20, -5, 18, -15, 4, 5, -4, 1);
    const Mat33 A4_T(-24, 18, 5, 20, -15, -4, -5, 4, 1);
    ChMatrix33<real> B1 = ChMatrix33<real>(ToChMatrix33(A1));
    ChMatrix33<real> B2 = ChMatrix33<real>(ToChMatrix33(A2));

    {
        std::cout << "Cross Matrix\n";
        real3 v, w;
        chrono::Orthogonalize(n, v, w);
        Mat33 cross_m1 = SkewSymmetric(n);
        ChMatrix33<real> cross_m2;
        cross_m2.Set_X_matrix(ToChVector(n));
        WeakEqual(cross_m1, ToMat33(cross_m2));
    }

    {
        std::cout << "A Matrix\n";
        Mat33 A1(R1);
        ChMatrix33<real> A2 = ChMatrix33<real>(ToChQuaternion(R1));
        WeakEqual(A1, ToMat33(A2), C_EPSILON * 3);
    }
    {
        std::cout << "Multiply Matrix\n";
        Mat33 Res1 = A1 * A2;
        ChMatrix33<real> Res2 = B1 * B2;
        WeakEqual(Res1, ToMat33(Res2));
    }
    {
        std::cout << "Add Matrix\n";
        Mat33 Res1 = A1 + A2;
        ChMatrix33<real> Res2 = B1 + B2;
        WeakEqual(Res1, ToMat33(Res2));
    }
    {
        std::cout << "Subtract Matrix\n";
        Mat33 Res1 = A1 - A2;
        ChMatrix33<real> Res2 = B1 - B2;
        WeakEqual(Res1, ToMat33(Res2));
    }
    {
        std::cout << "Post Scale Matrix\n";
        Mat33 Res1 = A1 * 3.1;
        ChMatrix33<real> Res2 = B1 * 3.1;
        WeakEqual(Res1, ToMat33(Res2));
    }
    {
        std::cout << "Pre Scale Matrix\n";
        Mat33 Res1 = 3.1 * A1;
        ChMatrix33<real> Res2 = B1 * 3.1;
        WeakEqual(Res1, ToMat33(Res2));
    }
    {
        std::cout << "Multiply T Matrix \n";
        Mat33 Res1 = TransposeMult(A1, A2);
        ChMatrix33<real> Res2;
        Res2.MatrTMultiply(B1, B2);
        WeakEqual(Res1, ToMat33(Res2), C_EPSILON * 2);
    }

    {
        std::cout << "Multiply Matrix T\n";
        Mat33 Res1 = MultTranspose(A1, A2);
        ChMatrix33<real> Res2;
        Res2.MatrMultiplyT(B1, B2);
        WeakEqual(Res1, ToMat33(Res2), C_EPSILON * 2);
    }

    {
        std::cout << "Transpose\n";
        WeakEqual(Transpose(A4), A4_T, C_EPSILON);
    }
    {
        std::cout << "Outer Product\n";
        real3 a(1, 2, 3);
        real3 b(6, 7, 8);

        Mat33 Res1 = OuterProduct(a, b);
        Mat33 Res2(6, 12, 18, 7, 14, 21, 8, 16, 24);
        WeakEqual(Res1, Res2, C_EPSILON);
    }

    std::cout << "Determinant\n";
    WeakEqual(Determinant(A3), 1, C_EPSILON);

    std::cout << "Trace\n";
    WeakEqual(Trace(A1), 17, C_EPSILON);

    std::cout << "Adjoint\n";
    WeakEqual(Adjoint(A3), A4, C_EPSILON);

    std::cout << "Adjoint Transpose\n";
    WeakEqual(AdjointTranspose(A4), Transpose(A3), C_EPSILON);

    std::cout << "Inverse\n";
    WeakEqual(Inverse(A3), A4, C_EPSILON);

    std::cout << "Inverse Transpose\n";
    WeakEqual(InverseTranspose(A3), Transpose(Inverse(A3)), C_EPSILON);

    return 0;
}
