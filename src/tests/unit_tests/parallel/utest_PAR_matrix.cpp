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

using namespace chrono;
int main(int argc, char* argv[]) {
    real3 n = Normalize(real3(rand(), rand(), rand()));
    quaternion R1 = Normalize(quaternion(rand(), rand(), rand(), rand()));
    quaternion R2 = Normalize(quaternion(rand(), rand(), rand(), rand()));
    const Mat33 AOne(1, 1, 1, 1, 1, 1, 1, 1, 1);
    const Mat33 A1(1, 2, 4, 5, 6, 7, 8, 9, 10);
    const Mat33 A2(10, 2, 4, 7, 2, 5, 8, 3, 1);
    const Mat33 A3(1, 0, 5, 2, 1, 6, 3, 4, 0);
    const Mat33 A4(-24, 20, -5, 18, -15, 4, 5, -4, 1);
    const Mat33 A4_T(-24, 18, 5, 20, -15, -4, -5, 4, 1);
    const Mat33 A5(0.0, 6.4, 3.2, 4.0, -0.8, 3.2, 6.4, 3.2, 5.6);
    ChMatrix33<real> BOne = ChMatrix33<real>(ToChMatrix33(AOne));
    ChMatrix33<real> B1 = ChMatrix33<real>(ToChMatrix33(A1));
    ChMatrix33<real> B2 = ChMatrix33<real>(ToChMatrix33(A2));

    real3 a1(1, 2, 3);
    real3 a2(6, 7, 8);
    ChVector<> b1(1, 2, 3);
    ChVector<> b2(6, 7, 8);
    std::cout << "3x3 Matrix Tests ============\n";

    std::cout << "0 Matrix\n";
    WeakEqual(Mat33(0), ToMat33(ChMatrix33<real>(0)));

    std::cout << "Diag Matrix\n";
    WeakEqual(Mat33(1), Mat33(1, 0, 0, 0, 1, 0, 0, 0, 1));

    std::cout << "Diag 3 Matrix\n";
    WeakEqual(Mat33(real3(1, 2, 3)), Mat33(1, 0, 0, 0, 2, 0, 0, 0, 3));

    std::cout << "Column Constructor\n";
    WeakEqual(Mat33(real3(1, 2, 4), real3(5, 6, 7), real3(8, 9, 10)), A1);

    std::cout << "Element Constructor\n";
    WeakEqual(A4[0], -24);
    WeakEqual(A4[1], 20);
    WeakEqual(A4[2], -5);
    WeakEqual(A4[4], 18);
    WeakEqual(A4[5], -15);
    WeakEqual(A4[6], 4);
    WeakEqual(A4[8], 5);
    WeakEqual(A4[9], -4);
    WeakEqual(A4[10], 1);

    std::cout << "Copy Constructor\n";
    WeakEqual(Mat33(A1), A1);

    std::cout << "Quaternion Constructor \n";
    WeakEqual(Mat33(R1), ToMat33(ToChQuaternion(R1)), C_EPSILON * 3);

    std::cout << "() Operator \n";
    WeakEqual(A4(0, 0), -24);
    WeakEqual(A4(1, 2), -4);

    std::cout << "col Operator \n";
    WeakEqual(A4.col(0), real3(-24, 20, -5));
    WeakEqual(A4.col(1), real3(18, -15, 4));
    WeakEqual(A4.col(2), real3(5, -4, 1));

    std::cout << "row Operator \n";
    WeakEqual(A4.row(0), real3(-24, 18, 5));
    WeakEqual(A4.row(1), real3(20, -15, -4));
    WeakEqual(A4.row(2), real3(-5, 4, 1));

    {
        std::cout << "= Operator\n";
        Mat33 T = A1;
        WeakEqual(T, A1);
    }

    std::cout << "Multiply Matrix\n";
    WeakEqual(AOne * AOne, ToMat33(BOne * BOne));

    std::cout << "Multiply Matrix\n";
    WeakEqual(A1 * A2, ToMat33(B1 * B2));

    std::cout << "Multiply Matrix Vector\n";
    WeakEqual(A1 * a1, ToReal3(B1 * b1));

    std::cout << "Add Matrix\n";
    WeakEqual(A1 + A2, ToMat33(B1 + B2));

    std::cout << "Subtract Matrix\n";
    WeakEqual(A1 - A2, ToMat33(B1 - B2));

    std::cout << "Abs Matrix\n";
    WeakEqual(Abs(A4), Mat33(24, 20, 5, 18, 15, 4, 5, 4, 1));

    std::cout << "Post Scale Matrix\n";
    WeakEqual(A1 * 3.1, ToMat33(B1 * 3.1));

    std::cout << "Pre Scale Matrix\n";
    WeakEqual(3.1 * A1, ToMat33(B1 * 3.1));
    {
        std::cout << "Cross Matrix\n";
        Mat33 cross_m1 = SkewSymmetric(n);
        ChMatrix33<real> cross_m2;
        cross_m2.Set_X_matrix(ToChVector(n));
        WeakEqual(cross_m1, ToMat33(cross_m2));
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
        ChMatrix33<real> Res2;
        Res2.MatrMultiplyT(B1, B2);
        WeakEqual(MultTranspose(A1, A2), ToMat33(Res2), C_EPSILON * 2);
    }

    {
        std::cout << "Outer Product\n";
        Mat33 Res1 = OuterProduct(a1, a2);
        Mat33 Res2(6, 12, 18, 7, 14, 21, 8, 16, 24);
        WeakEqual(Res1, Res2, C_EPSILON);
    }
    std::cout << "Transpose\n";
    WeakEqual(Transpose(A4), A4_T, C_EPSILON);

    std::cout << "Determinant\n";
    WeakEqual(Determinant(A5), 45.056, C_EPSILON * 400);

    std::cout << "Trace\n";
    WeakEqual(Trace(A5), 4.8, C_EPSILON);

    std::cout << "Adjoint\n";
    WeakEqual(Adjoint(A3), A4, C_EPSILON);

    std::cout << "Adjoint Transpose\n";
    WeakEqual(AdjointTranspose(A4), Transpose(A3), C_EPSILON);

    std::cout << "Inverse\n";
    WeakEqual(Inverse(A3), A4, C_EPSILON);

    std::cout << "Inverse Transpose\n";
    WeakEqual(InverseTranspose(A3), Transpose(Inverse(A3)), C_EPSILON);

    std::cout << "Frobenius Norm\n";
    WeakEqual(Norm(A5), 12.674383614203887588, C_EPSILON);

    std::cout << "Largest Column Normalized\n";
    WeakEqual(LargestColumnNormalized(A4),
              real3(-.75856744948921676267, 0.63213954124101396889, -.15803488531025349222), C_EPSILON);

    std::cout << "Normal Equations Matrix\n";
    WeakEqual(NormalEquationsMatrix(A3), Transpose(A3) * A3, C_EPSILON);
    WeakEqual(NormalEquationsMatrix(A3), Mat33(26, 32, 3, 32, 41, 10, 3, 10, 25), C_EPSILON);

    std::cout << "Symm2x2 Matrix Tests ============\n";
    {
        std::cout << "A^T*B With Symmetric Result\n";

        Mat32 C1(real3(1, 2, 3), real3(3, 2, 6));
        Mat32 C2(real3(2, 3, 1), real3(2, 2, 4));

        SymMat22 RES = TransposeTimesWithSymmetricResult(C1, C2);
        WeakEqual(RES, SymMat22(11, 18, 34));
    }

    return 0;
}
