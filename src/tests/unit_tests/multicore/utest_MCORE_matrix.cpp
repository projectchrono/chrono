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
// Chrono::Multicore unit test for MPR collision detection
// =============================================================================

#include "chrono/multicore_math/matrix.h"
#include "chrono/core/ChMatrixMBD.h"

#include "../ut_utils.h"

using namespace chrono;

class Mat33Test : public ::testing::Test {
  protected:
    void SetUp() override {
        n = Normalize(real3(rand(), rand(), rand()));

        R1 = Normalize(quaternion(rand(), rand(), rand(), rand()));
        R2 = Normalize(quaternion(rand(), rand(), rand(), rand()));

        AOne = Mat33(1, 1, 1, 1, 1, 1, 1, 1, 1);
        A1 = Mat33(1, 2, 4, 5, 6, 7, 8, 9, 10);
        A2 = Mat33(10, 2, 4, 7, 2, 5, 8, 3, 1);
        A3 = Mat33(1, 0, 5, 2, 1, 6, 3, 4, 0);
        A4 = Mat33(-24, 20, -5, 18, -15, 4, 5, -4, 1);
        A4_T = Mat33(-24, 18, 5, 20, -15, -4, -5, 4, 1);
        A5 = Mat33(0.0, 6.4, 3.2, 4.0, -0.8, 3.2, 6.4, 3.2, 5.6);

        BOne = ToChMatrix33(AOne);
        B1 = ToChMatrix33(A1);
        B2 = ToChMatrix33(A2);

        a1 = real3(1, 2, 3);
        a2 = real3(6, 7, 8);

        b1 = ChVector3d(1, 2, 3);
        b2 = ChVector3d(6, 7, 8);
    }

    real3 n;
    quaternion R1, R2;
    Mat33 AOne, A1, A2, A3, A4, A4_T, A5;
    ChMatrix33<real> BOne, B1, B2;
    real3 a1, a2;
    ChVector3d b1, b2;
};

TEST_F(Mat33Test, constructors) {
    // 0 Matrix
    Assert_eq(Mat33(0), FromChMatrix33(ChMatrix33<real>(0)));

    // Diag Matrix
    Assert_eq(Mat33(1), Mat33(1, 0, 0, 0, 1, 0, 0, 0, 1));

    // Diag 3 Matrix
    Assert_eq(Mat33(real3(1, 2, 3)), Mat33(1, 0, 0, 0, 2, 0, 0, 0, 3));

    // Column Constructor
    Assert_eq(Mat33(real3(1, 2, 4), real3(5, 6, 7), real3(8, 9, 10)), A1);

    // Element Constructor
    ASSERT_EQ(A4[0], -24);
    ASSERT_EQ(A4[1], 20);
    ASSERT_EQ(A4[2], -5);
    ASSERT_EQ(A4[4], 18);
    ASSERT_EQ(A4[5], -15);
    ASSERT_EQ(A4[6], 4);
    ASSERT_EQ(A4[8], 5);
    ASSERT_EQ(A4[9], -4);
    ASSERT_EQ(A4[10], 1);

    // Copy Constructor
    Assert_eq(Mat33(A1), A1);

    // Quaternion Constructor
    Assert_near(Mat33(R1), FromChMatrix33(ToChQuaternion(R1)), C_REAL_EPSILON * 3);
}

TEST_F(Mat33Test, operators) {
    const Mat33 A4(-24, 20, -5, 18, -15, 4, 5, -4, 1);

    // () Operator
    ASSERT_EQ(A4(0, 0), -24);
    ASSERT_EQ(A4(1, 2), -4);

    // col Operator
    Assert_eq(A4.col(0), real3(-24, 20, -5));
    Assert_eq(A4.col(1), real3(18, -15, 4));
    Assert_eq(A4.col(2), real3(5, -4, 1));

    // row Operator
    Assert_eq(A4.row(0), real3(-24, 18, 5));
    Assert_eq(A4.row(1), real3(20, -15, -4));
    Assert_eq(A4.row(2), real3(-5, 4, 1));

    {
        // = Operator
        Mat33 T = A1;
        Assert_eq(T, A1);
    }

    // Multiply Matrix
    Assert_near(AOne * AOne, FromChMatrix33(BOne * BOne));

    // Multiply Matrix
    Assert_near(A1 * A2, FromChMatrix33(B1 * B2));

    // Multiply Matrix Vector
    Assert_near(A1 * a1, FromChMatrix33(B1 * b1));

    // Add Matrix
    Assert_near(A1 + A2, FromChMatrix33(B1 + B2));

    // Subtract Matrix
    Assert_near(A1 - A2, FromChMatrix33(B1 - B2));

    // Abs Matrix
    Assert_near(Abs(A4), Mat33(24, 20, 5, 18, 15, 4, 5, 4, 1));

    // Post Scale Matrix
    Assert_near(A1 * 3.1, FromChMatrix33(B1 * 3.1));

    // Pre Scale Matrix
    Assert_near(3.1 * A1, FromChMatrix33(B1 * 3.1));
}

TEST_F(Mat33Test, functions) {
    {
        // Cross Matrix
        Mat33 cross_m1 = SkewSymmetric(n);
        ChStarMatrix33<real> cross_m2(ToChVector(n));
        Assert_near(cross_m1, FromChMatrix33(cross_m2));
    }
    {
        // Multiply T Matrix
        Mat33 Res1 = TransposeMult(A1, A2);
        ChMatrix33<real> Res2 = B1.transpose() * B2;
        Assert_near(Res1, FromChMatrix33(Res2), C_REAL_EPSILON * 2);
    }
    {
        // Multiply Matrix T
        ChMatrix33<real> Res2 = B1 * B2.transpose();
        Assert_near(MultTranspose(A1, A2), FromChMatrix33(Res2), C_REAL_EPSILON * 2);
    }
    {
        // Outer Product
        Mat33 Res1 = OuterProduct(a1, a2);
        Mat33 Res2(6, 12, 18, 7, 14, 21, 8, 16, 24);
        Assert_near(Res1, Res2, C_REAL_EPSILON);
    }
    // Transpose
    Assert_near(Transpose(A4), A4_T, C_REAL_EPSILON);

    // Determinant
    ASSERT_NEAR(Determinant(A5), 45.056, C_REAL_EPSILON * 400);

    // Trace
    ASSERT_NEAR(Trace(A5), 4.8, C_REAL_EPSILON);

    // Adjoint
    Assert_near(Adjoint(A3), A4, C_REAL_EPSILON);

    // Adjoint Transpose
    Assert_near(AdjointTranspose(A4), Transpose(A3), C_REAL_EPSILON);

    // Inverse
    Assert_near(Inverse(A3), A4, C_REAL_EPSILON);

    // Inverse Transpose
    Assert_near(InverseTranspose(A3), Transpose(Inverse(A3)), C_REAL_EPSILON);

    // Frobenius Norm
    ASSERT_NEAR(Norm(A5), 12.674383614203887588, C_REAL_EPSILON);

    // Largest Column Normalized
    Assert_near(LargestColumnNormalized(A4),
                real3(-.75856744948921676267, 0.63213954124101396889, -.15803488531025349222), C_REAL_EPSILON);

    // Normal Equations Matrix
    Assert_near(NormalEquationsMatrix(A3), Transpose(A3) * A3, C_REAL_EPSILON);
    Assert_near(NormalEquationsMatrix(A3), Mat33(26, 32, 3, 32, 41, 10, 3, 10, 25), C_REAL_EPSILON);

    // Symm2x2 Matrix Tests ============
    {
        // A^T*B With Symmetric Result

        Mat32 C1(real3(1, 2, 3), real3(3, 2, 6));
        Mat32 C2(real3(2, 3, 1), real3(2, 2, 4));

        SymMat22 RES = TransposeTimesWithSymmetricResult(C1, C2);
        Assert_near(RES, SymMat22(11, 18, 34));
    }
}
