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
// Authors: Radu Serban
// =============================================================================
//
// Tests for miscellaneous linear algebra support.
//
// =============================================================================

#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMatrixMBD.h"
#include "chrono/core/ChTransform.h"

#include "gtest/gtest.h"

using std::cout;
using std::endl;
using namespace chrono;

const double ABS_ERR = 1e-10;

TEST(LinearAlgebraTest, create_assign) {
    ChMatrixDynamic<> Md1(5, 7);
    ChMatrixDynamic<> Md2(4, 4);
    ChMatrixNM<double, 4, 4> Ms;
    ChMatrix33<> A33(Q_from_AngY(0.4));

    Ms.setConstant(0.1);  // Fill a matrix with an element
    ASSERT_DOUBLE_EQ(Ms.maxCoeff(), 0.1);
    ASSERT_DOUBLE_EQ(Ms.minCoeff(), 0.1);

    Md1.setRandom();         // initialize with random numbers
    double e32 = Md1(3, 2);
    Md1.transposeInPlace();  // transpose the matrix in place
    ASSERT_DOUBLE_EQ(Md1(2, 3), e32);

    Md1.resize(2, 2);        // resize
    Md1.setZero();           // set all elements to zero
    ASSERT_DOUBLE_EQ(Md1.maxCoeff(), 0.0);
    ASSERT_DOUBLE_EQ(Md1.minCoeff(), 0.0);

    Md2.setIdentity();  // Create a diagonal matrix
    Md2 *= 3;
    ASSERT_DOUBLE_EQ(Md2.trace(), 4 * 3.0);

    ChMatrixDynamic<> M1(Md2);   // Copy constructor
    ChMatrixDynamic<> M2(-M1);   // The - unary operator returns a negated matrix
    ChMatrixDynamic<> M3(8, 4);  // 8x4 uninitialized matrix
    M3 = M1;                     // Copy (resize as needed)
    M3 = M1.transpose();         // transposed copy.
    ASSERT_TRUE((M3 + M2.transpose()).isZero());

    ChMatrix33<> B33(1.0);
    cout << "3x3 identity matrix\n" << B33 << endl;
    ASSERT_DOUBLE_EQ(B33.trace(), 3.0);
}

TEST(LinearAlgebraTest, operations) {
    ChMatrixNM<double, 2, 3> A;
    ChMatrixDynamic<double> B(2, 3);
    A.setRandom();
    B.setRandom();

    ChMatrixDynamic<> sum1(A + B);
    ChMatrixDynamic<> sum2 = A + B;
    ChMatrixDynamic<> sum3 = A;
    sum3 += B;
    ASSERT_TRUE(sum2 == sum1);
    ASSERT_TRUE(sum3 == sum1);

    // Different ways to do subtraction..
    ChMatrixDynamic<> diff1 = A - B;
    ChMatrixDynamic<> diff2 = A;
    diff2 -= B;
    ASSERT_TRUE(diff1 == diff2);

    // Multiplication with scalar
    ChMatrixDynamic<> ms1 = A * 10;
    ChMatrixNM<double, 2, 3> ms2 = 10 * A;
    ChMatrixDynamic<> ms3 = A;
    ms3 *= 10;
    ASSERT_TRUE(ms2 == ms1);
    ASSERT_TRUE(ms3 == ms1);

    // Matrix multiplications
    ChMatrixDynamic<> C(3, 2);
    ChMatrixDynamic<> D(2, 3);
    C << 1, 2, 3, 4, 5, 6;
    D << 1, 2, 3, 4, 5, 6;
    ChMatrix33<> R(Q_from_AngX(CH_C_PI_2));

    ChMatrixDynamic<double> CD = C * D;
    ChMatrixDynamic<double> CD_t = D.transpose() * C.transpose();
    ChMatrixDynamic<> RC = R * C;
    ChMatrixDynamic<> DR = D * R;
    cout << R << "\n";
    cout << "rot * matrix\n" << R * C << endl;
    cout << "matrix * rot\n" << D * R << endl;
    ASSERT_TRUE(CD.transpose() == CD_t);
    ASSERT_DOUBLE_EQ(RC(1, 0), -C(2, 0));
    ASSERT_DOUBLE_EQ(DR(1, 2), -D(1, 1));

    // Component-wise matrix multiplication and division
    ChMatrixDynamic<> C_times_D = C.array() * D.transpose().array();
    ChMatrixDynamic<> C_div_D = C.array() / D.transpose().array();
    cout << "C\n" << C << endl;
    cout << "D'\n" << D.transpose() << endl;
    cout << "C_times_D' (component-wise)\n" << C_times_D << endl;
    cout << "C_div_D' (component-wise)\n" << C_div_D << endl;

    // Assigning matrix rows
    ChMatrix33<> J;
    J << 10, 20, 30, 40, 50, 60, 70, 80, 90;
    cout << "3x3 matrix J\n" << J << endl;
    ChVectorN<double, 10> V;
    V.setZero();
    V.segment(3, 3) = J.row(0);
    cout << "Place row0 in V starting at index 3\n" << V.transpose() << endl;
    V.segment(7, 3) = J.row(1);
    cout << "Place row1 in V starting at index 7\n" << V.transpose() << endl;
}

TEST(LinearAlgebraTest, vector_rotation) {
    ChQuaternion<> q(1, 2, 3, 4);
    ChMatrix33<> A(q.GetNormalized());

    ChVector<> v1(1, 2, 3);
    ChVector<> v2 = A * v1;
    ChVector<> v3 = A.transpose() * v2;

    cout << A << endl;
    cout << v1 << endl;
    cout << v2 << endl;
    cout << v3 << endl;
    ASSERT_TRUE(v3.Equals(v1, 1e-8));
}

TEST(LinearAlgebraTest, extensions) {
    ChMatrixNM<double, 2, 3> A;
    A.setRandom();
    cout << "random 2x3 matrix A:\n" << A << endl;
    A.fillDiagonal(10.1);
    cout << "fill diagonal with 10.1:\n" << A << endl;
    A.fill(2.1);
    cout << "fill entire matrix with 2.1:\n" << A << endl;

    ChMatrixNM<double, 2, 3> B(A);
    B(1, 2) += 0.01;
    cout << "matrix B = A with B(1,2) incremented by 0.01\n" << B << endl;
    cout << "|A-B| < 0.1?   " << A.equals(B, 0.1) << endl;
    cout << "|A-B| < 0.001? " << A.equals(B, 0.001) << endl;
    ASSERT_TRUE(A.equals(B, 0.1));
    ASSERT_FALSE(A.equals(B, 0.001));

    ChVectorDynamic<> v(3);
    v << 2, 3, 4;
    ChVectorDynamic<> w(3);
    w << 0.1, 0.2, 0.3;
    double wrms_ref = std::sqrt((0.2 * 0.2 + 0.6 * 0.6 + 1.2 * 1.2) / 3);
    cout << "||v||_wrms, w = " << v.wrmsNorm(w) << endl;
    cout << "||v + v||_wrms, w = " << (v + v).wrmsNorm(w) << endl;
    ASSERT_NEAR(v.wrmsNorm(w), wrms_ref, ABS_ERR);
    ASSERT_NEAR((v + v).wrmsNorm(w), 2 * wrms_ref, ABS_ERR);

    cout << "v + v + 1: " << (v + v) + 1 << endl;
    cout << "1 + v: " << 1 + v << endl;
}

TEST(LinearAlgebraTest, pasting) {
    ChMatrixDynamic<> A(4, 6);
    A.setRandom();
    cout << A << "\n\n";

    ChMatrixNM<double, 2, 3> B;
    B << 1, 2, 3, 4, 5, 6;
    cout << B << "\n\n";

    ChMatrixNM<double, 5, 7> C;
    C.setZero();
    C.block<2, 3>(1, 1) << 10, 20, 30, 40, 50, 60;
    cout << C << "\n\n";

    ChMatrixDynamic<> X = A;
    X.block<2, 3>(1, 2) = B;
    cout << X << "\n\n";
    ASSERT_DOUBLE_EQ(X(1, 2), B(0, 0));
    ASSERT_DOUBLE_EQ(X(1, 3), B(0, 1));
    ASSERT_DOUBLE_EQ(X(1, 4), B(0, 2));

    X.block<2, 3>(1, 2) += C.block<2, 3>(1, 1);
    cout << X << "\n\n";
    ASSERT_DOUBLE_EQ(X(1, 2), B(0, 0) + C(1, 1));
    ASSERT_DOUBLE_EQ(X(1, 3), B(0, 1) + C(1, 2));
    ASSERT_DOUBLE_EQ(X(1, 4), B(0, 2) + C(1, 3));
}

TEST(LinearAlgebraTest, custom_matrices) {
    ChMatrix34<> G = Eigen::Matrix<double, 3, 4>::Ones(3, 4);
    ChQuaternion<> q(1, 2, 3, 4);
    ChVector<> v = G * q;
    cout << v << endl;

    ChMatrix43<> Gt2 = 2 * G.transpose();
    cout << Gt2 << endl;
    q = Gt2 * v;
    cout << q << endl;
    q = G.transpose() * v;
    cout << q << endl;

    ChMatrix33<> rot(Q_from_AngX(CH_C_PI / 6));
    cout << rot << endl;
    cout << rot.transpose() << endl;

    ChMatrix34<> res1 = rot * G;
    ChMatrix34<> res2 = rot * Gt2.transpose();
    ChMatrix43<> res3 = Gt2 * rot;
    ChMatrix43<> res4 = G.transpose() * rot;
    ASSERT_TRUE(2 * res1 == res2);
    ASSERT_TRUE(res3 == 2 * res4);

    ////ChMatrix44<> A44;
    ////A44.setRandom();
    ////ChQuaternion<> q2 = A44 * q;

    ChStarMatrix44<> X(ChQuaternion<>(1, 2, 3, 4));
    cout << "4x4 star matrix X:\n" << X << endl;
    ASSERT_TRUE(X(1, 2) == -4);

    X.semiTranspose();
    cout << "Semi-transpose X:\n" << X << endl;
    ASSERT_TRUE(X(1, 2) == 4);

    X.semiNegate();
    cout << "Semi-negate X:\n" << X << endl;
    ASSERT_TRUE(X(1, 2) == -4);
}

TEST(LinearAlgebra, slicing) {
    ChVectorN<double, 5> v1;
    v1 << 1, 2, 3, 4, 5;
    ChVectorDynamic<double> v2(5);
    v2(0) = 1;
    v2(1) = 2;
    v2(2) = 3;
    v2(3) = 4;
    v2(4) = 5;

    ChArray<int> idx(3);
    idx(0) = 1;
    idx(1) = 3;
    idx(2) = 4;

    auto w1 = SliceVector(v1, idx);
    auto w2 = SliceVector(v2, idx);

    ASSERT_TRUE(w1.size() == idx.size());
    ASSERT_TRUE(w2.size() == idx.size());
    for (int i = 0; i < idx.size(); i++) {
        ASSERT_TRUE(w1(i) == v1(idx(i)));
        ASSERT_TRUE(w2(i) == v2(idx(i)));
    }

    // Cases with potential aliasing
    // NOTE: no aliasing when SliceVector is implemented as a function. Aliasing possible if switching to a macro!
    v2 = SliceVector(v2, idx).eval();
    
    ASSERT_TRUE(v2.size() == idx.size());
    for (int i = 0; i < idx.size(); i++) {
        ASSERT_TRUE(v2(i) == w2(i));
    }
}

TEST(LinearAlgebraTest, solve) {
    ChMatrixNM<double, 3, 3> A;
    Eigen::Vector3d b;
    A << 1, 2, 3, 4, 5, 6, 7, 8, 10;
    b << 3, 3, 4;
    cout << "matrix A:\n" << A << endl;
    cout << "vector b:\n" << b << endl;
    Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
    cout << "solution:\n" << x << endl;
    cout << "Ax-b:\n" << A * x - b << endl;
    ASSERT_NEAR((A * x - b).norm(), 0.0, ABS_ERR);
}
