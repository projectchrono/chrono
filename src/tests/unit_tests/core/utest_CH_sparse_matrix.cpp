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
// Tests for Chrono sparse matrix classes.
//
// =============================================================================

#include <cmath>
#include <iostream>

#include "gtest/gtest.h"

#include "chrono/core/ChLinkedListMatrix.h"
#include "chrono/core/ChMapMatrix.h"
#include "chrono/core/ChMatrixDynamic.h"

using namespace chrono;

// ----------------------------------------------------

TEST(ChLinkedListMatrix, solve_general) {
    ChLinkedListMatrix A(5, 5);
    A.SetElement(1, 0, 0.130);
    A.SetElement(3, 0, 0.012);
    A.SetElement(2, 1, 1);
    A.SetElement(0, 2, -1);
    A.SetElement(3, 2, 0.337);
    A.SetElement(1, 3, 0.569);
    A.SetElement(4, 3, -0.1);
    A.SetElement(2, 4, 0.469);
    A.SetElement(4, 4, 1);

    double det_ref = 0.006828;

    ChMatrixDynamic<> x_ref(5, 1);
    x_ref(0, 0) = 0.34;
    x_ref(1, 0) = 0.58;
    x_ref(2, 0) = 0.23;
    x_ref(3, 0) = 0.75;
    x_ref(4, 0) = 0.25;

    ChMatrixDynamic<> b(5, 1);
    b(0, 0) = -0.23;
    b(1, 0) = 0.47095;
    b(2, 0) = 0.69725;
    b(3, 0) = 0.08159;
    b(4, 0) = 0.1750;

    ChMatrixDynamic<> x(5, 1);
    int err = A.SolveGeneral(b, x);

    ASSERT_EQ(err, 0);
    ASSERT_TRUE(x.Equals(x_ref, 1e-10));

    double det = A.GetDeterminant();

    ASSERT_NEAR(det, det_ref, 1e-10);
}

TEST(ChLinkedListMatrix, LU_singular) {
    ChLinkedListMatrix A(4, 4);
    A.SetElement(0, 0, 0.5);
    A.SetElement(0, 1, 0.3);
    A.SetElement(0, 2, -0.7);
    A.SetElement(1, 2, 0.2);
    A.SetElement(3, 2, -0.6);
    A.SetElement(3, 3, 0.5);

    int err = A.Setup_LU();
    double det = A.GetDeterminant();

    ASSERT_EQ(err, 3);
    ASSERT_DOUBLE_EQ(det, 0.0);
}

TEST(ChLinkedListMatrix, solve_symmetric) {
    ChLinkedListMatrix A(5, 5);
    A.SetElement(0, 0, 0.32);
    A.SetElement(1, 1, -0.14);
    A.SetElement(2, 2, 0.54);
    A.SetElement(3, 3, -0.40);
    A.SetElement(4, 4, 0.38);

    A.SetElement(0, 1, 0.06);
    A.SetElement(0, 4, -0.08);
    A.SetElement(1, 4, -0.82);

    double det_ref = 0.048555072;

    ChMatrixDynamic<> x_ref(5, 1);
    x_ref(0, 0) = 0.34;
    x_ref(1, 0) = 0.58;
    x_ref(2, 0) = 0.23;
    x_ref(3, 0) = 0.75;
    x_ref(4, 0) = 0.25;

    ChMatrixDynamic<> b(5, 1);
    b(0, 0) = 0.1236;
    b(1, 0) = -0.2658;
    b(2, 0) = 0.1242;
    b(3, 0) = -0.3;
    b(4, 0) = -0.4078;

    ChMatrixDynamic<> x(5, 1);
    int err = A.SolveSymmetric(b, x);

    ASSERT_EQ(err, 0);
    ASSERT_TRUE(x.Equals(x_ref, 1e-10));

    double det = A.GetDeterminant();

    ASSERT_NEAR(det, det_ref, 1e-10);
}

TEST(ChLinkedListMatrix, LDL_singular) {
    ChLinkedListMatrix A(4, 4);
    A.SetElement(0, 0, 0.5);
    A.SetElement(2, 2, 0.3);
    A.SetElement(3, 3, 0.5);

    A.SetElement(0, 2, -0.7);
    A.SetElement(2, 3, -0.6);

    int err = A.Setup_LDL();
    double det = A.GetDeterminant();

    ASSERT_EQ(err, 1);
    ASSERT_DOUBLE_EQ(det, 0.0);
}

// ----------------------------------------------------

TEST(ChMapMatrix, check) {
    ChMapMatrix A(5, 5);
    A.SetElement(1, 0, 0.130);
    A.SetElement(3, 0, 0.012);
    A.SetElement(2, 1, 1);
    A.SetElement(0, 2, -1);
    A.SetElement(3, 2, 0.337);
    A.SetElement(1, 3, 0.569);
    A.SetElement(4, 3, -0.1);
    A.SetElement(2, 4, 0.469);
    A.SetElement(4, 4, 1);
    ////A.StreamOUTsparseMatlabFormat(GetLog());
    ////A.StreamOUT(GetLog());

    A.SetElement(3, 2, -2);
    ASSERT_EQ(A.GetElement(3, 2), -2);

    A.SetElement(1, 3, 1, false);
    ASSERT_EQ(A.GetElement(1, 3), 1.569);

    ChMapMatrix B(A);
    ChMatrixDynamic<double> Ad, Bd;
    A.ConvertToDense(Ad);
    B.ConvertToDense(Bd);

    ASSERT_TRUE(Ad.Equals(Bd));

    std::vector<int> ia;
    std::vector<int> ja;
    std::vector<double> a;
    A.ConvertToCSR(ia, ja, a);

    ASSERT_EQ(ia[0], 0);
    ASSERT_EQ(ia[1], 1);
    ASSERT_EQ(ia[2], 3);
    ASSERT_EQ(ia[3], 5);
    ASSERT_EQ(ia[4], 7);
    ASSERT_EQ(ia[5], 9);

    ASSERT_EQ(ja[0], 2);
    ASSERT_EQ(ja[1], 0);
    ASSERT_EQ(ja[2], 3);
    ASSERT_EQ(ja[3], 1);
    ASSERT_EQ(ja[4], 4);
    ASSERT_EQ(ja[5], 0);
    ASSERT_EQ(ja[6], 2);
    ASSERT_EQ(ja[7], 3);
    ASSERT_EQ(ja[8], 4);

    ASSERT_EQ(a[0], -1.0);
    ASSERT_EQ(a[1], 0.13);
    ASSERT_EQ(a[2], 1.569);
    ASSERT_EQ(a[3], 1.0);
    ASSERT_EQ(a[4], 0.469);
    ASSERT_EQ(a[5], 0.012);
    ASSERT_EQ(a[6], -2.0);
    ASSERT_EQ(a[7], -0.1);
    ASSERT_EQ(a[8], 1.0);
}
