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
// Authors: Tyler Olsen, Radu Serban
// =============================================================================
//
// Unit test for ChFunctionLambda
//
// =============================================================================

#include "gtest/gtest.h"

#include "motion_functions/ChFunctionLambda.h"

using namespace chrono;

TEST(ChFunctionLambdaTest, square) {
    auto F1 = make_ChFunctionLambda([](auto x) { return x * x; });
    ASSERT_DOUBLE_EQ(F1.Get_y(3.0), 9.0);
    ASSERT_DOUBLE_EQ(F1.Get_y_dx(3.0), 6.0);
    ASSERT_DOUBLE_EQ(F1.Get_y_dxdx(3.0), 2.0);
}

TEST(ChFunctionLambdaTest, exponential) {
    auto F2 = make_ChFunctionLambda([](auto x) { return exp(2 * x); });
    ASSERT_DOUBLE_EQ(F2.Get_y(2.0), std::exp(4.0));
    ASSERT_DOUBLE_EQ(F2.Get_y_dx(2.0), 2.0 * std::exp(4.0));
    ASSERT_DOUBLE_EQ(F2.Get_y_dxdx(2.0), 4.0 * std::exp(4.0));
}

TEST(ChFunctionLambdaTest, cosine) {
    auto F3 = make_shared_ChFunctionLambda([](auto x) { return cos(x); });
    ASSERT_DOUBLE_EQ(F3->Get_y(5.0), std::cos(5.0));
    ASSERT_DOUBLE_EQ(F3->Get_y_dx(5.0), -std::sin(5.0));
    ASSERT_DOUBLE_EQ(F3->Get_y_dxdx(5.0), -std::cos(5.0));
}
