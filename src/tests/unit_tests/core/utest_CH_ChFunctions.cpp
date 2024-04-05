// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Tyler Olsen, Radu Serban, Dario Mangoni
// =============================================================================
//
// Unit test for ChFunctions
//
// =============================================================================
#include <cmath>

#include "gtest/gtest.h"
#include "chrono/functions/ChFunctionLambda.h"
#include "chrono/functions/ChFunctionInterp.h"
#include "chrono/utils/ChConstants.h"

using namespace chrono;

#define TOL_FUN 1e-5

TEST(ChFunctionLambda, basic_usage) {
    ChFunctionLambda fun;
    fun.SetFunction([](double x) { return std::sin(x); });

    ASSERT_NEAR(fun.GetVal(-CH_2PI), std::sin(-CH_2PI), TOL_FUN);
    ASSERT_NEAR(fun.GetVal(0.0), 0.0, TOL_FUN);
    ASSERT_NEAR(fun.GetVal(1.0), std::sin(1.0), TOL_FUN);
    ASSERT_NEAR(fun.GetVal(CH_2PI), std::sin(CH_2PI), TOL_FUN);
    ASSERT_NEAR(fun.GetVal(7 * CH_2PI), std::sin(7 * CH_2PI), TOL_FUN);
}

TEST(ChFunctionInterp, interp1_noextrap) {
    //// Baseline from Matlab R2023b
    // x = [-1.7, -1.0, 0.0, 0.1, 9.8, 11.3];
    // v = [-11.7, -15.0, 2.7, 0.3, 13.5, -2.4];
    // xq = [-5, 0.05, x(1), 0.0, x(end), 18.3];
    // vq_extrap = interp1(x, v, xq, 'linear');

    ChFunctionInterp fun_table;
    fun_table.AddPoint(0.0, 2.7);
    fun_table.AddPoint(0.1, 0.3);
    fun_table.AddPoint(9.8, 13.5);
    fun_table.AddPoint(-1.7, -11.7);
    fun_table.AddPoint(-1.0, -15.0);
    fun_table.AddPoint(11.3, -2.4);

    ASSERT_NEAR(fun_table.GetVal(-5), -11.7, TOL_FUN);
    ASSERT_NEAR(fun_table.GetVal(0.05), 1.5, TOL_FUN);
    ASSERT_NEAR(fun_table.GetVal(-1.7), -11.7, TOL_FUN);
    ASSERT_NEAR(fun_table.GetVal(0.0), 2.7, TOL_FUN);
    ASSERT_NEAR(fun_table.GetVal(11.3), -2.4, TOL_FUN);
    ASSERT_NEAR(fun_table.GetVal(18.3), -2.4, TOL_FUN);
}

TEST(ChFunctionInterp, interp1_extrap) {
    //// Baseline from Matlab R2023b
    // x = [-1.7, -1.0, 0.0, 0.1, 9.8, 11.3];
    // v = [-11.7, -15.0, 2.7, 0.3, 13.5, -2.4];
    // xq = [-5, x(1), 0.0, 0.05, -1.5, -0.7, 3.7, 0.0, x(end), 18.3];
    // vq_extrap = interp1(x, v, xq, 'linear', 'extrap');
    // format rational % for some results

    ChFunctionInterp fun_table;
    fun_table.SetExtrapolate(true);
    fun_table.AddPoint(0.0, 2.7);
    fun_table.AddPoint(0.1, 0.3);
    fun_table.AddPoint(9.8, 13.5);
    fun_table.AddPoint(-1.7, -11.7);
    fun_table.AddPoint(-1.0, -15.0);
    fun_table.AddPoint(11.3, -2.4);

    ASSERT_NEAR(fun_table.GetVal(-5), 27.0 / 7.0, TOL_FUN);
    ASSERT_NEAR(fun_table.GetVal(-1.7), -117.0 / 10.0, TOL_FUN);
    ASSERT_NEAR(fun_table.GetVal(0.0), 2.7, TOL_FUN);
    ASSERT_NEAR(fun_table.GetVal(0.05), 1.5, TOL_FUN);
    ASSERT_NEAR(fun_table.GetVal(-1.5), -177.0 / 14.0, TOL_FUN);
    ASSERT_NEAR(fun_table.GetVal(-0.7), -9.69, TOL_FUN);
    ASSERT_NEAR(fun_table.GetVal(3.7), 2012.0 / 387.0, TOL_FUN);
    ASSERT_NEAR(fun_table.GetVal(0.0), 2.7, TOL_FUN);
    ASSERT_NEAR(fun_table.GetVal(11.3), -2.4, TOL_FUN);
    ASSERT_NEAR(fun_table.GetVal(18.3), -76.6, TOL_FUN);

    // check operator() overloading
    // ASSERT_DOUBLE_EQ(fun_table.GetVal(-0.7), fun_table(-0.7));
}

// TEST(ChFunctionInterp, wrong_insertions) {
//    ChFunctionInterp fun_table_noovr;
//    fun_table_noovr.AddPoint(0.0, 2.7);
//    bool thrown = false;
//    try {
//        fun_table_noovr.AddPoint(0.0, 0.3);
//    } catch (const std::invalid_argument& e) {
//        std::cout << "Caught exception '" << e.what() << "'" << std::endl;
//        thrown = true;
//    }
//    ASSERT_TRUE(thrown);
//
//    ChFunctionInterp fun_table_ovr;
//    fun_table_ovr.AddPoint(0.0, 2.7);
//    EXPECT_NO_THROW(fun_table_ovr.AddPoint(0.0, 0.3, true));
//}
