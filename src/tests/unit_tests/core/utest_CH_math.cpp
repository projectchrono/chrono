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
// Tests for miscellaneous math support.
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChQuadrature.h"
#include "chrono/utils/ChConstants.h"

#include "gtest/gtest.h"

using std::cout;
using std::endl;
using namespace chrono;

const double ABS_ERR = 1e-8;

TEST(MathTest, quadrature) {
    class MySine1d : public ChIntegrand1D<double> {
      public:
        void Evaluate(double& result, const double x) { result = std::sin(x); }
    };

    MySine1d mfx;
    double qresult = 0;
    ChQuadrature::Integrate1D<double>(qresult, mfx, 0, CH_PI, 6);  // 6th order Gauss-Legendre quadrature on [0,pi]
    cout << "Quadrature 1d result: " << qresult << " (analytic solution: 2.0) \n";
    ASSERT_NEAR(qresult, 2.0, ABS_ERR);

    class MySine2d : public ChIntegrand2D<double> {
      public:
        void Evaluate(double& result, const double x, const double y) { result = std::sin(x); }
    };

    MySine2d mfx2d;
    qresult = 0;
    ChQuadrature::Integrate2D<double>(qresult, mfx2d, 0, CH_PI, -1, 1, 6);
    cout << "Quadrature 2d result: " << qresult << " (analytic solution: 4.0) \n";
    ASSERT_NEAR(qresult, 4.0, ABS_ERR);

    class MySine2dM : public ChIntegrand2D<ChMatrixNM<double, 1, 2>> {
      public:
        void Evaluate(ChMatrixNM<double, 1, 2>& result, const double x, const double y) {
            result(0, 0) = x * y;
            result(0, 1) = 0.5 * y * y;
        }
    };

    MySine2dM mfx2dM;
    ChMatrixNM<double, 1, 2> resultM;
    resultM.setZero();
    ChQuadrature::Integrate2D<ChMatrixNM<double, 1, 2>>(resultM, mfx2dM, 0, 1, 0, 3, 6);
    cout << "Quadrature 2d matrix result: " << resultM << " (analytic solution: 2.25, 4.5) \n";
    ASSERT_NEAR(resultM(0, 0), 2.25, ABS_ERR);
    ASSERT_NEAR(resultM(0, 1), 4.5, ABS_ERR);
}
