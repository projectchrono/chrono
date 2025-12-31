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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Demo on how to use Chrono mathematical functions
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChMatrix.h"

#include "chrono/core/ChVector3.h"
#include "chrono/core/ChQuadrature.h"
#include "chrono/utils/ChConstants.h"

using namespace chrono;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    std::cout << "\n=== Computing integrals of functions in 1D/2D/3D ===\n" << std::endl;

    // Define a y=f(x) function by inheriting ChIntegrand1D:
    class MySine1d : public ChIntegrand1D<double> {
      public:
        void Evaluate(double& result, const double x) { result = std::sin(x); }
    };

    // Create an object from the function class
    MySine1d mfx;
    // Invoke 6th order Gauss-Legendre quadrature on 0..PI interval:
    double qresult = 0;
    ChQuadrature::Integrate1D<double>(qresult, mfx, 0, CH_PI, 6);
    std::cout << "Quadrature 1d result: " << qresult << " (analytic solution: 2.0)" << std::endl;

    // Other quadrature tests, this time in 2D
    class MySine2d : public ChIntegrand2D<double> {
      public:
        void Evaluate(double& result, const double x, const double y) { result = std::sin(x); }
    };

    MySine2d mfx2d;
    qresult = 0;
    ChQuadrature::Integrate2D<double>(qresult, mfx2d, 0, CH_PI, -1, 1, 6);
    std::cout << "Quadrature 2d result: " << qresult << " (analytic solution: 4.0)" << std::endl;

    // Other quadrature tests, this time with vector function (that is, integrates 2x1 matrix)
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
    std::cout << "Quadrature 2d matrix result: " << resultM << " (analytic solution: 2.25, 4.5)" << std::endl;

    return 0;
}
