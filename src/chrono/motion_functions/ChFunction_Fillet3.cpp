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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/core/ChLinearAlgebra.h"
#include "chrono/motion_functions/ChFunction_Fillet3.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_Fillet3)

ChFunction_Fillet3::ChFunction_Fillet3(const ChFunction_Fillet3& other) {
    end = other.end;
    y1 = other.y1;
    y2 = other.y2;
    dy1 = other.dy1;
    dy2 = other.dy2;
    c1 = other.c1;
    c2 = other.c2;
    c3 = other.c3;
    c4 = other.c4;
}

double ChFunction_Fillet3::Get_y(double x) const {
    double ret = 0;
    if (x <= 0)
        return y1;
    if (x >= end)
        return y2;
    double a = x / end;
    ret = c1 * pow(x, 3) + c2 * pow(x, 2) + c3 * x + c4;
    return ret;
}

double ChFunction_Fillet3::Get_y_dx(double x) const {
    double ret = 0;
    if (x <= 0)
        return 0;
    if (x >= end)
        return 0;
    double a = x / end;
    ret = 3 * c1 * pow(x, 2) + 2 * c2 * x + c3;
    return ret;
}

double ChFunction_Fillet3::Get_y_dxdx(double x) const {
    double ret = 0;
    if (x <= 0)
        return 0;
    if (x >= end)
        return 0;
    double a = x / end;
    ret = 6 * c1 * x + 2 * c2;
    return ret;
}

void ChFunction_Fillet3::SetupCoefficients() {
    ChMatrixDynamic<> ma(4, 4);
    ChMatrixDynamic<> mb(4, 1);
    ChMatrixDynamic<> mx(4, 1);

    mb(0, 0) = y1;
    mb(1, 0) = y2;
    mb(2, 0) = dy1;
    mb(3, 0) = dy2;

    ma(0, 3) = 1.0;

    ma(1, 0) = pow(end, 3);
    ma(1, 1) = pow(end, 2);
    ma(1, 2) = end;
    ma(1, 3) = 1.0;

    ma(2, 2) = 1.0;

    ma(3, 0) = 3 * pow(end, 2);
    ma(3, 1) = 2 * end;
    ma(3, 2) = 1.0;

    ChLinearAlgebra::Solve_LinSys(ma, &mb, &mx);

    c1 = mx(0, 0);
    c2 = mx(1, 0);
    c3 = mx(2, 0);
    c4 = mx(3, 0);
}

}  // end namespace chrono
