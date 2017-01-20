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
// Authors: Tyler Olsen
// =============================================================================
//
// Unit test for ChFunction_Lambda
//
// =============================================================================

#include <iostream>
#include "motion_functions/ChFunction_Lambda.h"

using namespace chrono;

int main() {
    bool passed = true, temp = true;

    // y = x*x @ x=3.0. --> y=9, dy/dx=6, d^2y/dx^2 = 2
    auto F1 = make_ChFunction_Lambda([](auto x) { return x * x; });
    temp = temp && F1.Get_y(3.0) == 9.0 && F1.Get_y_dx(3.0) == 6.0 && F1.Get_y_dxdx(3.0) == 2.0;

    if (!temp)
        std::cerr << "ChFunction_Lambda Error: Test 1 not passed\n";
    passed &= temp;
    temp = true;

    // y = exp(2*x)
    auto F2 = make_ChFunction_Lambda([](auto x) { return exp(2 * x); });
    temp = temp && F2.Get_y(2.0) == std::exp(4.0) && F2.Get_y_dx(2.0) == 2.0 * std::exp(4.0) &&
           F2.Get_y_dxdx(2.0) == 4.0 * std::exp(4.0);

    if (!temp)
        std::cerr << "ChFunction_Lambda Error: Test 2 not passed\n";
    passed &= temp;
    temp = true;

    // Allows for minute differences due to numerical errors
    double F3_tolerance = 1e-16;

    // y = cos(x), via shared pointer
    auto Fp3 = make_shared_ChFunction_Lambda([](auto x) { return cos(x); });
    temp = temp && std::abs(Fp3->Get_y(5.0) - std::cos(5.0)) < F3_tolerance &&
           std::abs(Fp3->Get_y_dx(5.0) - (-std::sin(5.0))) < F3_tolerance &&
           std::abs(Fp3->Get_y_dxdx(5.0) - (-std::cos(5.0))) < F3_tolerance;

    if (!temp)
        std::cerr << "ChFunction_Lambda Error: Test 3 not passed\n";
    passed &= temp;

    return !passed;
}
