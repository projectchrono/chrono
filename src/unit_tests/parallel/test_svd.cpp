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
// Authors: Hammad Mazhar
// =============================================================================
//
// ChronoParallel unit test for MPR collision detection
// =============================================================================

#include <stdio.h>
#include <vector>
#include <cmath>

#include "unit_testing.h"

#include "chrono_parallel/math/mat33.h"
#include "chrono_parallel/math/svd.h"

using namespace chrono;

int main(int argc, char* argv[]) {
    const Mat33 A(1, 0, 5, 2, 1, 6, 3, 4, 0);
    Mat33 U, V;
    real3 SV;
    chrono::SVD(A, U, SV, V);

    WeakEqual(SV, real3(8.2788392304997842851, 4.8435729666086053555, 0.024938177079601287432), 2e-5);

    return 0;
}
