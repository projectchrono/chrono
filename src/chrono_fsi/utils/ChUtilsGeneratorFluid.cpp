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
// Author: Arman Pazouki, Milad Rakhsha
// =============================================================================
//
// Utility class for generating fluid markers.
// =============================================================================

#include "chrono_fsi/utils/ChUtilsGeneratorFluid.h"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"

namespace chrono {
namespace fsi {
namespace utils {

// Kernel function
//// TODO: provide more options for the kernel function
Real W3h_Spline(Real d, Real h) {
    Real invh = 1.0 / h;
    Real q = fabs(d) * invh;
    if (q < 1) {
        return (0.25f * (INVPI * invh * invh * invh) * (cube(2 - q) - 4 * cube(1 - q)));
    }
    if (q < 2) {
        return (0.25f * (INVPI * invh * invh * invh) * cube(2 - q));
    }
    return 0;
}

// Particle mass calculator based on the initial spacing and density
Real massCalculator(Real Kernel_h, Real InitialSpacing, Real rho0) {
    int IDX = 10;
    Real sum_wij = 0;
    for (int i = -IDX; i <= IDX; i++)
        for (int j = -IDX; j <= IDX; j++)
            for (int k = -IDX; k <= IDX; k++) {
                Real3 pos = mR3(i, j, k) * InitialSpacing;
                Real W = W3h_Spline(length(pos), Kernel_h);
                sum_wij += W;
            }
    return rho0 / sum_wij;
}

// Particle number calculator based on the initial spacing and kernel length
Real IniNeiNum(Real Kernel_h, Real InitialSpacing) {
    int IDX = 10;
    int count = 0;
    for (int i = -IDX; i <= IDX; i++)
        for (int j = -IDX; j <= IDX; j++)
            for (int k = -IDX; k <= IDX; k++) {
                Real3 pos = mR3(i, j, k) * InitialSpacing;
                Real W = W3h_Spline(length(pos), Kernel_h);
                if (W > 0)
                    count++;
            }
    return count;
}

}  // end namespace utils
}  // end namespace fsi
}  // end namespace chrono
