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
// Author: Arman Pazouki
// =============================================================================
//
// Utility class for generating fluid markers.//
// =============================================================================

#ifndef CH_UTILSGENERATORBCE__CUH
#define CH_UTILSGENERATORBCE__CUH

#include "chrono_fsi/ChParams.cuh"
#include "chrono_fsi/custom_math.h"
#include <string>
#include <thrust/host_vector.h>

namespace chrono {
namespace fsi {
namespace utils {
// =============================================================================
void CreateBCE_On_Sphere(thrust::host_vector<Real3> &posRadBCE, Real rad,
                         SimParams *paramsH);

void CreateBCE_On_Cylinder(thrust::host_vector<Real3> &posRadBCE, Real cyl_rad,
                           Real cyl_h, SimParams *paramsH);

void CreateBCE_On_Box(thrust::host_vector<Real3> &posRadBCE, const Real3 &hsize,
                      int face, SimParams *paramsH);

void LoadBCE_fromFile(thrust::host_vector<Real3> &posRadBCE, // do not set the
                                                             // size here since
                                                             // you are using
                                                             // push back later
                      std::string fileName);

} // end namespace utils
} // end namespace fsi
} // end namespace chrono
#endif
