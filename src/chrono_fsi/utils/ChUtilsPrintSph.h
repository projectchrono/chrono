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
// Utility function to print the save fluid, bce, and boundary data into file
// =============================================================================
#ifndef CHUTILSPRINTSPH_H
#define CHUTILSPRINTSPH_H
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/custom_math.h"
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

struct SimParams;

namespace chrono {
namespace fsi {
namespace utils {

/// function to save the fluid data into file
///
/// When called, this function creates three files to write fluid,
/// fluid-boundary
/// and BCE markers data into file
CH_FSI_API void PrintToFile(const thrust::device_vector<Real3> &posRadD,
                            const thrust::device_vector<Real3> &velMasD,
                            const thrust::device_vector<Real4> &rhoPresMuD,
                            const thrust::host_vector<int4> &referenceArray,
                            const std::string &out_dir,
			    bool printToParaview=false);


} // end namespace utils
} // end namespace fsi
} // end namespace chrono

#endif
