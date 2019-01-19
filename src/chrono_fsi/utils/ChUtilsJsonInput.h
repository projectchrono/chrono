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
// Authors: Milad Rakhsha
// =============================================================================
#ifndef CHUTILSJSONINPUT_H_
#define CHUTILSJSONINPUT_H_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChParams.cuh"
#include "chrono_fsi/custom_math.h"

namespace chrono {
namespace fsi {
namespace utils {

/// Read Chrono::FSI simulation parameters from specified JSON file.
CH_FSI_API bool ParseJSON(const char* json_file, SimParams* paramsH, Real3 Domain);

}  // namespace utils
}  // namespace fsi
}  // namespace chrono

#endif
