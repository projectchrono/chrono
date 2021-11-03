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
// Author: Milad Rakhsha
// =============================================================================
//
// Utility function to read input from JSON files
// =============================================================================

#ifndef CH_FSI_UTILS_JSON_H
#define CH_FSI_UTILS_JSON_H

#include <memory>
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/math/custom_math.h"
#include "chrono_fsi/physics/ChParams.h"
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"
using namespace rapidjson;

namespace chrono {
namespace fsi {

// Forward declaration
struct SimParams;

namespace utils {

/// @addtogroup fsi_utils
/// @{

/// Parse FSI parameters from the specified JSON file.
CH_FSI_API bool ParseJSON(const std::string& json_file, std::shared_ptr<fsi::SimParams> paramsH, Real3 Domain);

/// Create outut directories.
CH_FSI_API void PrepareOutputDir(std::shared_ptr<fsi::SimParams> paramsH,
                                 std::string& demo_dir,
                                 std::string out_dir,
                                 std::string jsonFile);

/// Load a Real3 vector from the given JSON value.
Real3 LoadVectorJSON(const Value& a);

/// Issue warning about an invalid argument (write to stdout).
void InvalidArg(std::string arg);

/// @} fsi_utils

}  // namespace utils
}  // namespace fsi
}  // namespace chrono

#endif
