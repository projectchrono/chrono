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

#ifndef CHUTILSJSON_H_
#define CHUTILSJSON_H_
#include <memory>
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/math/custom_math.h"
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"
using namespace rapidjson;

namespace chrono {
namespace fsi {
struct SimParams;

namespace utils {

CH_FSI_API bool ParseJSON(const std::string& json_file, std::shared_ptr<fsi::SimParams> paramsH, Real3 Domain);
CH_FSI_API void PrepareOutputDir(std::shared_ptr<fsi::SimParams> paramsH,
                                 std::string& demo_dir,
                                 std::string out_dir,
                                 std::string jsonFile);

Real3 LoadVectorJSON(const Value& a);

void InvalidArg(std::string arg);

}  // namespace utils
}  // namespace fsi
}  // namespace chrono

#endif /* CHUTILSJSON_H_ */
