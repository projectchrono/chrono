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
#include "chrono/ChConfig.h"
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/custom_math.h"
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;
struct SimParams;

namespace chrono {
namespace fsi {
namespace utils {

CH_FSI_API bool ParseJSON(const char* json_file, SimParams* paramsH, Real3 Domain);
CH_FSI_API Real3 LoadVectorJSON(const Value& a);
CH_FSI_API void InvalidArg(std::string arg);

}  // namespace utils
}  // namespace fsi
}  // namespace chrono

#endif /* CHUTILSJSON_H_ */
