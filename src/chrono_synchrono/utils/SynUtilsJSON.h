// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// A set of helper functions for pulling data from json structures into Chrono
// or SynChrono objects
//
// =============================================================================

#ifndef SYN_JSON_UTILS_H
#define SYN_JSON_UTILS_H

#include "chrono_synchrono/SynApi.h"

#include "chrono/core/ChCoordsys.h"

#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_utils
/// @{

// -----------------------------------------------------------------------------

/// Load and return a RapidJSON document from the specified file.
/// A Null document is returned if the file cannot be opened.
SYN_API rapidjson::Document ReadFileJSON(const std::string& filename);

// -----------------------------------------------------------------------------

/// Load and return a ChVector from the specified JSON array
SYN_API ChVector<> ReadVectorJSON(const rapidjson::Value& a);

/// Load and return a ChQuaternion from the specified JSON array
SYN_API ChQuaternion<> ReadQuaternionJSON(const rapidjson::Value& a);

/// Load and return a ChCoordsys from the specified JSON values
SYN_API ChCoordsys<> ReadCoordsysJSON(const rapidjson::Value& a, const rapidjson::Value& b);

// -----------------------------------------------------------------------------

///  Load and return the visualization type from the specified JSON file.
SYN_API vehicle::VisualizationType ReadVisualizationTypeJSON(const std::string& type);

// -----------------------------------------------------------------------------

/// @} synchrono_utils

}  // namespace synchrono
}  // namespace chrono

#endif
