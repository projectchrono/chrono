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
// Authors: Radu Serban
// =============================================================================
//
// Utility functions for parsing JSON files.
//
// =============================================================================

#ifndef CH_JSON_UTILS_H
#define CH_JSON_UTILS_H

#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/assets/ChColor.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// Load and return a ChVector from the specified JSON array
CH_VEHICLE_API ChVector<> LoadVectorJSON(const rapidjson::Value& a);

///  Load and return a ChQuaternion from the specified JSON array
CH_VEHICLE_API ChQuaternion<> LoadQuaternionJSON(const rapidjson::Value& a);

///  Load and return a ChColor from the specified JSON array
CH_VEHICLE_API ChColor LoadColorJSON(const rapidjson::Value& a);

}  // end namespace vehicle
}  // end namespace chrono

#endif
