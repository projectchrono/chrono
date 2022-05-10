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
// Articulated chassis connector model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/chassis/ChassisConnectorArticulated.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

ChassisConnectorArticulated::ChassisConnectorArticulated(const std::string& filename)
    : ChChassisConnectorArticulated("") {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ChassisConnectorArticulated::ChassisConnectorArticulated(const rapidjson::Document& d)
    : ChChassisConnectorArticulated("") {
    Create(d);
}

void ChassisConnectorArticulated::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read maximum steering angle (assumed to be given in degrees)
    m_maxangle = d["Maximum Steering Angle"].GetDouble() * CH_C_DEG_TO_RAD;
}

}  // end namespace vehicle
}  // end namespace chrono
