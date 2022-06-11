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
// Hitch chassis connector model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/chassis/ChassisConnectorHitch.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

ChassisConnectorHitch::ChassisConnectorHitch(const std::string& filename) : ChChassisConnectorHitch("") {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ChassisConnectorHitch::ChassisConnectorHitch(const rapidjson::Document& d) : ChChassisConnectorHitch("") {
    Create(d);
}

void ChassisConnectorHitch::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);
}

}  // end namespace vehicle
}  // end namespace chrono
