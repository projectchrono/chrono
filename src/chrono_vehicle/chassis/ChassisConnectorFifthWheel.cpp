// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// FifthWheel chassis connector model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/chassis/ChassisConnectorFifthWheel.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

ChassisConnectorFifthWheel::ChassisConnectorFifthWheel(const std::string& filename) : ChChassisConnectorFifthWheel("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

ChassisConnectorFifthWheel::ChassisConnectorFifthWheel(const rapidjson::Document& d)
    : ChChassisConnectorFifthWheel("") {
    Create(d);
}

void ChassisConnectorFifthWheel::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);
}

}  // end namespace vehicle
}  // end namespace chrono
