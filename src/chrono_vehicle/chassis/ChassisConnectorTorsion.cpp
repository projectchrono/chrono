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
// Torsion chassis connector model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/chassis/ChassisConnectorTorsion.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

ChassisConnectorTorsion::ChassisConnectorTorsion(const std::string& filename) : ChChassisConnectorTorsion("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

ChassisConnectorTorsion::ChassisConnectorTorsion(const rapidjson::Document& d) : ChChassisConnectorTorsion("") {
    Create(d);
}

void ChassisConnectorTorsion::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read torsional stiffness
    m_torsion_stiffness = d["Torsional Stiffness"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
