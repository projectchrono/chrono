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
// Wheeled vehicle shafts-based brake model constructed with data from file
// (JSON format).
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/brake/BrakeShafts.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
BrakeShafts::BrakeShafts(const std::string& filename) : ChBrakeShafts("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

BrakeShafts::BrakeShafts(const rapidjson::Document& d) : ChBrakeShafts("") {
    Create(d);
}

void BrakeShafts::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read shaft inertia
    m_shaft_inertia = d["Shaft Inertia"].GetDouble();

    // Read maximum braking torque
    m_maxtorque = d["Maximum Torque"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
