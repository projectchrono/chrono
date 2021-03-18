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
// Simplified powertrain model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/core/ChGlobal.h"

#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SimplePowertrain::SimplePowertrain(const std::string& filename) : ChSimplePowertrain("") {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

SimplePowertrain::SimplePowertrain(const rapidjson::Document& d) : ChSimplePowertrain("") {
    Create(d);
}

void SimplePowertrain::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read data
    m_fwd_gear_ratio = d["Forward Gear Ratio"].GetDouble();
    m_rev_gear_ratio = d["Reverse Gear Ratio"].GetDouble();

    m_max_torque = d["Maximum Engine Torque"].GetDouble();
    m_max_speed = d["Maximum Engine Speed"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
