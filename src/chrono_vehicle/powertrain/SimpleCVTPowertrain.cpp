// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Simplified cvt powertrain model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/core/ChGlobal.h"

#include "chrono_vehicle/powertrain/SimpleCVTPowertrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SimpleCVTPowertrain::SimpleCVTPowertrain(const std::string& filename) : ChSimpleCVTPowertrain("") {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

SimpleCVTPowertrain::SimpleCVTPowertrain(const rapidjson::Document& d) : ChSimpleCVTPowertrain("") {
    Create(d);
}

void SimpleCVTPowertrain::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read data
    m_fwd_gear_ratio = d["Forward Gear Ratio"].GetDouble();
    m_rev_gear_ratio = d["Reverse Gear Ratio"].GetDouble();

    m_max_torque = d["Maximum Engine Torque"].GetDouble();
    m_max_power = d["Maximum Engine Power"].GetDouble();
    m_max_speed = d["Maximum Engine Speed"].GetDouble();
}

void SimpleCVTPowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = m_rev_gear_ratio;
    fwd.push_back(m_fwd_gear_ratio);
}

} // end namespace vehicle
}  // end namespace chrono
