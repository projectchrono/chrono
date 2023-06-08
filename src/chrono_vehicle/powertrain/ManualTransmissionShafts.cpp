// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Marcel Offermans
// =============================================================================

#include "chrono/core/ChGlobal.h"

#include "chrono_vehicle/powertrain/ManualTransmissionShafts.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

ManualTransmissionShafts::ManualTransmissionShafts(const std::string& filename) : ChManualTransmissionShafts("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ManualTransmissionShafts::ManualTransmissionShafts(const rapidjson::Document& d) : ChManualTransmissionShafts("") {
    Create(d);
}

void ManualTransmissionShafts::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    m_transmissionblock_inertia = d["Transmission Block Inertia"].GetDouble();
    m_ingear_shaft_inertia = d["Input Shaft Inertia"].GetDouble();
    m_motorshaft_inertia = d["Motorshaft Inertia"].GetDouble();
    m_driveshaft_inertia = d["Driveshaft Inertia"].GetDouble();

    // Read transmission data
    assert(d.HasMember("Gear Box"));

    m_rev_gear = d["Gear Box"]["Reverse Gear Ratio"].GetDouble();
    unsigned int np = d["Gear Box"]["Forward Gear Ratios"].Size();
    m_fwd_gear.resize(np);
    for (unsigned int i = 0; i < np; i++)
        m_fwd_gear[i] = d["Gear Box"]["Forward Gear Ratios"][i].GetDouble();

    // Read clutch data
    if (d.HasMember("Clutch Torque Limit")) {
        m_clutch_torque_limit = d["Clutch Torque Limit"].GetDouble();
    } else {
        m_clutch_torque_limit = 500;  // default value if not specified
    }
}

// -----------------------------------------------------------------------------

void ManualTransmissionShafts::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = m_rev_gear;
    fwd = m_fwd_gear;
}
}  // end namespace vehicle
}  // end namespace chrono
