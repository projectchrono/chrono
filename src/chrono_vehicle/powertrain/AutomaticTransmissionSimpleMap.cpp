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
// Authors: Radu Serban
// =============================================================================
//
// JSON specification of an automatic transmission model template based on a
// simple gear-shifting model.
//
// =============================================================================

#include "chrono_vehicle/powertrain/AutomaticTransmissionSimpleMap.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

AutomaticTransmissionSimpleMap::AutomaticTransmissionSimpleMap(const std::string& filename)
    : ChAutomaticTransmissionSimpleMap("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

AutomaticTransmissionSimpleMap::AutomaticTransmissionSimpleMap(const rapidjson::Document& d)
    : ChAutomaticTransmissionSimpleMap("") {
    Create(d);
}

void AutomaticTransmissionSimpleMap::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read transmission data
    m_rev_gear = d["Gear Box"]["Reverse Gear Ratio"].GetDouble();
    unsigned int np = d["Gear Box"]["Forward Gear Ratios"].Size();
    m_fwd_gear.resize(np);
    for (unsigned int i = 0; i < np; i++)
        m_fwd_gear[i] = d["Gear Box"]["Forward Gear Ratios"][i].GetDouble();

    m_shift_bands.Read(d["Gear Box"]["Shift Points Map RPM"]);
}

// -----------------------------------------------------------------------------

void AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = m_rev_gear;
    fwd = m_fwd_gear;
}

void AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    m_shift_bands.Set(shift_bands, CH_C_RPM_TO_RPS, CH_C_RPM_TO_RPS);
}

}  // end namespace vehicle
}  // end namespace chrono
