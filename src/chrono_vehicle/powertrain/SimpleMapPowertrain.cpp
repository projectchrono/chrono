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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// JSON-based specification of a simple map-based powertrain model
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
// - configured via JSON file
//
// =============================================================================

#include "chrono_vehicle/powertrain/SimpleMapPowertrain.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Constructor for a powertrain using data from the specified JSON file.
// -----------------------------------------------------------------------------
SimpleMapPowertrain::SimpleMapPowertrain(const std::string& filename) : ChSimpleMapPowertrain("") {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

SimpleMapPowertrain::SimpleMapPowertrain(const rapidjson::Document& d) : ChSimpleMapPowertrain("") {
    Create(d);
}

// -----------------------------------------------------------------------------
// Worker function for creating a SimpleMapPowertrain using data in the given
// RapidJSON document.
// -----------------------------------------------------------------------------
void SimpleMapPowertrain::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read engine data
    assert(d.HasMember("Engine"));

    m_max_engine_speed = CH_C_RPM_TO_RPS * d["Engine"]["Maximal Engine Speed RPM"].GetDouble();

    m_engine_map_full.Read(d["Engine"]["Map Full Throttle"]);
    m_engine_map_zero.Read(d["Engine"]["Map Zero Throttle"]);

    // Read transmission data
    assert(d.HasMember("Transmission"));

    m_rev_gear = d["Transmission"]["Reverse Gear Ratio"].GetDouble();
    unsigned int np = d["Transmission"]["Forward Gear Ratios"].Size();
    m_fwd_gear.resize(np);
    for (unsigned int i = 0; i < np; i++)
        m_fwd_gear[i] = d["Transmission"]["Forward Gear Ratios"][i].GetDouble();

    m_shift_bands.Read(d["Transmission"]["Shift Points Map RPM"]);
}

// -----------------------------------------------------------------------------

void SimpleMapPowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = m_rev_gear;
    fwd = m_fwd_gear;
}

void SimpleMapPowertrain::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    m_engine_map_zero.Set(map0, CH_C_RPM_TO_RPS, 1.0);
    m_engine_map_full.Set(mapF, CH_C_RPM_TO_RPS, 1.0);
}

void SimpleMapPowertrain::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    m_shift_bands.Set(shift_bands, CH_C_RPM_TO_RPS, CH_C_RPM_TO_RPS);
}

}  // end namespace vehicle
}  // end namespace chrono
