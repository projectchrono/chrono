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
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

const double rpm2rads = CH_C_PI / 30;

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

    m_max_engine_speed = rpm2rads * d["Engine"]["Maximal Engine Speed RPM"].GetDouble();

    ReadMapData(d["Engine"]["Map Full Throttle"], m_engine_map_full);
    ReadMapData(d["Engine"]["Map Zero Throttle"], m_engine_map_zero);

    // Read transmission data
    assert(d.HasMember("Transmission"));

    m_rev_gear = d["Transmission"]["Reverse Gear Ratio"].GetDouble();
    unsigned int np = d["Transmission"]["Forward Gear Ratios"].Size();
    m_fwd_gear.resize(np);
    for (unsigned int i = 0; i < np; i++)
        m_fwd_gear[i] = d["Transmission"]["Forward Gear Ratios"][i].GetDouble();

    ReadMapData(d["Transmission"]["Shift Points Map RPM"], m_shift_bands);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SimpleMapPowertrain::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = m_rev_gear;
    fwd = m_fwd_gear;
}

// -----------------------------------------------------------------------------
// Utility functions for reading (from a JSON object) and assigning (to a
// recorder function) map data specified as (x,y) pairs.
// -----------------------------------------------------------------------------
void SimpleMapPowertrain::ReadMapData(const rapidjson::Value& a, MapData& map_data) {
    assert(a.IsArray());
    map_data.m_n = a.Size();
    for (unsigned int i = 0; i < map_data.m_n; i++) {
        map_data.m_x.push_back(a[i][0u].GetDouble());
        map_data.m_y.push_back(a[i][1u].GetDouble());
    }
}

void SimpleMapPowertrain::SetMapData(const MapData& map_data, std::shared_ptr<ChFunction_Recorder>& map) {
    for (unsigned int i = 0; i < map_data.m_n; i++) {
        map->AddPoint(map_data.m_x[i], map_data.m_y[i]);
    }
}

void SimpleMapPowertrain::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    for (unsigned int i = 0; i < m_engine_map_zero.m_n; i++) {
        map0.AddPoint(rpm2rads * m_engine_map_zero.m_x[i], m_engine_map_zero.m_y[i]);
    }

    for (unsigned int i = 0; i < m_engine_map_full.m_n; i++) {
        mapF.AddPoint(rpm2rads * m_engine_map_full.m_x[i], m_engine_map_full.m_y[i]);
    }
}

void SimpleMapPowertrain::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    for (unsigned int i = 0; i < m_shift_bands.m_n; i++) {
        shift_bands.push_back(
            std::make_pair<double, double>(rpm2rads * m_shift_bands.m_x[i], rpm2rads * m_shift_bands.m_y[i]));
    }
}

}  // end namespace vehicle
}  // end namespace chrono
