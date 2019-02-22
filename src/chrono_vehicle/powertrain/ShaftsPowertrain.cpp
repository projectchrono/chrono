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
// ChShaft-based powertrain model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/physics/ChGlobal.h"

#include "chrono_vehicle/powertrain/ShaftsPowertrain.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Constructor a shafts powertrain using data from the specified JSON file.
// -----------------------------------------------------------------------------
ShaftsPowertrain::ShaftsPowertrain(const std::string& filename) : ChShaftsPowertrain("") {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ShaftsPowertrain::ShaftsPowertrain(const rapidjson::Document& d) : ChShaftsPowertrain("") {
    Create(d);
}

// -----------------------------------------------------------------------------
// Worker function for creating a ShaftsPowertrain powertrain using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void ShaftsPowertrain::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read engine data
    assert(d.HasMember("Engine"));

    m_motorblock_inertia = d["Engine"]["Motor Block Inertia"].GetDouble();
    m_crankshaft_inertia = d["Engine"]["Crankshaft Inertia"].GetDouble();
    ReadMapData(d["Engine"]["Torque Map"], m_engine_torque);
    ReadMapData(d["Engine"]["Losses Map"], m_engine_losses);

    // Read transmission data
    assert(d.HasMember("Transmission"));

    m_ingear_shaft_inertia = d["Transmission"]["Input Shaft Inertia"].GetDouble();
    m_rev_gear = d["Transmission"]["Reverse Gear Ratio"].GetDouble();
    unsigned int np = d["Transmission"]["Forward Gear Ratios"].Size();
    m_fwd_gear.resize(np);
    for (unsigned int i = 0; i < np; i++)
        m_fwd_gear[i] = d["Transmission"]["Forward Gear Ratios"][i].GetDouble();

    // Read torque converter data
    assert(d.HasMember("Torque Converter"));
    ReadMapData(d["Torque Converter"]["Capacity Factor Map"], m_tc_capacity_factor);
    ReadMapData(d["Torque Converter"]["Torque Ratio Map"], m_tc_torque_ratio);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ShaftsPowertrain::SetGearRatios(std::vector<double>& gear_ratios) {
    gear_ratios.push_back(m_rev_gear);
    for (unsigned int i = 0; i < m_fwd_gear.size(); i++)
        gear_ratios.push_back(m_fwd_gear[i]);
}

// -----------------------------------------------------------------------------
// Utility functions for reading (from a JSON object) and assigning (to a
// recorder function) map data specified as (x,y) pairs.
// -----------------------------------------------------------------------------
void ShaftsPowertrain::ReadMapData(const rapidjson::Value& a, MapData& map_data) {
    assert(a.IsArray());
    map_data.m_n = a.Size();
    for (unsigned int i = 0; i < map_data.m_n; i++) {
        map_data.m_x.push_back(a[i][0u].GetDouble());
        map_data.m_y.push_back(a[i][1u].GetDouble());
    }
}

void ShaftsPowertrain::SetMapData(const MapData& map_data, std::shared_ptr<ChFunction_Recorder>& map) {
    for (unsigned int i = 0; i < map_data.m_n; i++) {
        map->AddPoint(map_data.m_x[i], map_data.m_y[i]);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
