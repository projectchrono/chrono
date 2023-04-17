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
// JSON specification of a simple engine model based on torque-speed engine maps
//
// =============================================================================

#include "chrono_vehicle/powertrain/SimpleMapEngine.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

const double rpm2rads = CH_C_PI / 30;

SimpleMapEngine::SimpleMapEngine(const std::string& filename) : ChSimpleMapEngine("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

SimpleMapEngine::SimpleMapEngine(const rapidjson::Document& d) : ChSimpleMapEngine("") {
    Create(d);
}

void SimpleMapEngine::Create(const rapidjson::Document& d) {
    // Invoke base class method
    ChPart::Create(d);

    // Read engine data
    m_max_engine_speed = rpm2rads * d["Maximal Engine Speed RPM"].GetDouble();

    ReadMapData(d["Map Full Throttle"], m_engine_map_full);
    ReadMapData(d["Map Zero Throttle"], m_engine_map_zero);
}

// Utility functions for reading (from a JSON object) and assigning (to a recorder function) map data
void SimpleMapEngine::ReadMapData(const rapidjson::Value& a, MapData& map_data) {
    assert(a.IsArray());
    map_data.m_n = a.Size();
    for (unsigned int i = 0; i < map_data.m_n; i++) {
        map_data.m_x.push_back(a[i][0u].GetDouble());
        map_data.m_y.push_back(a[i][1u].GetDouble());
    }
}

void SimpleMapEngine::SetMapData(const MapData& map_data, std::shared_ptr<ChFunction_Recorder>& map) {
    for (unsigned int i = 0; i < map_data.m_n; i++) {
        map->AddPoint(map_data.m_x[i], map_data.m_y[i]);
    }
}

void SimpleMapEngine::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    for (unsigned int i = 0; i < m_engine_map_zero.m_n; i++) {
        map0.AddPoint(rpm2rads * m_engine_map_zero.m_x[i], m_engine_map_zero.m_y[i]);
    }

    for (unsigned int i = 0; i < m_engine_map_full.m_n; i++) {
        mapF.AddPoint(rpm2rads * m_engine_map_full.m_x[i], m_engine_map_full.m_y[i]);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
