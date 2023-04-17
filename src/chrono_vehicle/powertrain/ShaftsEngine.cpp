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
// ChShaft-based engine model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/core/ChGlobal.h"

#include "chrono_vehicle/powertrain/ShaftsEngine.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

const double rpm2rads = CH_C_PI / 30;

ShaftsEngine::ShaftsEngine(const std::string& filename) : ChShaftsEngine("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ShaftsEngine::ShaftsEngine(const rapidjson::Document& d) : ChShaftsEngine("") {
    Create(d);
}

void ShaftsEngine::Create(const rapidjson::Document& d) {
    // Invoke base class method
    ChPart::Create(d);

    // Read engine data
    m_motorblock_inertia = d["Motor Block Inertia"].GetDouble();
    m_crankshaft_inertia = d["Crankshaft Inertia"].GetDouble();
    ReadMapData(d["Torque Map"], m_engine_torque);
    ReadMapData(d["Losses Map"], m_engine_losses);
}

// Utility functions for reading (from a JSON object) and assigning (to a recorder function) map data
void ShaftsEngine::ReadMapData(const rapidjson::Value& a, MapData& map_data) {
    assert(a.IsArray());
    map_data.m_n = a.Size();
    for (unsigned int i = 0; i < map_data.m_n; i++) {
        map_data.m_x.push_back(a[i][0u].GetDouble());
        map_data.m_y.push_back(a[i][1u].GetDouble());
    }
}

void ShaftsEngine::SetMapData(const MapData& map_data,
                              double x_factor,
                              double y_factor,
                              std::shared_ptr<ChFunction_Recorder>& map) {
    for (unsigned int i = 0; i < map_data.m_n; i++) {
        map->AddPoint(x_factor * map_data.m_x[i], y_factor * map_data.m_y[i]);
    }
}

void ShaftsEngine::SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) {
    SetMapData(m_engine_torque, rpm2rads, 1.0, map);
}

void ShaftsEngine::SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) {
    SetMapData(m_engine_losses, rpm2rads, 1.0, map);
}

}  // end namespace vehicle
}  // end namespace chrono
