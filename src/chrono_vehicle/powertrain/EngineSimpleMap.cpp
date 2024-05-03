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

#include "chrono_vehicle/powertrain/EngineSimpleMap.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

EngineSimpleMap::EngineSimpleMap(const std::string& filename) : ChEngineSimpleMap("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

EngineSimpleMap::EngineSimpleMap(const rapidjson::Document& d) : ChEngineSimpleMap("") {
    Create(d);
}

void EngineSimpleMap::Create(const rapidjson::Document& d) {
    // Invoke base class method
    ChPart::Create(d);

    // Read engine data
    m_max_engine_speed = CH_RPM_TO_RAD_S * d["Maximal Engine Speed RPM"].GetDouble();

    m_engine_map_full.Read(d["Map Full Throttle"]);
    m_engine_map_zero.Read(d["Map Zero Throttle"]);
}

void EngineSimpleMap::SetEngineTorqueMaps(ChFunctionInterp& map0, ChFunctionInterp& mapF) {
    m_engine_map_zero.Set(map0, CH_RPM_TO_RAD_S, 1.0);
    m_engine_map_full.Set(mapF, CH_RPM_TO_RAD_S, 1.0);
}

}  // end namespace vehicle
}  // end namespace chrono
