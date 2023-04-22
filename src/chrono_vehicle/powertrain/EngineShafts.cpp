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

#include "chrono_vehicle/powertrain/EngineShafts.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

EngineShafts::EngineShafts(const std::string& filename) : ChEngineShafts("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

EngineShafts::EngineShafts(const rapidjson::Document& d) : ChEngineShafts("") {
    Create(d);
}

void EngineShafts::Create(const rapidjson::Document& d) {
    // Invoke base class method
    ChPart::Create(d);

    // Read engine data
    m_motorblock_inertia = d["Motor Block Inertia"].GetDouble();
    m_motorshaft_inertia = d["Motorshaft Inertia"].GetDouble();
    m_engine_torque.Read(d["Torque Map"]);
    m_engine_losses.Read(d["Losses Map"]);
}

void EngineShafts::SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) {
    m_engine_torque.Set(*map, CH_C_RPM_TO_RPS, 1.0);
}

void EngineShafts::SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) {
    m_engine_losses.Set(*map, CH_C_RPM_TO_RPS, 1.0);
}

}  // end namespace vehicle
}  // end namespace chrono
