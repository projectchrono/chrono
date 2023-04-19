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

using namespace rapidjson;

namespace chrono {
namespace vehicle {

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
    m_engine_torque.Read(d["Torque Map"]);
    m_engine_losses.Read(d["Losses Map"]);
}

void ShaftsEngine::SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) {
    m_engine_torque.Set(*map, CH_C_RPM_TO_RPS, 1.0);
}

void ShaftsEngine::SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) {
    m_engine_losses.Set(*map, CH_C_RPM_TO_RPS, 1.0);
}

}  // end namespace vehicle
}  // end namespace chrono
