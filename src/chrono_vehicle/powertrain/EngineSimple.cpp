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
// Simplified engine model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/powertrain/EngineSimple.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

EngineSimple::EngineSimple(const std::string& filename) : ChEngineSimple("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

EngineSimple::EngineSimple(const rapidjson::Document& d) : ChEngineSimple("") {
    Create(d);
}

void EngineSimple::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read data
    m_max_torque = d["Maximum Engine Torque"].GetDouble();
    m_max_power = d["Maximum Engine Power"].GetDouble();
    m_max_speed = d["Maximum Engine Speed"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
