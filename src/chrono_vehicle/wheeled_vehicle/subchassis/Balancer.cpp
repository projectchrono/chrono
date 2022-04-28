// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
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
// Balancer subchassis system constructed with data from file.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/subchassis/Balancer.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

Balancer::Balancer(const std::string& filename) : ChBalancer(""), m_bushingData(nullptr) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

Balancer::Balancer(const rapidjson::Document& d) : ChBalancer(""), m_bushingData(nullptr) {
    Create(d);
}

Balancer::~Balancer() {}

// -----------------------------------------------------------------------------

void Balancer::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read beam data
    assert(d.HasMember("Beam"));
    assert(d["Beam"].IsObject());

    m_beam_mass = d["Beam"]["Mass"].GetDouble();
    m_points[BEAM] = ReadVectorJSON(d["Beam"]["COM"]);
    m_beam_inertia = ReadVectorJSON(d["Beam"]["Inertia"]);
    m_beam_dimensions = ReadVectorJSON(d["Beam"]["Size"]);

    // Read joint data
    m_points[REVOLUTE] = ReadVectorJSON(d["Beam"]["Pin Location"]);
    if (d["Beam"].HasMember("Bushing Data")) {
        // Read bushing properties
        m_bushingData = ReadBushingDataJSON(d["Beam"]["Bushing Data"]);
    } else {
        // Read pitch limits
        m_beam_max_pitch = d["Maximum Pitch"].GetDouble();
    }
}


}  // end namespace vehicle
}  // end namespace chrono
