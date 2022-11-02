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
// Rigid suspension with pinned axle constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/suspension/RigidPinnedAxle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a rigid suspension using data from the specified JSON file.
// -----------------------------------------------------------------------------
RigidPinnedAxle::RigidPinnedAxle(const std::string& filename) : ChRigidPinnedAxle("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

RigidPinnedAxle::RigidPinnedAxle(const rapidjson::Document& d) : ChRigidPinnedAxle("") {
    Create(d);
}

// -----------------------------------------------------------------------------
// Worker function for creating a RigidPinnedAxle suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void RigidPinnedAxle::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read Spindle data
    assert(d.HasMember("Spindle"));
    assert(d["Spindle"].IsObject());

    m_spindleMass = d["Spindle"]["Mass"].GetDouble();
    m_points[SPINDLE] = ReadVectorJSON(d["Spindle"]["COM"]);
    m_spindleInertia = ReadVectorJSON(d["Spindle"]["Inertia"]);
    m_spindleRadius = d["Spindle"]["Radius"].GetDouble();
    m_spindleWidth = d["Spindle"]["Width"].GetDouble();

    // Read central tube data
    assert(d.HasMember("Tube"));
    assert(d["Tube"].IsObject());

    m_axleTubeMass = d["Tube"]["Mass"].GetDouble();
    m_axleTubeCOM = ReadVectorJSON(d["Tube"]["COM"]);
    m_axleTubeInertia = ReadVectorJSON(d["Tube"]["Inertia"]);
    m_axleTubeRadius = d["Tube"]["Radius"].GetDouble();
    m_axlePinLoc = ReadVectorJSON(d["Tube"]["Pin Location"]);

    // Read axle inertia
    assert(d.HasMember("Axle"));
    assert(d["Axle"].IsObject());

    m_axleInertia = d["Axle"]["Inertia"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
