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
// Authors: Daniel Melanz
// =============================================================================
//
// MacPherson strut suspension constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/suspension/MacPhersonStrut.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a MacPherson strut suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
MacPhersonStrut::MacPhersonStrut(const std::string& filename)
    : ChMacPhersonStrut(""), m_springForceCB(nullptr), m_shockForceCB(nullptr), m_LCABushingData(nullptr) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

MacPhersonStrut::MacPhersonStrut(const rapidjson::Document& d)
    : ChMacPhersonStrut(""), m_springForceCB(nullptr), m_shockForceCB(nullptr), m_LCABushingData(nullptr) {
    Create(d);
}

MacPhersonStrut::~MacPhersonStrut() {}

// -----------------------------------------------------------------------------
// Worker function for creating a MacPherson strut suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void MacPhersonStrut::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    if (d.HasMember("Camber Angle (deg)"))
        m_camber_angle = d["Camber Angle (deg)"].GetDouble() * CH_C_DEG_TO_RAD;
    else
        m_camber_angle = 0;

    if (d.HasMember("Toe Angle (deg)"))
        m_toe_angle = d["Toe Angle (deg)"].GetDouble() * CH_C_DEG_TO_RAD;
    else
        m_toe_angle = 0;

    // Read Spindle data
    assert(d.HasMember("Spindle"));
    assert(d["Spindle"].IsObject());

    m_spindleMass = d["Spindle"]["Mass"].GetDouble();
    m_points[SPINDLE] = ReadVectorJSON(d["Spindle"]["COM"]);
    m_spindleInertia = ReadVectorJSON(d["Spindle"]["Inertia"]);
    m_spindleRadius = d["Spindle"]["Radius"].GetDouble();
    m_spindleWidth = d["Spindle"]["Width"].GetDouble();

    // Read Upright data
    assert(d.HasMember("Upright"));
    assert(d["Upright"].IsObject());

    m_uprightMass = d["Upright"]["Mass"].GetDouble();
    m_points[UPRIGHT] = ReadVectorJSON(d["Upright"]["COM"]);
    m_uprightInertia = ReadVectorJSON(d["Upright"]["Inertia"]);
    m_uprightRadius = d["Upright"]["Radius"].GetDouble();

    // Read LCA data
    assert(d.HasMember("Control Arm"));
    assert(d["Control Arm"].IsObject());

    m_LCAMass = d["Control Arm"]["Mass"].GetDouble();
    m_points[LCA_CM] = ReadVectorJSON(d["Control Arm"]["COM"]);
    m_LCAInertia = ReadVectorJSON(d["Control Arm"]["Inertia"]);
    m_LCARadius = d["Control Arm"]["Radius"].GetDouble();
    m_points[LCA_F] = ReadVectorJSON(d["Control Arm"]["Location Chassis Front"]);
    m_points[LCA_B] = ReadVectorJSON(d["Control Arm"]["Location Chassis Back"]);
    m_points[LCA_U] = ReadVectorJSON(d["Control Arm"]["Location Upright"]);
    if (d["Control Arm"].HasMember("Bushing Data")) {
        m_LCABushingData = ReadBushingDataJSON(d["Control Arm"]["Bushing Data"]);
    }

    // Read strut data
    assert(d.HasMember("Strut"));
    assert(d["Strut"].IsObject());

    m_strutMass = d["Strut"]["Mass"].GetDouble();
    m_strutInertia = ReadVectorJSON(d["Strut"]["Inertia"]);
    m_strutRadius = d["Strut"]["Radius"].GetDouble();

    // Read Tierod data
    assert(d.HasMember("Tierod"));
    assert(d["Tierod"].IsObject());

    if (d["Tierod"].HasMember("Mass")) {
        assert(d["Tierod"].HasMember("Inertia"));
        assert(d["Tierod"].HasMember("Radius"));
        m_tierodMass = d["Tierod"]["Mass"].GetDouble();
        m_tierodRadius = d["Tierod"]["Radius"].GetDouble();
        m_tierodInertia = ReadVectorJSON(d["Tierod"]["Inertia"]);
        m_use_tierod_bodies = true;
        if (d["Tierod"].HasMember("Bushing Data")) {
            m_tierodBushingData = ReadBushingDataJSON(d["Tierod"]["Bushing Data"]);
        }
    } else {
        m_tierodMass = 0;
        m_tierodRadius = 0;
        m_tierodInertia = ChVector<>(0);
        m_use_tierod_bodies = false;
    }

    m_points[TIEROD_C] = ReadVectorJSON(d["Tierod"]["Location Chassis"]);
    m_points[TIEROD_U] = ReadVectorJSON(d["Tierod"]["Location Upright"]);

    // Read spring data and create force callback
    assert(d.HasMember("Spring"));
    assert(d["Spring"].IsObject());
    m_points[SPRING_C] = ReadVectorJSON(d["Spring"]["Location Chassis"]);
    m_points[SPRING_U] = ReadVectorJSON(d["Spring"]["Location Upright"]);
    m_springForceCB = ReadTSDAFunctorJSON(d["Spring"], m_springRestLength);

    // Read shock data and create force callback
    assert(d.HasMember("Shock"));
    assert(d["Shock"].IsObject());
    m_points[SHOCK_C] = ReadVectorJSON(d["Shock"]["Location Chassis"]);
    m_points[SHOCK_U] = ReadVectorJSON(d["Shock"]["Location Upright"]);
    m_shockForceCB = ReadTSDAFunctorJSON(d["Shock"], m_shockRestLength);

    // Read axle inertia
    assert(d.HasMember("Axle"));
    assert(d["Axle"].IsObject());
    m_axleInertia = d["Axle"]["Inertia"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
