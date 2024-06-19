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
// Authors: Rainer Gericke
// =============================================================================
//
// DeDion axle suspension constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/suspension/DeDionAxle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a solid axle suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
DeDionAxle::DeDionAxle(const std::string& filename) : ChDeDionAxle(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

DeDionAxle::DeDionAxle(const rapidjson::Document& d) : ChDeDionAxle(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    Create(d);
}

DeDionAxle::~DeDionAxle() {}

// -----------------------------------------------------------------------------
// Worker function for creating a DeDionAxle suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void DeDionAxle::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    if (d.HasMember("Camber Angle (deg)"))
        m_camber_angle = d["Camber Angle (deg)"].GetDouble() * CH_DEG_TO_RAD;
    else
        m_camber_angle = 0;

    if (d.HasMember("Toe Angle (deg)"))
        m_toe_angle = d["Toe Angle (deg)"].GetDouble() * CH_DEG_TO_RAD;
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

    // Read Axle Tube data
    assert(d.HasMember("Axle Tube"));
    assert(d["Axle Tube"].IsObject());

    m_axleTubeMass = d["Axle Tube"]["Mass"].GetDouble();
    m_axleTubeCOM = ReadVectorJSON(d["Axle Tube"]["COM"]);
    m_axleTubeInertia = ReadVectorJSON(d["Axle Tube"]["Inertia"]);
    m_axleTubeRadius = d["Axle Tube"]["Radius"].GetDouble();
    m_points[AXLE_C] = ReadVectorJSON(d["Axle Tube"]["Location Connector Chassis"]);
    // m_points[STABI_CON] = ReadVectorJSON(d["Axle Tube"]["Location Stabilizer"]);
    m_points[STABI_CON] = {0, 0, 0};

    // Read spring data and create force callback
    assert(d.HasMember("Spring"));
    assert(d["Spring"].IsObject());
    m_points[SPRING_C] = ReadVectorJSON(d["Spring"]["Location Chassis"]);
    m_points[SPRING_A] = ReadVectorJSON(d["Spring"]["Location Axle"]);
    m_springForceCB = ReadTSDAFunctorJSON(d["Spring"], m_springRestLength);

    // Read shock data and create force callback
    assert(d.HasMember("Shock"));
    assert(d["Shock"].IsObject());
    m_points[SHOCK_C] = ReadVectorJSON(d["Shock"]["Location Chassis"]);
    m_points[SHOCK_A] = ReadVectorJSON(d["Shock"]["Location Axle"]);
    m_shockForceCB = ReadTSDAFunctorJSON(d["Shock"], m_shockRestLength);

    // Read Watt Mechanism Elements
    assert(d.HasMember("Watt Mechanism"));
    assert(d["Watt Mechanism"].IsObject());
    m_wattCenterMass = d["Watt Mechanism"]["Center Link Mass"].GetDouble();
    m_wattSideMass = d["Watt Mechanism"]["Side Link Mass"].GetDouble();
    m_wattLinkRadius = d["Watt Mechanism"]["Link Radius"].GetDouble();
    m_wattCenterInertia = ReadVectorJSON(d["Watt Mechanism"]["Center Link Inertia"]);
    m_wattSideInertia = ReadVectorJSON(d["Watt Mechanism"]["Side Link Inertia"]);
    m_points[WATT_CNT_LE] = ReadVectorJSON(d["Watt Mechanism"]["Location Left Link to Center"]);
    m_points[WATT_CNT_RI] = ReadVectorJSON(d["Watt Mechanism"]["Location Right Link to Center"]);
    m_points[WATT_LE_CH] = ReadVectorJSON(d["Watt Mechanism"]["Location Left Link to Chassis"]);
    m_points[WATT_RI_CH] = ReadVectorJSON(d["Watt Mechanism"]["Location Right Link to Chassis"]);

    // Read axle inertia
    assert(d.HasMember("Axle"));
    assert(d["Axle"].IsObject());
    m_axleInertia = d["Axle"]["Inertia"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
