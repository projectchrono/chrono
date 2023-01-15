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
// Three-link Independent Rear Suspension constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/suspension/ThreeLinkIRS.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a three-link IRS suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
ThreeLinkIRS::ThreeLinkIRS(const std::string& filename)
    : ChThreeLinkIRS(""),
      m_springForceCB(nullptr),
      m_shockForceCB(nullptr),
      m_armChassisBushingData(nullptr),
      m_armUpperBushingData(nullptr),
      m_armLowerBushingData(nullptr),
      m_chassisUpperBushingData(nullptr),
      m_chassisLowerBushingData(nullptr) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ThreeLinkIRS::ThreeLinkIRS(const rapidjson::Document& d)
    : ChThreeLinkIRS(""),
      m_springForceCB(nullptr),
      m_shockForceCB(nullptr),
      m_armChassisBushingData(nullptr),
      m_armUpperBushingData(nullptr),
      m_armLowerBushingData(nullptr),
      m_chassisUpperBushingData(nullptr),
      m_chassisLowerBushingData(nullptr) {
    Create(d);
}

ThreeLinkIRS::~ThreeLinkIRS() {}

// -----------------------------------------------------------------------------
// Worker function for creating a ThreeLinkIRS suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void ThreeLinkIRS::Create(const rapidjson::Document& d) {
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

    // Read trailing arm data
    assert(d.HasMember("Trailing Arm"));
    assert(d["Trailing Arm"].IsObject());

    m_armMass = d["Trailing Arm"]["Mass"].GetDouble();
    m_points[TA_CM] = ReadVectorJSON(d["Trailing Arm"]["COM"]);
    m_armInertia = ReadVectorJSON(d["Trailing Arm"]["Inertia"]);
    m_armRadius = d["Trailing Arm"]["Radius"].GetDouble();
    m_points[TA_C] = ReadVectorJSON(d["Trailing Arm"]["Location Chassis"]);
    m_points[TA_S] = ReadVectorJSON(d["Trailing Arm"]["Location Spindle"]);
    if (d["Trailing Arm"].HasMember("Bushing Data Chassis"))
        m_armChassisBushingData = ReadBushingDataJSON(d["Trailing Arm"]["Bushing Data Chassis"]);

    // Read upper link data
    assert(d.HasMember("Upper Link"));
    assert(d["Upper Link"].IsObject());

    m_upperMass = d["Upper Link"]["Mass"].GetDouble();
    m_points[UL_CM] = ReadVectorJSON(d["Upper Link"]["COM"]);
    m_upperInertia = ReadVectorJSON(d["Upper Link"]["Inertia"]);
    m_upperLinkRadius = d["Upper Link"]["Radius"].GetDouble();
    m_points[UL_C] = ReadVectorJSON(d["Upper Link"]["Location Chassis"]);
    m_points[UL_A] = ReadVectorJSON(d["Upper Link"]["Location Arm"]);
    m_dirs[UNIV_AXIS_UPPER] = ReadVectorJSON(d["Upper Link"]["Universal Joint Axis"]);
    if (d["Upper Link"].HasMember("Bushing Data Arm"))
        m_armUpperBushingData = ReadBushingDataJSON(d["Upper Link"]["Bushing Data Arm"]);
    if (d["Upper Link"].HasMember("Bushing Data Chassis"))
        m_chassisUpperBushingData = ReadBushingDataJSON(d["Upper Link"]["Bushing Data Chassis"]);

    // Read lower link data
    assert(d.HasMember("Lower Link"));
    assert(d["Lower Link"].IsObject());

    m_lowerMass = d["Lower Link"]["Mass"].GetDouble();
    m_points[LL_CM] = ReadVectorJSON(d["Lower Link"]["COM"]);
    m_lowerInertia = ReadVectorJSON(d["Lower Link"]["Inertia"]);
    m_lowerLinkRadius = d["Lower Link"]["Radius"].GetDouble();
    m_points[LL_C] = ReadVectorJSON(d["Lower Link"]["Location Chassis"]);
    m_points[LL_A] = ReadVectorJSON(d["Lower Link"]["Location Arm"]);
    m_dirs[UNIV_AXIS_LOWER] = ReadVectorJSON(d["Lower Link"]["Universal Joint Axis"]);
    if (d["Lower Link"].HasMember("Bushing Data Arm"))
        m_armLowerBushingData = ReadBushingDataJSON(d["Lower Link"]["Bushing Data Arm"]);
    if (d["Lower Link"].HasMember("Bushing Data Chassis"))
        m_chassisLowerBushingData = ReadBushingDataJSON(d["Lower Link"]["Bushing Data Chassis"]);

    // Read spring data and create force callback
    assert(d.HasMember("Spring"));
    assert(d["Spring"].IsObject());
    m_points[SPRING_C] = ReadVectorJSON(d["Spring"]["Location Chassis"]);
    m_points[SPRING_A] = ReadVectorJSON(d["Spring"]["Location Arm"]);
    m_springForceCB = ReadTSDAFunctorJSON(d["Spring"], m_springRestLength);

    // Read shock data and create force callback
    assert(d.HasMember("Shock"));
    assert(d["Shock"].IsObject());
    m_points[SHOCK_C] = ReadVectorJSON(d["Shock"]["Location Chassis"]);
    m_points[SHOCK_A] = ReadVectorJSON(d["Shock"]["Location Arm"]);
    m_shockForceCB = ReadTSDAFunctorJSON(d["Shock"], m_shockRestLength);

    // Read axle inertia
    assert(d.HasMember("Axle"));
    assert(d["Axle"].IsObject());
    m_axleInertia = d["Axle"]["Inertia"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
