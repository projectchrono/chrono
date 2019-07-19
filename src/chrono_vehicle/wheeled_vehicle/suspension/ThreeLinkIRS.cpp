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

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a three-link IRS suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
ThreeLinkIRS::ThreeLinkIRS(const std::string& filename)
    : ChThreeLinkIRS(""), m_springForceCB(nullptr), m_shockForceCB(nullptr) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ThreeLinkIRS::ThreeLinkIRS(const rapidjson::Document& d)
    : ChThreeLinkIRS(""), m_springForceCB(nullptr), m_shockForceCB(nullptr) {
    Create(d);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ThreeLinkIRS::~ThreeLinkIRS() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

// -----------------------------------------------------------------------------
// Worker function for creating a ThreeLinkIRS suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void ThreeLinkIRS::Create(const rapidjson::Document& d) {
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

    // Read trailing arm data
    assert(d.HasMember("Trailing Arm"));
    assert(d["Trailing Arm"].IsObject());

    m_armMass = d["Trailing Arm"]["Mass"].GetDouble();
    m_points[TA_CM] = ReadVectorJSON(d["Trailing Arm"]["COM"]);
    m_armInertia = ReadVectorJSON(d["Trailing Arm"]["Inertia"]);
    m_armRadius = d["Trailing Arm"]["Radius"].GetDouble();
    m_points[TA_C] = ReadVectorJSON(d["Trailing Arm"]["Location Chassis"]);
    m_points[TA_S] = ReadVectorJSON(d["Trailing Arm"]["Location Spindle"]);

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

    // Read spring data and create force callback
    assert(d.HasMember("Spring"));
    assert(d["Spring"].IsObject());

    m_points[SPRING_C] = ReadVectorJSON(d["Spring"]["Location Chassis"]);
    m_points[SPRING_A] = ReadVectorJSON(d["Spring"]["Location Arm"]);
    m_springRestLength = d["Spring"]["Free Length"].GetDouble();

    if (d["Spring"].HasMember("Spring Coefficient")) {
        m_springForceCB = new LinearSpringForce(d["Spring"]["Spring Coefficient"].GetDouble());
    } else if (d["Spring"].HasMember("Curve Data")) {
        int num_points = d["Spring"]["Curve Data"].Size();
        MapSpringForce* springForceCB = new MapSpringForce();
        for (int i = 0; i < num_points; i++) {
            springForceCB->add_point(d["Spring"]["Curve Data"][i][0u].GetDouble(),
                                     d["Spring"]["Curve Data"][i][1u].GetDouble());
        }
        m_springForceCB = springForceCB;
    }

    // Read shock data and create force callback
    assert(d.HasMember("Shock"));
    assert(d["Shock"].IsObject());

    m_points[SHOCK_C] = ReadVectorJSON(d["Shock"]["Location Chassis"]);
    m_points[SHOCK_A] = ReadVectorJSON(d["Shock"]["Location Arm"]);

    if (d["Shock"].HasMember("Damping Coefficient")) {
        m_shockForceCB = new LinearDamperForce(d["Shock"]["Damping Coefficient"].GetDouble());
    } else if (d["Shock"].HasMember("Curve Data")) {
        int num_points = d["Shock"]["Curve Data"].Size();
        MapDamperForce* shockForceCB = new MapDamperForce();
        for (int i = 0; i < num_points; i++) {
            shockForceCB->add_point(d["Shock"]["Curve Data"][i][0u].GetDouble(),
                                    d["Shock"]["Curve Data"][i][1u].GetDouble());
        }
        m_shockForceCB = shockForceCB;
    }

    // Read axle inertia
    assert(d.HasMember("Axle"));
    assert(d["Axle"].IsObject());

    m_axleInertia = d["Axle"]["Inertia"].GetDouble();
}

}  // end namespace vehicle
}  // end namespace chrono
