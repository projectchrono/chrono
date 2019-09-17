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
// Authors: Daniel Melanz, Radu Serban
// =============================================================================
//
// Solid axle suspension constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/suspension/SolidAxle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a solid axle suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
SolidAxle::SolidAxle(const std::string& filename) : ChSolidAxle(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

SolidAxle::SolidAxle(const rapidjson::Document& d) : ChSolidAxle(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    Create(d);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SolidAxle::~SolidAxle() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

// -----------------------------------------------------------------------------
// Worker function for creating a SolidAxle suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void SolidAxle::Create(const rapidjson::Document& d) {
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

    // Read Knuckle data
    assert(d.HasMember("Knuckle"));
    assert(d["Knuckle"].IsObject());

    m_knuckleMass = d["Knuckle"]["Mass"].GetDouble();
    m_points[KNUCKLE_CM] = ReadVectorJSON(d["Knuckle"]["COM"]);
    m_knuckleInertia = ReadVectorJSON(d["Knuckle"]["Inertia"]);
    m_knuckleRadius = d["Knuckle"]["Radius"].GetDouble();
    m_points[KNUCKLE_L] = ReadVectorJSON(d["Knuckle"]["Location Lower"]);
    m_points[KNUCKLE_U] = ReadVectorJSON(d["Knuckle"]["Location Upper"]);

    // Read UL data
    assert(d.HasMember("Upper Link"));
    assert(d["Upper Link"].IsObject());

    m_ULMass = d["Upper Link"]["Mass"].GetDouble();
    m_points[UL_CM] = ReadVectorJSON(d["Upper Link"]["COM"]);
    m_ULInertia = ReadVectorJSON(d["Upper Link"]["Inertia"]);
    m_ULRadius = d["Upper Link"]["Radius"].GetDouble();
    m_points[UL_A] = ReadVectorJSON(d["Upper Link"]["Location Axle"]);
    m_points[UL_C] = ReadVectorJSON(d["Upper Link"]["Location Chassis"]);

    // Read LL data
    assert(d.HasMember("Lower Link"));
    assert(d["Lower Link"].IsObject());

    m_LLMass = d["Lower Link"]["Mass"].GetDouble();
    m_points[LL_CM] = ReadVectorJSON(d["Lower Link"]["COM"]);
    m_LLInertia = ReadVectorJSON(d["Lower Link"]["Inertia"]);
    m_LLRadius = d["Lower Link"]["Radius"].GetDouble();
    m_points[LL_A] = ReadVectorJSON(d["Lower Link"]["Location Axle"]);
    m_points[LL_C] = ReadVectorJSON(d["Lower Link"]["Location Chassis"]);

    // Read Axle Tube data
    assert(d.HasMember("Axle Tube"));
    assert(d["Axle Tube"].IsObject());

    m_axleTubeMass = d["Axle Tube"]["Mass"].GetDouble();
    m_axleTubeCOM = ReadVectorJSON(d["Axle Tube"]["COM"]);
    m_axleTubeInertia = ReadVectorJSON(d["Axle Tube"]["Inertia"]);
    m_axleTubeRadius = d["Axle Tube"]["Radius"].GetDouble();

    // Read Tierod data
    assert(d.HasMember("Tierod"));
    assert(d["Tierod"].IsObject());

    m_tierodMass = d["Tierod"]["Mass"].GetDouble();
    m_tierodInertia = ReadVectorJSON(d["Tierod"]["Inertia"]);
    m_points[TIEROD_K] = ReadVectorJSON(d["Tierod"]["Location Knuckle"]);
    m_tierodRadius = d["Tierod"]["Radius"].GetDouble();

    // Read Draglink data
    assert(d.HasMember("Draglink"));
    assert(d["Draglink"].IsObject());

    m_draglinkMass = d["Draglink"]["Mass"].GetDouble();
    m_draglinkInertia = ReadVectorJSON(d["Draglink"]["Inertia"]);
    m_points[DRAGLINK_C] = ReadVectorJSON(d["Draglink"]["Location Chassis"]);
    m_points[BELLCRANK_DRAGLINK] = ReadVectorJSON(d["Draglink"]["Location Bell Crank"]);
    m_draglinkRadius = d["Draglink"]["Radius"].GetDouble();

    // Read Bell Crank data
    assert(d.HasMember("Bell Crank"));
    assert(d["Bell Crank"].IsObject());

    m_bellCrankMass = d["Bell Crank"]["Mass"].GetDouble();
    m_bellCrankInertia = ReadVectorJSON(d["Bell Crank"]["Inertia"]);
    m_points[BELLCRANK_TIEROD] = ReadVectorJSON(d["Bell Crank"]["Location Tierod"]);
    m_points[BELLCRANK_AXLE] = ReadVectorJSON(d["Bell Crank"]["Location Axle"]);
    m_bellCrankRadius = d["Bell Crank"]["Radius"].GetDouble();

    // Read spring data and create force callback
    assert(d.HasMember("Spring"));
    assert(d["Spring"].IsObject());

    m_points[SPRING_C] = ReadVectorJSON(d["Spring"]["Location Chassis"]);
    m_points[SPRING_A] = ReadVectorJSON(d["Spring"]["Location Axle"]);
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
    m_points[SHOCK_A] = ReadVectorJSON(d["Shock"]["Location Axle"]);

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
