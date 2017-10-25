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
// Multi-link suspension constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/suspension/MultiLink.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a multi-link suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
MultiLink::MultiLink(const std::string& filename) : ChMultiLink(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

MultiLink::MultiLink(const rapidjson::Document& d) : ChMultiLink(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    Create(d);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MultiLink::~MultiLink() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

// -----------------------------------------------------------------------------
// Worker function for creating a MultiLink suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void MultiLink::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read Spindle data
    assert(d.HasMember("Spindle"));
    assert(d["Spindle"].IsObject());

    m_spindleMass = d["Spindle"]["Mass"].GetDouble();
    m_points[SPINDLE] = LoadVectorJSON(d["Spindle"]["COM"]);
    m_spindleInertia = LoadVectorJSON(d["Spindle"]["Inertia"]);
    m_spindleRadius = d["Spindle"]["Radius"].GetDouble();
    m_spindleWidth = d["Spindle"]["Width"].GetDouble();

    // Read Upright data
    assert(d.HasMember("Upright"));
    assert(d["Upright"].IsObject());

    m_uprightMass = d["Upright"]["Mass"].GetDouble();
    m_points[UPRIGHT] = LoadVectorJSON(d["Upright"]["COM"]);
    m_uprightInertia = LoadVectorJSON(d["Upright"]["Inertia"]);
    m_uprightRadius = d["Upright"]["Radius"].GetDouble();

    // Read Upper Arm data
    assert(d.HasMember("Upper Arm"));
    assert(d["Upper Arm"].IsObject());

    m_upperArmMass = d["Upper Arm"]["Mass"].GetDouble();
    m_points[UA_CM] = LoadVectorJSON(d["Upper Arm"]["COM"]);
    m_upperArmInertia = LoadVectorJSON(d["Upper Arm"]["Inertia"]);
    m_upperArmRadius = d["Upper Arm"]["Radius"].GetDouble();
    m_points[UA_F] = LoadVectorJSON(d["Upper Arm"]["Location Chassis Front"]);
    m_points[UA_B] = LoadVectorJSON(d["Upper Arm"]["Location Chassis Back"]);
    m_points[UA_U] = LoadVectorJSON(d["Upper Arm"]["Location Upright"]);

    // Read Lateral data
    assert(d.HasMember("Lateral"));
    assert(d["Lateral"].IsObject());

    m_lateralMass = d["Lateral"]["Mass"].GetDouble();
    m_points[LAT_CM] = LoadVectorJSON(d["Lateral"]["COM"]);
    m_lateralInertia = LoadVectorJSON(d["Lateral"]["Inertia"]);
    m_lateralRadius = d["Lateral"]["Radius"].GetDouble();
    m_points[LAT_C] = LoadVectorJSON(d["Lateral"]["Location Chassis"]);
    m_points[LAT_U] = LoadVectorJSON(d["Lateral"]["Location Upright"]);
    m_directions[UNIV_AXIS_LINK_LAT] = LoadVectorJSON(d["Lateral"]["Universal Joint Axis Link"]);
    m_directions[UNIV_AXIS_CHASSIS_LAT] = LoadVectorJSON(d["Lateral"]["Universal Joint Axis Chassis"]);

    // Read Trailing Link data
    assert(d.HasMember("Trailing Link"));
    assert(d["Trailing Link"].IsObject());

    m_trailingLinkMass = d["Trailing Link"]["Mass"].GetDouble();
    m_points[TL_CM] = LoadVectorJSON(d["Trailing Link"]["COM"]);
    m_trailingLinkInertia = LoadVectorJSON(d["Trailing Link"]["Inertia"]);
    m_trailingLinkRadius = d["Trailing Link"]["Radius"].GetDouble();
    m_points[TL_C] = LoadVectorJSON(d["Trailing Link"]["Location Chassis"]);
    m_points[TL_U] = LoadVectorJSON(d["Trailing Link"]["Location Upright"]);
    m_directions[UNIV_AXIS_LINK_TL] = LoadVectorJSON(d["Trailing Link"]["Universal Joint Axis Link"]);
    m_directions[UNIV_AXIS_CHASSIS_TL] = LoadVectorJSON(d["Trailing Link"]["Universal Joint Axis Chassis"]);

    // Read Tierod data
    assert(d.HasMember("Tierod"));
    assert(d["Tierod"].IsObject());

    m_points[TIEROD_C] = LoadVectorJSON(d["Tierod"]["Location Chassis"]);
    m_points[TIEROD_U] = LoadVectorJSON(d["Tierod"]["Location Upright"]);

    // Read spring data and create force callback
    assert(d.HasMember("Spring"));
    assert(d["Spring"].IsObject());

    m_points[SPRING_C] = LoadVectorJSON(d["Spring"]["Location Chassis"]);
    m_points[SPRING_L] = LoadVectorJSON(d["Spring"]["Location Link"]);
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

    m_points[SHOCK_C] = LoadVectorJSON(d["Shock"]["Location Chassis"]);
    m_points[SHOCK_L] = LoadVectorJSON(d["Shock"]["Location Link"]);

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
