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

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a MacPherson strut suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
MacPhersonStrut::MacPhersonStrut(const std::string& filename) 
    : ChMacPhersonStrut(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

MacPhersonStrut::MacPhersonStrut(const rapidjson::Document& d)
    : ChMacPhersonStrut(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    Create(d);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MacPhersonStrut::~MacPhersonStrut() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

// -----------------------------------------------------------------------------
// Worker function for creating a MacPherson strut suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void MacPhersonStrut::Create(const rapidjson::Document& d) {
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

    // Read strut data
    assert(d.HasMember("Strut"));
    assert(d["Strut"].IsObject());

    m_strutMass = d["Strut"]["Mass"].GetDouble();
    m_strutInertia = ReadVectorJSON(d["Strut"]["Inertia"]);
    m_strutRadius = d["Strut"]["Radius"].GetDouble();

    // Read Tierod data
    assert(d.HasMember("Tierod"));
    assert(d["Tierod"].IsObject());

    m_points[TIEROD_C] = ReadVectorJSON(d["Tierod"]["Location Chassis"]);
    m_points[TIEROD_U] = ReadVectorJSON(d["Tierod"]["Location Upright"]);

    // Read spring data and create force callback
    assert(d.HasMember("Spring"));
    assert(d["Spring"].IsObject());

    m_points[SPRING_C] = ReadVectorJSON(d["Spring"]["Location Chassis"]);
    m_points[SPRING_U] = ReadVectorJSON(d["Spring"]["Location Upright"]);
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
    m_points[SHOCK_U] = ReadVectorJSON(d["Shock"]["Location Upright"]);

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
