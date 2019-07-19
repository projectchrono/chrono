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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Steerable leaf-spring solid axle suspension constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ToeBarLeafspringAxle.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a solid axle suspension using data from the specified JSON file.
// -----------------------------------------------------------------------------
ToeBarLeafspringAxle::ToeBarLeafspringAxle(const std::string& filename)
    : ChToeBarLeafspringAxle(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ToeBarLeafspringAxle::ToeBarLeafspringAxle(const rapidjson::Document& d)
    : ChToeBarLeafspringAxle(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    Create(d);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ToeBarLeafspringAxle::~ToeBarLeafspringAxle() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

// -----------------------------------------------------------------------------
// Worker function for creating a ToeBarLeafspringAxle suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void ToeBarLeafspringAxle::Create(const rapidjson::Document& d) {
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

    // Read Axle Tube data
    assert(d.HasMember("Axle Tube"));
    assert(d["Axle Tube"].IsObject());

    m_axleTubeMass = d["Axle Tube"]["Mass"].GetDouble();
    m_axleTubeCOM = ReadVectorJSON(d["Axle Tube"]["COM"]);
    m_axleTubeInertia = ReadVectorJSON(d["Axle Tube"]["Inertia"]);
    m_axleTubeRadius = d["Axle Tube"]["Radius"].GetDouble();

    // Read Knuckle data
    assert(d.HasMember("Knuckle"));
    assert(d["Knuckle"].IsObject());

    m_knuckleMass = d["Knuckle"]["Mass"].GetDouble();
    m_points[KNUCKLE_CM] = ReadVectorJSON(d["Knuckle"]["COM"]);
    m_knuckleInertia = ReadVectorJSON(d["Knuckle"]["Inertia"]);
    m_knuckleRadius = d["Knuckle"]["Radius"].GetDouble();
    m_points[KNUCKLE_L] = ReadVectorJSON(d["Knuckle"]["Location Lower"]);
    m_points[KNUCKLE_U] = ReadVectorJSON(d["Knuckle"]["Location Upper"]);
    m_points[KNUCKLE_DRL] = ReadVectorJSON(d["Knuckle"]["Location Draglink"]);
    if (m_points[KNUCKLE_DRL].y() < 0.0) {
        m_use_left_knuckle = false;
        m_points[KNUCKLE_DRL].y() *= -1.0;
        std::cout << "Right Knuckle is actuated!" << std::endl;
    }

    // Read Tierod aka Toe Bar data
    assert(d.HasMember("Tierod"));
    assert(d["Tierod"].IsObject());

    m_tierodMass = d["Tierod"]["Mass"].GetDouble();
    m_tierodInertia = ReadVectorJSON(d["Tierod"]["Inertia"]);
    m_points[TIEROD_K] = ReadVectorJSON(d["Tierod"]["Location Knuckle"]);
    m_tierodRadius = d["Tierod"]["Radius"].GetDouble();

    // Read spring data and create force callback
    assert(d.HasMember("Draglink"));
    assert(d["Draglink"].IsObject());

    m_draglinkMass = d["Draglink"]["Mass"].GetDouble();
    m_draglinkInertia = ReadVectorJSON(d["Draglink"]["Inertia"]);
    m_points[DRAGLINK_C] = ReadVectorJSON(d["Draglink"]["Location Chassis"]);
    m_draglinkRadius = d["Draglink"]["Radius"].GetDouble();

    // Read Draglink data
    assert(d.HasMember("Tierod"));
    assert(d["Tierod"].IsObject());

    m_points[SPRING_C] = ReadVectorJSON(d["Spring"]["Location Chassis"]);
    m_points[SPRING_A] = ReadVectorJSON(d["Spring"]["Location Axle"]);
    m_springRestLength = d["Spring"]["Free Length"].GetDouble();
    m_springMinLength = d["Spring"]["Minimal Length"].GetDouble();
    m_springMaxLength = d["Spring"]["Maximal Length"].GetDouble();

    if (d["Spring"].HasMember("Spring Coefficient")) {
        m_springForceCB = new LinearSpringBistopForce(d["Spring"]["Spring Coefficient"].GetDouble(), m_springMinLength,
                                                      m_springMaxLength);
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
        if (d["Shock"].HasMember("Degressivity Compression") && d["Shock"].HasMember("Degressivity Expansion")) {
            m_shockForceCB = new DegressiveDamperForce(d["Shock"]["Damping Coefficient"].GetDouble(),
                                                       d["Shock"]["Degressivity Compression"].GetDouble(),
                                                       d["Shock"]["Degressivity Expansion"].GetDouble());
        } else {
            m_shockForceCB = new LinearDamperForce(d["Shock"]["Damping Coefficient"].GetDouble());
        }
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
