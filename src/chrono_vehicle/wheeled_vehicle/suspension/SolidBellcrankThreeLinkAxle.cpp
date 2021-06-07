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
// Three link solid axle suspension constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/suspension/SolidBellcrankThreeLinkAxle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a solid axle suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
SolidBellcrankThreeLinkAxle::SolidBellcrankThreeLinkAxle(const std::string& filename)
    : ChSolidBellcrankThreeLinkAxle(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

SolidBellcrankThreeLinkAxle::SolidBellcrankThreeLinkAxle(const rapidjson::Document& d)
    : ChSolidBellcrankThreeLinkAxle(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    Create(d);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SolidBellcrankThreeLinkAxle::~SolidBellcrankThreeLinkAxle() {}

// -----------------------------------------------------------------------------
// Worker function for creating a SolidBellcrankThreeLinkAxle suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void SolidBellcrankThreeLinkAxle::Create(const rapidjson::Document& d) {
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
    m_knuckleInertia = ReadVectorJSON(d["Knuckle"]["Inertia"]);
    m_points[KNUCKLE_CM] = ReadVectorJSON(d["Knuckle"]["COM"]);
    m_points[KNUCKLE_L] = ReadVectorJSON(d["Knuckle"]["Location Lower"]);
    m_points[KNUCKLE_U] = ReadVectorJSON(d["Knuckle"]["Location Upper"]);
    m_points[KNUCKLE_T] = ReadVectorJSON(d["Knuckle"]["Location Tierod"]);

    // Read Bellcrank data
    assert(d.HasMember("Bellcrank"));
    assert(d["Bellcrank"].IsObject());

    m_bellcrankMass = d["Bellcrank"]["Mass"].GetDouble();
    m_bellcrankInertia = ReadVectorJSON(d["Bellcrank"]["Inertia"]);
    m_points[BELLCRANK_A] = ReadVectorJSON(d["Bellcrank"]["Location Axle"]);
    m_points[BELLCRANK_T] = ReadVectorJSON(d["Bellcrank"]["Location Tierod"]);

    // Read Draglink data
    assert(d.HasMember("Draglink"));
    assert(d["Draglink"].IsObject());

    m_draglinkMass = d["Draglink"]["Mass"].GetDouble();
    m_draglinkInertia = ReadVectorJSON(d["Draglink"]["Inertia"]);
    m_points[DRAGLINK_S] = ReadVectorJSON(d["Draglink"]["Location Steering"]);
    m_points[BELLCRANK_D] = ReadVectorJSON(d["Draglink"]["Location Bellcrank"]);

    // Read triangular link data
    assert(d.HasMember("Triangular Link"));
    assert(d["Triangular Link"].IsObject());
    m_trangleMass = d["Triangular Link"]["Mass"].GetDouble();
    m_triangleInertia = ReadVectorJSON(d["Triangular Link"]["Inertia"]);
    m_points[TRIANGLE_C] = ReadVectorJSON(d["Triangular Link"]["Location Chassis"]);
    m_points[TRIANGLE_A] = ReadVectorJSON(d["Triangular Link"]["Location Axle"]);

    // Read longitudinal link data
    assert(d.HasMember("Longitudinal Link"));
    assert(d["Longitudinal Link"].IsObject());
    m_linkMass = d["Longitudinal Link"]["Mass"].GetDouble();
    m_linkInertia = ReadVectorJSON(d["Longitudinal Link"]["Inertia"]);
    m_points[LINK_C] = ReadVectorJSON(d["Longitudinal Link"]["Location Chassis"]);
    m_points[LINK_A] = ReadVectorJSON(d["Longitudinal Link"]["Location Axle"]);

    // read tierod
    assert(d.HasMember("Tierod"));
    assert(d["Tierod"].IsObject());
    m_tierodMass = d["Tierod"]["Mass"].GetDouble();
    m_tierodInertia = ReadVectorJSON(d["Tierod"]["Inertia"]);

    // Read spring data and create force callback
    assert(d.HasMember("Spring"));
    assert(d["Spring"].IsObject());

    m_points[SPRING_C] = ReadVectorJSON(d["Spring"]["Location Chassis"]);
    m_points[SPRING_A] = ReadVectorJSON(d["Spring"]["Location Axle"]);
    m_springRestLength = d["Spring"]["Free Length"].GetDouble();

    if (d["Spring"].HasMember("Minimum Length") && d["Spring"].HasMember("Maximum Length")) {
        if (d["Spring"].HasMember("Spring Coefficient")) {
            m_springForceCB = chrono_types::make_shared<LinearSpringBistopForce>(
                d["Spring"]["Spring Coefficient"].GetDouble(), d["Spring"]["Minimum Length"].GetDouble(),
                d["Spring"]["Maximum Length"].GetDouble());
        } else if (d["Spring"].HasMember("Curve Data")) {
            int num_points = d["Spring"]["Curve Data"].Size();
            auto springForceCB = chrono_types::make_shared<MapSpringBistopForce>(
                d["Spring"]["Minimum Length"].GetDouble(), d["Spring"]["Maximum Length"].GetDouble());
            for (int i = 0; i < num_points; i++) {
                springForceCB->add_point(d["Spring"]["Curve Data"][i][0u].GetDouble(),
                                         d["Spring"]["Curve Data"][i][1u].GetDouble());
            }
            m_springForceCB = springForceCB;
        }
    } else {
        if (d["Spring"].HasMember("Spring Coefficient")) {
            m_springForceCB =
                chrono_types::make_shared<LinearSpringForce>(d["Spring"]["Spring Coefficient"].GetDouble());
        } else if (d["Spring"].HasMember("Curve Data")) {
            int num_points = d["Spring"]["Curve Data"].Size();
            auto springForceCB = chrono_types::make_shared<MapSpringForce>();
            for (int i = 0; i < num_points; i++) {
                springForceCB->add_point(d["Spring"]["Curve Data"][i][0u].GetDouble(),
                                         d["Spring"]["Curve Data"][i][1u].GetDouble());
            }
            m_springForceCB = springForceCB;
        }
    }

    // Read shock data and create force callback
    assert(d.HasMember("Shock"));
    assert(d["Shock"].IsObject());

    m_points[SHOCK_C] = ReadVectorJSON(d["Shock"]["Location Chassis"]);
    m_points[SHOCK_A] = ReadVectorJSON(d["Shock"]["Location Axle"]);

    if (d["Shock"].HasMember("Damping Coefficient")) {
        if (d["Shock"].HasMember("Degressivity Compression") && d["Shock"].HasMember("Degressivity Expansion")) {
            m_shockForceCB = chrono_types::make_shared<DegressiveDamperForce>(
                d["Shock"]["Damping Coefficient"].GetDouble(), d["Shock"]["Degressivity Compression"].GetDouble(),
                d["Shock"]["Degressivity Expansion"].GetDouble());
        } else {
            m_shockForceCB =
                chrono_types::make_shared<LinearDamperForce>(d["Shock"]["Damping Coefficient"].GetDouble());
        }
    } else if (d["Shock"].HasMember("Curve Data")) {
        int num_points = d["Shock"]["Curve Data"].Size();
        auto shockForceCB = chrono_types::make_shared<MapDamperForce>();
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
