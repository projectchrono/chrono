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
// Leaf-spring solid axle suspension constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/suspension/LeafspringAxle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a solid axle suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
LeafspringAxle::LeafspringAxle(const std::string& filename)
    : ChLeafspringAxle(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

LeafspringAxle::LeafspringAxle(const rapidjson::Document& d)
    : ChLeafspringAxle(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    Create(d);
}

LeafspringAxle::~LeafspringAxle() {}

// -----------------------------------------------------------------------------
// Worker function for creating a LeafspringAxle suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void LeafspringAxle::Create(const rapidjson::Document& d) {
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

    // Read Axle Tube data
    assert(d.HasMember("Axle Tube"));
    assert(d["Axle Tube"].IsObject());

    m_axleTubeMass = d["Axle Tube"]["Mass"].GetDouble();
    m_axleTubeCOM = ReadVectorJSON(d["Axle Tube"]["COM"]);
    m_axleTubeInertia = ReadVectorJSON(d["Axle Tube"]["Inertia"]);
    m_axleTubeRadius = d["Axle Tube"]["Radius"].GetDouble();

    // Read spring data and create force callback
    assert(d.HasMember("Spring"));
    assert(d["Spring"].IsObject());

    m_points[SPRING_C] = ReadVectorJSON(d["Spring"]["Location Chassis"]);
    m_points[SPRING_A] = ReadVectorJSON(d["Spring"]["Location Axle"]);
    m_springRestLength = d["Spring"]["Free Length"].GetDouble();
    double preload = 0;
    if (d["Spring"].HasMember("Preload"))
        preload = d["Spring"]["Preload"].GetDouble();

    if (d["Spring"].HasMember("Minimum Length") && d["Spring"].HasMember("Maximum Length")) {
        if (d["Spring"].HasMember("Spring Coefficient")) {
            m_springForceCB = chrono_types::make_shared<LinearSpringBistopForce>(
                d["Spring"]["Spring Coefficient"].GetDouble(), d["Spring"]["Minimum Length"].GetDouble(),
                d["Spring"]["Maximum Length"].GetDouble(), preload);
        } else if (d["Spring"].HasMember("Curve Data")) {
            int num_points = d["Spring"]["Curve Data"].Size();
            auto springForceCB = chrono_types::make_shared<MapSpringBistopForce>(
                d["Spring"]["Minimum Length"].GetDouble(), d["Spring"]["Maximum Length"].GetDouble(), preload);
            for (int i = 0; i < num_points; i++) {
                springForceCB->add_point(d["Spring"]["Curve Data"][i][0u].GetDouble(),
                                         d["Spring"]["Curve Data"][i][1u].GetDouble());
            }
            m_springForceCB = springForceCB;
        }
    } else {
        if (d["Spring"].HasMember("Spring Coefficient")) {
            m_springForceCB =
                chrono_types::make_shared<LinearSpringForce>(d["Spring"]["Spring Coefficient"].GetDouble(), preload);
        } else if (d["Spring"].HasMember("Curve Data")) {
            int num_points = d["Spring"]["Curve Data"].Size();
            auto springForceCB = chrono_types::make_shared<MapSpringForce>(preload);
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
