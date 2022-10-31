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

#include "chrono_vehicle/wheeled_vehicle/suspension/SAELeafspringAxle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a solid axle suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
SAELeafspringAxle::SAELeafspringAxle(const std::string& filename)
    : ChSAELeafspringAxle(""),
      m_springForceCB(NULL),
      m_shockForceCB(NULL),
      m_shackleBushingData(nullptr),
      m_clampBushingData(nullptr),
      m_leafspringBushingData(nullptr) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

SAELeafspringAxle::SAELeafspringAxle(const rapidjson::Document& d)
    : ChSAELeafspringAxle(""),
      m_springForceCB(nullptr),
      m_shockForceCB(nullptr),
      m_shackleBushingData(nullptr),
      m_clampBushingData(nullptr),
      m_leafspringBushingData(nullptr) {
    Create(d);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SAELeafspringAxle::~SAELeafspringAxle() {}

// -----------------------------------------------------------------------------
// Worker function for creating a SAELeafspringAxle suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void SAELeafspringAxle::Create(const rapidjson::Document& d) {
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
    assert(d.HasMember("Auxiliary Spring"));
    assert(d["Auxiliary Spring"].IsObject());

    m_points[SPRING_C] = ReadVectorJSON(d["Auxiliary Spring"]["Location Chassis"]);
    m_points[SPRING_A] = ReadVectorJSON(d["Auxiliary Spring"]["Location Axle"]);
    m_springRestLength = d["Auxiliary Spring"]["Free Length"].GetDouble();
    double preload_aux = 0;
    if (d["Auxiliary Spring"].HasMember("Preload"))
        preload_aux = d["Auxiliary Spring"]["Preload"].GetDouble();

    if (d["Auxiliary Spring"].HasMember("Minimum Length") && d["Auxiliary Spring"].HasMember("Maximum Length")) {
        if (d["Auxiliary Spring"].HasMember("Spring Coefficient")) {
            m_springForceCB = chrono_types::make_shared<LinearSpringBistopForce>(
                d["Auxiliary Spring"]["Spring Coefficient"].GetDouble(),
                d["Auxiliary Spring"]["Minimum Length"].GetDouble(),
                d["Auxiliary Spring"]["Maximum Length"].GetDouble(), preload_aux);
        } else if (d["Auxiliary Spring"].HasMember("Curve Data")) {
            int num_points = d["Auxiliary Spring"]["Curve Data"].Size();
            auto springForceCB = chrono_types::make_shared<MapSpringBistopForce>(
                d["Auxiliary Spring"]["Minimum Length"].GetDouble(),
                d["Auxiliary Spring"]["Maximum Length"].GetDouble(), preload_aux);
            for (int i = 0; i < num_points; i++) {
                springForceCB->add_point(d["Auxiliary Spring"]["Curve Data"][i][0u].GetDouble(),
                                         d["Auxiliary Spring"]["Curve Data"][i][1u].GetDouble());
            }
            m_springForceCB = springForceCB;
        }
    } else {
        if (d["Auxiliary Spring"].HasMember("Spring Coefficient")) {
            m_springForceCB = chrono_types::make_shared<LinearSpringForce>(
                d["Auxiliary Spring"]["Spring Coefficient"].GetDouble(), preload_aux);
        } else if (d["Auxiliary Spring"].HasMember("Curve Data")) {
            int num_points = d["Auxiliary Spring"]["Curve Data"].Size();
            auto springForceCB = chrono_types::make_shared<MapSpringForce>(preload_aux);
            for (int i = 0; i < num_points; i++) {
                springForceCB->add_point(d["Auxiliary Spring"]["Curve Data"][i][0u].GetDouble(),
                                         d["Auxiliary Spring"]["Curve Data"][i][1u].GetDouble());
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

    // read leafspring hardpoints
    assert(d.HasMember("Leafspring"));
    assert(d["Leafspring"].IsObject());

    m_points[FRONT_HANGER] = ReadVectorJSON(d["Leafspring"]["Location Front Hanger"]);
    m_points[REAR_HANGER] = ReadVectorJSON(d["Leafspring"]["Location Rear Hanger"]);
    m_points[SHACKLE] = ReadVectorJSON(d["Leafspring"]["Location Shackle"]);
    m_points[CLAMP_A] = ReadVectorJSON(d["Leafspring"]["Location Clamp A"]);
    m_points[CLAMP_B] = ReadVectorJSON(d["Leafspring"]["Location Clamp B"]);

    double vertical_stiffness = d["Leafspring"]["Vertical Stiffness"].GetDouble();
    double stiffness_bias = d["Leafspring"]["Stiffness Bias"].GetDouble();
    double lateral_stiffness = d["Leafspring"]["Lateral Stiffness"].GetDouble();
    double vertical_preload = d["Leafspring"]["Vertical Preload Force"].GetDouble();

    // read mass properties
    assert(d["Leafspring"].HasMember("Frontleaf"));
    assert(d["Leafspring"]["Frontleaf"].IsObject());

    m_frontleafMass = d["Leafspring"]["Frontleaf"]["Mass"].GetDouble();
    m_frontleafInertia = ReadVectorJSON(d["Leafspring"]["Frontleaf"]["Inertia"]);

    assert(d["Leafspring"].HasMember("Rearleaf"));
    assert(d["Leafspring"]["Rearleaf"].IsObject());

    m_rearleafMass = d["Leafspring"]["Rearleaf"]["Mass"].GetDouble();
    m_rearleafInertia = ReadVectorJSON(d["Leafspring"]["Rearleaf"]["Inertia"]);

    assert(d["Leafspring"].HasMember("Half Clamp"));
    assert(d["Leafspring"]["Half Clamp"].IsObject());

    m_clampMass = d["Leafspring"]["Half Clamp"]["Mass"].GetDouble();
    m_clampInertia = ReadVectorJSON(d["Leafspring"]["Half Clamp"]["Inertia"]);

    assert(d["Leafspring"].HasMember("Shackle"));
    assert(d["Leafspring"]["Shackle"].IsObject());

    m_shackleMass = d["Leafspring"]["Shackle"]["Mass"].GetDouble();
    m_shackleInertia = ReadVectorJSON(d["Leafspring"]["Shackle"]["Inertia"]);

    // Bushing data (optional)
    if (d["Leafspring"].HasMember("Shackle Bushing Data")) {
        m_shackleBushingData = ReadBushingDataJSON(d["Leafspring"]["Shackle Bushing Data"]);
    }
    if (d["Leafspring"].HasMember("Clamp Bushing Data")) {
        m_clampBushingData = ReadBushingDataJSON(d["Leafspring"]["Clamp Bushing Data"]);
    }
    if (d["Leafspring"].HasMember("Leafspring Bushing Data")) {
        m_shackleBushingData = ReadBushingDataJSON(d["Leafspring"]["Leafspring Bushing Data"]);
    }

    ChVector<> ra = getLocation(CLAMP_A) - getLocation(FRONT_HANGER);
    ChVector<> rb = getLocation(CLAMP_B) - getLocation(SHACKLE);

    ChVector<> preload(0, 0, vertical_preload / 2.0);
    ChVector<> Ma = preload.Cross(ra);
    ChVector<> Mb = preload.Cross(rb);

    double KAlat = lateral_stiffness / (1.0 + stiffness_bias);
    double KBlat = stiffness_bias * KAlat;

    double KAvert = vertical_stiffness / (1.0 + stiffness_bias);
    double KBvert = stiffness_bias * KAvert;

    double KrotLatA = KAlat * pow(ra.Length(), 2.0);
    double KrotLatB = KBlat * pow(rb.Length(), 2.0);

    double KrotVertA = KAvert * pow(ra.Length(), 2.0);
    double KrotVertB = KBvert * pow(rb.Length(), 2.0);

    double rest_angle_A = Ma.y() / KrotVertA;
    double rest_angle_B = Mb.y() / KrotVertB;

    double damping_factor = 0.01;

    m_latRotSpringCBA = chrono_types::make_shared<LinearSpringDamperTorque>(KrotLatA, KrotLatA * damping_factor, 0);
    m_latRotSpringCBB = chrono_types::make_shared<LinearSpringDamperTorque>(KrotLatB, KrotLatB * damping_factor, 0);

    m_vertRotSpringCBA =
        chrono_types::make_shared<LinearSpringDamperTorque>(KrotVertA, KrotVertA * damping_factor, rest_angle_A);
    m_vertRotSpringCBB =
        chrono_types::make_shared<LinearSpringDamperTorque>(KrotVertB, KrotVertB * damping_factor, rest_angle_B);
}

}  // end namespace vehicle
}  // end namespace chrono
