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

#include "chrono_vehicle/wheeled_vehicle/suspension/SAEToeBarLeafspringAxle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a solid axle suspension using data from the specified JSON file.
// -----------------------------------------------------------------------------
SAEToeBarLeafspringAxle::SAEToeBarLeafspringAxle(const std::string& filename)
    : ChSAEToeBarLeafspringAxle(""),
      m_springForceCB(NULL),
      m_shockForceCB(NULL),
      m_use_left_knuckle(true),
      m_shackleBushingData(nullptr),
      m_clampBushingData(nullptr),
      m_leafspringBushingData(nullptr) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

SAEToeBarLeafspringAxle::SAEToeBarLeafspringAxle(const rapidjson::Document& d)
    : ChSAEToeBarLeafspringAxle(""),
      m_springForceCB(NULL),
      m_shockForceCB(NULL),
      m_use_left_knuckle(true),
      m_shackleBushingData(nullptr),
      m_clampBushingData(nullptr),
      m_leafspringBushingData(nullptr) {
    Create(d);
}

SAEToeBarLeafspringAxle::~SAEToeBarLeafspringAxle() {}

// -----------------------------------------------------------------------------
// Worker function for creating a SAEToeBarLeafspringAxle suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void SAEToeBarLeafspringAxle::Create(const rapidjson::Document& d) {
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
    }

    // Read Tierod aka Toe Bar data
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
    m_draglinkRadius = d["Draglink"]["Radius"].GetDouble();

    // Read spring data and create force callback
    assert(d.HasMember("Auxiliary Spring"));
    assert(d["Auxiliary Spring"].IsObject());
    m_points[SPRING_C] = ReadVectorJSON(d["Auxiliary Spring"]["Location Chassis"]);
    m_points[SPRING_A] = ReadVectorJSON(d["Auxiliary Spring"]["Location Axle"]);
    m_springForceCB = ReadTSDAFunctorJSON(d["Auxiliary Spring"], m_springRestLength);

    // Read shock data and create force callback
    assert(d.HasMember("Shock"));
    assert(d["Shock"].IsObject());
    m_points[SHOCK_C] = ReadVectorJSON(d["Shock"]["Location Chassis"]);
    m_points[SHOCK_A] = ReadVectorJSON(d["Shock"]["Location Axle"]);
    m_shockForceCB = ReadTSDAFunctorJSON(d["Shock"], m_shockRestLength);

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
    ////std::cout << "m_frontleafMass " << m_frontleafMass << "\n";
    ////std::cout << "m_frontleafInertia " << m_frontleafInertia << "\n";

    assert(d["Leafspring"].HasMember("Rearleaf"));
    assert(d["Leafspring"]["Rearleaf"].IsObject());

    m_rearleafMass = d["Leafspring"]["Rearleaf"]["Mass"].GetDouble();
    m_rearleafInertia = ReadVectorJSON(d["Leafspring"]["Rearleaf"]["Inertia"]);
    ////std::cout << "m_rearleafMass " << m_rearleafMass << "\n";
    ////std::cout << "m_rearleafInertia " << m_rearleafInertia << "\n";

    assert(d["Leafspring"].HasMember("Half Clamp"));
    assert(d["Leafspring"]["Half Clamp"].IsObject());

    m_clampMass = d["Leafspring"]["Half Clamp"]["Mass"].GetDouble();
    m_clampInertia = ReadVectorJSON(d["Leafspring"]["Half Clamp"]["Inertia"]);
    ////std::cout << "m_clampMass " << m_clampMass << "\n";
    ////std::cout << "m_clampInertia " << m_clampInertia << "\n";

    assert(d["Leafspring"].HasMember("Shackle"));
    assert(d["Leafspring"]["Shackle"].IsObject());

    m_shackleMass = d["Leafspring"]["Shackle"]["Mass"].GetDouble();
    m_shackleInertia = ReadVectorJSON(d["Leafspring"]["Shackle"]["Inertia"]);
    ////std::cout << "m_shackleMass " << m_shackleMass << "\n";
    ////std::cout << "m_shackleInertia " << m_shackleInertia << "\n";

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

    ChVector3d ra = getLocation(CLAMP_A) - getLocation(FRONT_HANGER);
    ChVector3d rb = getLocation(CLAMP_B) - getLocation(SHACKLE);

    ChVector3d preload(0, 0, vertical_preload / 2.0);
    ChVector3d Ma = preload.Cross(ra);
    ChVector3d Mb = preload.Cross(rb);

    ////std::cout << "Ma " << Ma << std::endl;
    ////std::cout << "Mb " << Mb << std::endl;

    double KAlat = lateral_stiffness / (1.0 + stiffness_bias);
    double KBlat = stiffness_bias * KAlat;

    double KAvert = vertical_stiffness / (1.0 + stiffness_bias);
    double KBvert = stiffness_bias * KAvert;

    double KrotLatA = KAlat * std::pow(ra.Length(), 2.0);
    double KrotLatB = KBlat * std::pow(rb.Length(), 2.0);

    double KrotVertA = KAvert * std::pow(ra.Length(), 2.0);
    double KrotVertB = KBvert * std::pow(rb.Length(), 2.0);

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
