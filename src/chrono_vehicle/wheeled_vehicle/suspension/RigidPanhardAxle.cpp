// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Solid panhard axle suspension constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "chrono_vehicle/wheeled_vehicle/suspension/RigidPanhardAxle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construct a solid axle suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
RigidPanhardAxle::RigidPanhardAxle(const std::string& filename)
    : ChRigidPanhardAxle(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

RigidPanhardAxle::RigidPanhardAxle(const rapidjson::Document& d)
    : ChRigidPanhardAxle(""), m_springForceCB(NULL), m_shockForceCB(NULL) {
    Create(d);
}

RigidPanhardAxle::~RigidPanhardAxle() {}

// -----------------------------------------------------------------------------
// Worker function for creating a RigidPanhardAxle suspension using data in the
// specified RapidJSON document.
// -----------------------------------------------------------------------------
void RigidPanhardAxle::Create(const rapidjson::Document& d) {
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

    // Read Panhard Rod data
    assert(d.HasMember("Panhard Rod"));
    assert(d["Panhard Rod"].IsObject());

    m_panhardRodMass = d["Panhard Rod"]["Mass"].GetDouble();
    m_panhardRodRadius = d["Panhard Rod"]["Radius"].GetDouble();
    m_panhardRodInertia = ReadVectorJSON(d["Panhard Rod"]["Inertia"]);
    m_points[PANHARD_A] = ReadVectorJSON(d["Panhard Rod"]["Location Axle"]);
    m_points[PANHARD_C] = ReadVectorJSON(d["Panhard Rod"]["Location Chassis"]);

    // Read Anti Roll Bar data
    assert(d.HasMember("Anti Roll Bar"));
    assert(d["Anti Roll Bar"].IsObject());

    m_arbMass = d["Anti Roll Bar"]["Mass"].GetDouble();
    m_arbInertia = ReadVectorJSON(d["Anti Roll Bar"]["Inertia"]);
    m_arbRadius = d["Anti Roll Bar"]["Radius"].GetDouble();
    m_points[ANTIROLL_A] = ReadVectorJSON(d["Anti Roll Bar"]["Location Axle"]);
    m_points[ANTIROLL_C] = ReadVectorJSON(d["Anti Roll Bar"]["Location Chassis"]);
    m_arbStiffness = d["Anti Roll Bar"]["Rotational Stiffness"].GetDouble();
    m_arbDamping = d["Anti Roll Bar"]["Rotational Damping"].GetDouble();

    // Read spring data and create force callback
    assert(d.HasMember("Spring"));
    assert(d["Spring"].IsObject());
    m_points[SPRING_C] = ReadVectorJSON(d["Spring"]["Location Chassis"]);
    m_points[SPRING_A] = ReadVectorJSON(d["Spring"]["Location Axle"]);
    m_springForceCB = ReadTSDAFunctorJSON(d["Spring"], m_springRestLength);

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
}

}  // end namespace vehicle
}  // end namespace chrono
