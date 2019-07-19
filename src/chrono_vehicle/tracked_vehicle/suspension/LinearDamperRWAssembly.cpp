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
// Torsion-bar suspension system using linear dampers constructed with data from
// file (JSON format)
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/suspension/LinearDamperRWAssembly.h"
//
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
//
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
LinearDamperRWAssembly::LinearDamperRWAssembly(const std::string& filename, bool has_shock)
    : ChLinearDamperRWAssembly("", has_shock), m_spring_torqueCB(nullptr), m_shock_forceCB(nullptr) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

LinearDamperRWAssembly::LinearDamperRWAssembly(const rapidjson::Document& d, bool has_shock)
    : ChLinearDamperRWAssembly("", has_shock), m_spring_torqueCB(nullptr), m_shock_forceCB(nullptr) {
    Create(d);
}

LinearDamperRWAssembly::~LinearDamperRWAssembly() {
    delete m_shock_forceCB;
    delete m_spring_torqueCB;
}

void LinearDamperRWAssembly::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read suspension arm data
    assert(d.HasMember("Suspension Arm"));
    assert(d["Suspension Arm"].IsObject());

    m_arm_mass = d["Suspension Arm"]["Mass"].GetDouble();
    m_points[ARM] = ReadVectorJSON(d["Suspension Arm"]["COM"]);
    m_arm_inertia = ReadVectorJSON(d["Suspension Arm"]["Inertia"]);
    m_points[ARM_CHASSIS] = ReadVectorJSON(d["Suspension Arm"]["Location Chassis"]);
    m_points[ARM_WHEEL] = ReadVectorJSON(d["Suspension Arm"]["Location Wheel"]);
    m_arm_radius = d["Suspension Arm"]["Radius"].GetDouble();

    // Read data for torsional spring
    assert(d.HasMember("Torsional Spring"));
    assert(d["Torsional Spring"].IsObject());

    double torsion_k = d["Torsional Spring"]["Spring Constant"].GetDouble();
    double torsion_c = d["Torsional Spring"]["Damping Coefficient"].GetDouble();
    double torsion_t = d["Torsional Spring"]["Preload"].GetDouble();
    m_spring_torqueCB = new LinearSpringDamperActuatorTorque(torsion_k, torsion_c, torsion_t);

    // Read linear shock data
    assert(d.HasMember("Damper"));

    m_points[SHOCK_C] = ReadVectorJSON(d["Damper"]["Location Chassis"]);
    m_points[SHOCK_A] = ReadVectorJSON(d["Damper"]["Location Arm"]);
    double shock_c = d["Damper"]["Damping Coefficient"].GetDouble();
    m_shock_forceCB = new LinearDamperForce(shock_c);

    // Create the associated road-wheel
    assert(d.HasMember("Road Wheel Input File"));

    std::string file_name = d["Road Wheel Input File"].GetString();
    m_road_wheel = ReadRoadWheelJSON(vehicle::GetDataFile(file_name));
}

}  // end namespace vehicle
}  // end namespace chrono
