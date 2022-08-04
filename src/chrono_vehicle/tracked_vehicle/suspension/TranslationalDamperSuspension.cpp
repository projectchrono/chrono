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

#include "chrono_vehicle/tracked_vehicle/suspension/TranslationalDamperSuspension.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TranslationalDamperSuspension::TranslationalDamperSuspension(const std::string& filename, bool has_shock, bool lock_arm)
    : ChTranslationalDamperSuspension("", has_shock, lock_arm), m_spring_torqueCB(nullptr), m_shock_forceCB(nullptr) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

TranslationalDamperSuspension::TranslationalDamperSuspension(const rapidjson::Document& d, bool has_shock, bool lock_arm)
    : ChTranslationalDamperSuspension("", has_shock, lock_arm), m_spring_torqueCB(nullptr), m_shock_forceCB(nullptr) {
    Create(d);
}

TranslationalDamperSuspension::~TranslationalDamperSuspension() {}

void TranslationalDamperSuspension::Create(const rapidjson::Document& d) {
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

    m_spring_rest_angle = d["Torsional Spring"]["Free Angle"].GetDouble();

    if (d["Torsional Spring"].HasMember("Spring Constant")) {
        double torsion_k = d["Torsional Spring"]["Spring Constant"].GetDouble();
        double torsion_t = d["Torsional Spring"]["Preload"].GetDouble();
        m_spring_torqueCB =
            chrono_types::make_shared<LinearSpringDamperActuatorTorque>(torsion_k, 0.0, torsion_t, m_spring_rest_angle);
    } else {
        int num_points = d["Torsional Spring"]["Curve Data"].Size();
        auto springTorqueCB = chrono_types::make_shared<MapSpringTorque>();
        for (int i = 0; i < num_points; i++) {
            springTorqueCB->add_point(d["Damper"]["Curve Data"][i][0u].GetDouble(),
                                      d["Damper"]["Curve Data"][i][1u].GetDouble());
        }
        m_spring_torqueCB = springTorqueCB;
    }

    if (d["Torsional Spring"].HasMember("Damping Coefficient")) {
        m_damper_torqueCB =
            chrono_types::make_shared<LinearDamperTorque>(d["Torsional Spring"]["Damping Coefficient"].GetDouble());
    }

    // Read linear shock data
    assert(d.HasMember("Damper"));
    assert(d["Damper"].IsObject());

    m_points[SHOCK_C] = ReadVectorJSON(d["Damper"]["Location Chassis"]);
    m_points[SHOCK_A] = ReadVectorJSON(d["Damper"]["Location Arm"]);
    if (d["Damper"].HasMember("Damping Coefficient")) {
        double shock_c = d["Damper"]["Damping Coefficient"].GetDouble();
        m_shock_forceCB = chrono_types::make_shared<LinearDamperForce>(shock_c);
    } else {
        int num_points = d["Damper"]["Curve Data"].Size();
        auto shockForceCB = chrono_types::make_shared<MapDamperForce>();
        for (int i = 0; i < num_points; i++) {
            shockForceCB->add_point(d["Damper"]["Curve Data"][i][0u].GetDouble(),
                                    d["Damper"]["Curve Data"][i][1u].GetDouble());
        }
        m_shock_forceCB = shockForceCB;
    }

    // Create the associated road-wheel
    assert(d.HasMember("Road Wheel Input File"));

    std::string file_name = d["Road Wheel Input File"].GetString();
    m_road_wheel = ReadTrackWheelJSON(vehicle::GetDataFile(file_name));
}

}  // end namespace vehicle
}  // end namespace chrono
