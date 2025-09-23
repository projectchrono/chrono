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
// Torsion-bar suspension system using rotational damper constructed with data
// from file (JSON format)
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/suspension/RotationalDamperSuspension.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

RotationalDamperSuspension::RotationalDamperSuspension(const std::string& filename, bool has_shock, bool lock_arm)
    : ChRotationalDamperSuspension("", has_shock, lock_arm), m_spring_torqueCB(nullptr), m_shock_torqueCB(nullptr) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

RotationalDamperSuspension::RotationalDamperSuspension(const rapidjson::Document& d, bool has_shock, bool lock_arm)
    : ChRotationalDamperSuspension("", has_shock, lock_arm), m_spring_torqueCB(nullptr), m_shock_torqueCB(nullptr) {
    Create(d);
}

RotationalDamperSuspension::~RotationalDamperSuspension() {}

void RotationalDamperSuspension::Create(const rapidjson::Document& d) {
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
    double preload = 0;
    if (d["Torsional Spring"].HasMember("Preload"))
        preload = d["Torsional Spring"]["Preload"].GetDouble();

    if (d["Torsional Spring"].HasMember("Spring Constant")) {
        m_spring_torqueCB = chrono_types::make_shared<utils::LinearSpringTorque>(
            d["Torsional Spring"]["Spring Constant"].GetDouble(), preload);
    } else {
        int num_points = d["Torsional Spring"]["Curve Data"].Size();
        auto springTorqueCB = chrono_types::make_shared<utils::NonlinearSpringTorque>(preload);
        for (int i = 0; i < num_points; i++) {
            springTorqueCB->add_pointK(d["Torsional Spring"]["Curve Data"][i][0u].GetDouble(),
                                       d["Torsional Spring"]["Curve Data"][i][1u].GetDouble());
        }
        m_spring_torqueCB = springTorqueCB;
    }

    // Read rotational shock data
    assert(d.HasMember("Damper"));
    assert(d["Damper"].IsObject());

    if (d["Damper"].HasMember("Damping Coefficient")) {
        double damper_c = d["Damper"]["Damping Coefficient"].GetDouble();
        m_shock_torqueCB = chrono_types::make_shared<utils::LinearDamperTorque>(damper_c);
    } else {
        int num_points = d["Damper"]["Curve Data"].Size();
        auto shockTorqueCB = chrono_types::make_shared<utils::NonlinearDamperTorque>();
        for (int i = 0; i < num_points; i++) {
            shockTorqueCB->add_pointC(d["Damper"]["Curve Data"][i][0u].GetDouble(),
                                      d["Damper"]["Curve Data"][i][1u].GetDouble());
        }
        m_shock_torqueCB = shockTorqueCB;
    }

    // Create the associated road-wheel
    assert(d.HasMember("Road Wheel Input File"));

    std::string file_name = d["Road Wheel Input File"].GetString();
    m_road_wheel = ReadTrackWheelJSON(vehicle::GetDataFile(file_name));
}

}  // end namespace vehicle
}  // end namespace chrono
