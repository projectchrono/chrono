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

#include "chrono_vehicle/tracked_vehicle/suspension/RotationalDamperRWAssembly.h"
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
RotationalDamperRWAssembly::RotationalDamperRWAssembly(const std::string& filename, bool has_shock)
    : ChRotationalDamperRWAssembly("", has_shock), m_spring_torqueCB(nullptr), m_shock_torqueCB(nullptr) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

RotationalDamperRWAssembly::RotationalDamperRWAssembly(const rapidjson::Document& d, bool has_shock)
    : ChRotationalDamperRWAssembly("", has_shock), m_spring_torqueCB(nullptr), m_shock_torqueCB(nullptr) {
    Create(d);
}

RotationalDamperRWAssembly::~RotationalDamperRWAssembly() {
    delete m_shock_torqueCB;
    delete m_spring_torqueCB;
}

void RotationalDamperRWAssembly::Create(const rapidjson::Document& d) {
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

    ////double torsion_a0 = d["Torsional Spring"]["Free Angle"].GetDouble();
    double torsion_k = d["Torsional Spring"]["Spring Constant"].GetDouble();
    double torsion_c = d["Torsional Spring"]["Damping Coefficient"].GetDouble();
    double torsion_t = d["Torsional Spring"]["Preload"].GetDouble();
    m_spring_torqueCB = new LinearSpringDamperActuatorTorque(torsion_k, torsion_c, torsion_t);

    // Read linear shock data
    assert(d.HasMember("Damper"));
    assert(d["Damper"].IsObject());

    if (d["Damper"].HasMember("Damping Coefficient")) {
        double damper_c = d["Damper"]["Damping Coefficient"].GetDouble();
        m_shock_torqueCB = new LinearDamperTorque(damper_c);
    } else {
        int num_points = d["Damper"]["Curve Data"].Size();
        MapDamperTorque* shockTorqueCB = new MapDamperTorque();
        for (int i = 0; i < num_points; i++) {
            shockTorqueCB->add_point(d["Damper"]["Curve Data"][i][0u].GetDouble(),
                                     d["Damper"]["Curve Data"][i][1u].GetDouble());
        }
        m_shock_torqueCB = shockTorqueCB;
    }

    // Create the associated road-wheel
    assert(d.HasMember("Road Wheel Input File"));

    std::string file_name = d["Road Wheel Input File"].GetString();
    m_road_wheel = ReadRoadWheelJSON(vehicle::GetDataFile(file_name));
}

}  // end namespace vehicle
}  // end namespace chrono
