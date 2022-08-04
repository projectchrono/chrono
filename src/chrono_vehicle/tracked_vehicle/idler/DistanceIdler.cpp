// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
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
// Distance idler model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/idler/DistanceIdler.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

DistanceIdler::DistanceIdler(const std::string& filename) : ChDistanceIdler("") {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

DistanceIdler::DistanceIdler(const rapidjson::Document& d) : ChDistanceIdler("") {
    Create(d);
}

void DistanceIdler::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read carrier geometry and mass properties
    assert(d.HasMember("Carrier"));
    m_carrier_mass = d["Carrier"]["Mass"].GetDouble();
    m_carrier_inertia = ReadVectorJSON(d["Carrier"]["Inertia"]);
    m_points[CARRIER] = ReadVectorJSON(d["Carrier"]["COM"]);
    m_points[CARRIER_CHASSIS] = ReadVectorJSON(d["Carrier"]["Location Chassis"]);
    m_points[CARRIER_WHEEL] = ReadVectorJSON(d["Carrier"]["Location Wheel"]);
    m_carrier_vis_radius = d["Carrier"]["Visualization Radius"].GetDouble();
    m_pitch_angle = d["Carrier"]["Pitch Angle"].GetDouble();

    // Read tensioner data
    assert(d.HasMember("Tensioner"));
    m_points[MOTOR_CARRIER] = ReadVectorJSON(d["Tensioner"]["Location Carrier"]);
    m_points[MOTOR_ARM] = ReadVectorJSON(d["Tensioner"]["Location Arm"]);
    m_tensioner_time = d["Tensioner"]["Extension time"].GetDouble();
    m_tensioner_dist = d["Tensioner"]["Extension distance"].GetDouble();

    // Create the associated road-wheel
    assert(d.HasMember("Idler Wheel Input File"));

    std::string file_name = d["Idler Wheel Input File"].GetString();
    m_idler_wheel = ReadTrackWheelJSON(vehicle::GetDataFile(file_name));
}

}  // end namespace vehicle
}  // end namespace chrono
