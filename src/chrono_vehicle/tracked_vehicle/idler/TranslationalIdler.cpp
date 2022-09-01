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
// Translational idler model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/idler/TranslationalIdler.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

TranslationalIdler::TranslationalIdler(const std::string& filename) : ChTranslationalIdler("") {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

TranslationalIdler::TranslationalIdler(const rapidjson::Document& d) : ChTranslationalIdler("") {
    Create(d);
}

void TranslationalIdler::Create(const rapidjson::Document& d) {
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
    m_points[TSDA_CARRIER] = ReadVectorJSON(d["Tensioner"]["Location Carrier"]);
    m_points[TSDA_CHASSIS] = ReadVectorJSON(d["Tensioner"]["Location Chassis"]);
    m_tensioner_l0 = d["Tensioner"]["Free Length"].GetDouble();
    double tensioner_f = d["Tensioner"]["Preload"].GetDouble();
    if (d["Tensioner"].HasMember("Spring Coefficient")) {
        // Linear spring-damper
        double tensioner_k = d["Tensioner"]["Spring Coefficient"].GetDouble();
        double tensioner_c = d["Tensioner"]["Damping Coefficient"].GetDouble();
        m_tensionerForceCB = chrono_types::make_shared<LinearSpringDamperForce>(tensioner_k, tensioner_c, tensioner_f);
    } else if (d["Tensioner"].HasMember("Spring Curve Data")) {
        // Nonlinear (curves) spring-damper
        int num_pointsK = d["Tensioner"]["Spring Curve Data"].Size();
        int num_pointsC = d["Tensioner"]["Damper Curve Data"].Size();
        auto tensionerForceCB = chrono_types::make_shared<MapSpringDamperForce>(tensioner_f);
        for (int i = 0; i < num_pointsK; i++) {
            tensionerForceCB->add_pointK(d["Tensioner"]["Spring Curve Data"][i][0u].GetDouble(),
                                         d["Tensioner"]["Spring Curve Data"][i][1u].GetDouble());
        }
        for (int i = 0; i < num_pointsC; i++) {
            tensionerForceCB->add_pointC(d["Tensioner"]["Damper Curve Data"][i][0u].GetDouble(),
                                         d["Tensioner"]["Damper Curve Data"][i][1u].GetDouble());
        }
        m_tensionerForceCB = tensionerForceCB;
    }

    // Create the associated road-wheel
    assert(d.HasMember("Idler Wheel Input File"));

    std::string file_name = d["Idler Wheel Input File"].GetString();
    m_idler_wheel = ReadTrackWheelJSON(vehicle::GetDataFile(file_name));
}

}  // end namespace vehicle
}  // end namespace chrono
