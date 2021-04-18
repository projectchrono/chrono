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
// AIDriver specified through a JSON file.
//
// =============================================================================

#include "chrono_vehicle/driver/AIDriver.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// Construct using data from the specified JSON file.
AIDriver::AIDriver(ChVehicle& vehicle, const std::string& filename) : ChAIDriver(vehicle) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    // Read speed controller PID gains
    double Kp = d["Speed Controller"]["Kp"].GetDouble();
    double Ki = d["Speed Controller"]["Ki"].GetDouble();
    double Kd = d["Speed Controller"]["Kd"].GetDouble();
    SetSpeedControllerGains(Kp, Ki, Kd);

    // Read threshold throttle value
    SetThresholdThrottle(d["Speed Controller"]["Threshold Throttle"].GetDouble());

    // Read mapping fronmt wheel angle -> steering input
    const rapidjson::Value& map_data = d["Steering Map"];
    assert(map_data.IsArray());
    unsigned int n = map_data.Size();
    for (unsigned int i = 0; i < n; i++) {
        m_steering_map.AddPoint(map_data[i][0u].GetDouble(), map_data[i][1u].GetDouble());
    }
}

double AIDriver::CalculateSteering(double front_axle_radius, double rear_axle_radius) {
    return m_steering_map.Get_y(front_axle_radius);
}

}  // end namespace vehicle
}  // end namespace chrono
