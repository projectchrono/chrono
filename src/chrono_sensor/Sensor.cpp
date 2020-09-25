// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Base class for all sensor
//
// =============================================================================

#include "chrono_sensor/Sensor.h"

#include "chrono_sensor/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace sensor {

CH_SENSOR_API std::shared_ptr<ChSensor> Sensor::CreateFromJSON(const std::string& filename,
                                                               std::shared_ptr<chrono::ChBody> parent,
                                                               chrono::ChFrame<double> offsetPose) {
    // Open and parse the input file
    Document d = ReadFileJSON(filename);
    if (d.IsNull())
        return nullptr;

    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    std::string name = d["Name"].GetString();
    std::string type = d["Type"].GetString();
    assert(type.compare("Sensor") == 0);

    // ------------------------------------
    // Create the sensor from the JSON file
    // ------------------------------------
    auto sensor = ReadSensorJSON(filename, parent, offsetPose);
    ReadFilterListJSON(filename, sensor);
    sensor->SetName(name);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";

    return sensor;
}

}  // namespace sensor
}  // namespace chrono
