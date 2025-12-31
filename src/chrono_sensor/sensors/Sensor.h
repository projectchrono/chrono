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
// Sensor constructed from a JSON specification file
//
// =============================================================================

#ifndef SENSOR_H
#define SENSOR_H

#include "chrono_sensor/sensors/ChSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// Sensor class for constructing sensors from a JSON specification file.
class CH_SENSOR_API Sensor : public ChSensor {
  public:
    /// Create sensor from specified JSON file
    /// @param filename Path and filename for the JSON file.
    /// @param parent The parent on which the sensor should be attached.
    /// @param offsetPose The position and rotation of the Sensor
    static std::shared_ptr<ChSensor> CreateFromJSON(const std::string& filename,
                                                    std::shared_ptr<chrono::ChBody> parent,
                                                    chrono::ChFrame<double> offsetPose);
};  // class Sensor

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
