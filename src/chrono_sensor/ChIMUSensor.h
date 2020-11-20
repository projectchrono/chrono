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
// Authors: Asher Elmquist
// =============================================================================
//
// Container class for an IMU sensor
//
// =============================================================================

#ifndef CHIMUSENSOR_H
#define CHIMUSENSOR_H

#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/filters/ChFilterIMUUpdate.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// IMU class. Includes an accelerometer and gyroscope. The data is collected from the physical quantities computed for
/// the parent object accounting for the offset pose. This sensor operates in lock-step with the Chrono simulation.
class CH_SENSOR_API ChIMUSensor : public ChSensor {
  public:
    /// Class constructor for an IMU sensor
    /// @param parent Body to which the sensor is attached.
    /// @param updateRate Rate at which the sensor should update.
    /// @param offsetPose Relative position and orientation of the sensor with respect to its parent object.
    /// @param lag Lag time between end of data collection and when data becomes available to the user.
    /// @param collection_window Collection time over which the sensor should collect data from the simulation.
    /// @param noise_model Noise model for the sensor to use when augmentating data
    ChIMUSensor(std::shared_ptr<chrono::ChBody> parent,
                float updateRate,
                chrono::ChFrame<double> offsetPose,
                std::shared_ptr<ChIMUNoiseModel> noise_model);
    /// Class destructor
    ~ChIMUSensor();

    /// Variable for communicating the sensor's keyframes from the ChSystem into the data generation filter
    std::vector<std::tuple<ChVector<float>, ChVector<float>>> imu_key_frames;
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
