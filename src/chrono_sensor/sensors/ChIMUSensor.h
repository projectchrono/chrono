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

#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/filters/ChFilterIMUUpdate.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// Accelerometer class. The data is collected from the physical quantities
/// computed for the parent object accounting for the offset pose. This sensor operates in lock-step with the Chrono
/// simulation.
class CH_SENSOR_API ChAccelerometerSensor : public ChDynamicSensor {
  public:
    /// Class constructor for an accelerometer sensor
    /// @param parent Body to which the sensor is attached.
    /// @param updateRate Rate at which the sensor should update.
    /// @param offsetPose Relative position and orientation of the sensor with respect to its parent object.
    // @param lag Lag time between end of data collection and when data becomes available to the user.
    // @param collection_window Collection time over which the sensor should collect data from the simulation.
    /// @param noise_model Noise model for the sensor to use when augmentating data
    ChAccelerometerSensor(std::shared_ptr<chrono::ChBody> parent,
                          float updateRate,
                          chrono::ChFrame<double> offsetPose,
                          std::shared_ptr<ChNoiseModel> noise_model);
    /// Class destructor
    ~ChAccelerometerSensor() {}
    virtual void PushKeyFrame();
    virtual void ClearKeyFrames();

  private:
    std::vector<ChVector3d> m_keyframes;  ///< stores keyframes for sensor
    friend class ChFilterAccelerometerUpdate;
};
/// Gyroscope class. The data is collected from the physical quantities
/// computed for the parent object accounting for the offset pose. This sensor operates in lock-step with the Chrono
/// simulation.
class CH_SENSOR_API ChGyroscopeSensor : public ChDynamicSensor {
  public:
    /// Class constructor for a gyroscope
    /// @param parent Body to which the sensor is attached.
    /// @param updateRate Rate at which the sensor should update.
    /// @param offsetPose Relative position and orientation of the sensor with respect to its parent object.
    // @param lag Lag time between end of data collection and when data becomes available to the user.
    // @param collection_window Collection time over which the sensor should collect data from the simulation.
    /// @param noise_model Noise model for the sensor to use when augmentating data
    ChGyroscopeSensor(std::shared_ptr<chrono::ChBody> parent,
                      float updateRate,
                      chrono::ChFrame<double> offsetPose,
                      std::shared_ptr<ChNoiseModel> noise_model);
    /// Class destructor
    ~ChGyroscopeSensor() {}

    virtual void PushKeyFrame();
    virtual void ClearKeyFrames();

  private:
    std::vector<ChVector3d> m_keyframes;  ///< stores keyframes for sensor
    friend class ChFilterGyroscopeUpdate;
};

/// Magnetometer class. The data is collected from the physical quantities
/// computed for the parent object accounting for the offset pose. This sensor operates in lock-step with the Chrono
/// simulation.
class CH_SENSOR_API ChMagnetometerSensor : public ChDynamicSensor {
  public:
    /// Class constructor for a magnetometer
    /// @param parent Body to which the sensor is attached.
    /// @param updateRate Rate at which the sensor should update.
    /// @param offsetPose Relative position and orientation of the sensor with respect to its parent object.
    // @param lag Lag time between end of data collection and when data becomes available to the user.
    // @param collection_window Collection time over which the sensor should collect data from the simulation.
    /// @param noise_model Noise model for the sensor to use when augmentating data
    /// @param gps_reference the GPS reference location for the simulation origin
    ChMagnetometerSensor(std::shared_ptr<chrono::ChBody> parent,
                         float updateRate,
                         chrono::ChFrame<double> offsetPose,
                         std::shared_ptr<ChNoiseModel> noise_model,
                         ChVector3d gps_reference);
    /// Class destructor
    ~ChMagnetometerSensor() {}
    virtual void PushKeyFrame();
    virtual void ClearKeyFrames();

    /// Get the GPS reference location
    const ChVector3d GetGPSReference() const { return m_gps_reference; }

  private:
    std::vector<ChFrame<double>> m_keyframes;
    friend class ChFilterMagnetometerUpdate;

    const ChVector3d m_gps_reference;  ///< reference location in GPS coordinates (longitude, latitude, altitude)
};
/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
