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
// Container class for an GPS sensor
//
// =============================================================================

#ifndef CHGPSSENSOR_H
#define CHGPSSENSOR_H

// #include "ChApiSensor.h"

#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/filters/ChFilterGPSUpdate.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_sensor/utils/ChGPSUtils.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// GPS class. This class uses the reference location of a simulation and spherically maps the cartesian space
/// simulation onto a sphere, calculating the latitude, longitude, altitude relative to the Earth's surface.
/// The reference location defines what GPS coordinates are associated with the origin of the simulation. The mapping is
/// performed such that the +Z-axis points up, +X-axis points East, and the +Y-axis points North.
class CH_SENSOR_API ChGPSSensor : public ChDynamicSensor {
  public:
    /// Class constructor
    /// @param parent Body to which the sensor is attached.
    /// @param updateRate Rate at which the sensor should update.
    /// @param offsetPose Relative position and orientation of the sensor with respect to its parent object.
    /// @param lag Lag time between end of data collection and when data becomes available to the user.
    /// @param collection_window Collection time over which the sensor should collect data from the simulation.
    /// @param gps_reference Reference location in GPS coordinates (longitude, latitude, altitude) of simulation origin
    /// @param noise_model The noise model that should be used for augmenting the GPS data.
    ChGPSSensor(std::shared_ptr<chrono::ChBody> parent,
                float updateRate,
                chrono::ChFrame<double> offsetPose,
                ChVector3d gps_reference,
                std::shared_ptr<ChNoiseModel> noise_model);

    /// Class destructor
    ~ChGPSSensor();
    virtual void PushKeyFrame();
    virtual void ClearKeyFrames();

    /// Get the GPS reference location
    const ChVector3d GetGPSReference() const { return m_gps_reference; }

  private:
    /// Variable for communicating the sensor's keyframes from the ChSystem into the data generation filter
    std::vector<std::tuple<float, ChVector3d>> m_keyframes;
    friend class ChFilterGPSUpdate;

    const ChVector3d m_gps_reference;  ///< reference location in GPS coordinates (longitude, latitude, altitude)
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
