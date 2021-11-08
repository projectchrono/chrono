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
// Authors: Han Wang
// =============================================================================
//
// Tachometer model is parameterized by :
// 1. parent: the body the sensor is taking measurements
// 2. updateRate: frequency of data acquisition
// 3. axis: Axis of rotatioh to measure (X,Y,Z)
//
// =============================================================================
#ifndef CHTACHOMETER_H
#define CHTACHOMETER_H

#include "chrono_sensor/sensors/ChSensor.h"

namespace chrono {
namespace sensor {


enum Axis {X, Y, Z};

/// @addtogroup sensor_sensors
/// @{

/// Tachometer class. This class queries the chrono system for the angular velocity of the parent body.
class CH_SENSOR_API ChTachometerSensor : public ChDynamicSensor {
  public:
    /// Class constructor
    /// @param parent Body to which the sensor is attached
    /// @param update Rate at which the sensor should update.
    /// @param offsetPose Relative position and orientation of sensor relative to parent body
    /// @param axis Axis of rotation to measure (X,Y,Z)
    ChTachometerSensor(std::shared_ptr<chrono::ChBody> parent, float update, chrono::ChFrame<double> offsetPose, Axis axis);
    /// Class destructor
    ~ChTachometerSensor() {}
    virtual void PushKeyFrame();
    virtual void ClearKeyFrames();

  private:
    /// Variable for communicating the sensor's keyframes from the ChSystem into the data generation filter
    std::vector<ChVector<double>> m_keyframes;   
    friend class ChFilterTachometerUpdate;
    Axis m_axis;
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif