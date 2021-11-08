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
#include "chrono_sensor/sensors/ChTachometerSensor.h"
#include "chrono_sensor/filters/ChFilterTachometerUpdate.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace sensor {

CH_SENSOR_API ChTachometerSensor::ChTachometerSensor(std::shared_ptr<chrono::ChBody> parent,
                                                     float updateRate,
                                                     chrono::ChFrame<double> offsetPose,
                                                     Axis axis)
    :m_axis(axis), ChDynamicSensor(parent, updateRate, offsetPose) {
    m_filters.push_front(chrono_types::make_shared<ChFilterTachometerUpdate>());
}
CH_SENSOR_API void ChTachometerSensor::PushKeyFrame() {
    ChVector<double> rot_speed = m_parent->GetWvel_loc();
    m_keyframes.push_back(rot_speed);
}

CH_SENSOR_API void ChTachometerSensor::ClearKeyFrames() {
    m_keyframes.clear();
}

}  // namespace sensor
}  // namespace chrono