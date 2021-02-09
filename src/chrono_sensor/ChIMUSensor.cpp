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

#include "chrono_sensor/ChIMUSensor.h"
// #include "chrono_sensor/filters/ChFilterIMUUpdate.h"

namespace chrono {
namespace sensor {

CH_SENSOR_API ChIMUSensor::ChIMUSensor(std::shared_ptr<chrono::ChBody> parent,
                                       float updateRate,
                                       chrono::ChFrame<double> offsetPose,
                                       std::shared_ptr<ChIMUNoiseModel> noise_model)
    : ChSensor(parent, updateRate, offsetPose) {
    m_filters.push_front(chrono_types::make_shared<ChFilterIMUUpdate>(noise_model));
}
CH_SENSOR_API ChIMUSensor::~ChIMUSensor() {}

}  // namespace sensor
}  // namespace chrono
