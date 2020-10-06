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

#include "chrono_sensor/ChGPSSensor.h"
// #include "chrono_sensor/filters/ChFilterGPSUpdate.h"

namespace chrono {
namespace sensor {

CH_SENSOR_API ChGPSSensor::ChGPSSensor(std::shared_ptr<chrono::ChBody> parent,
                                       float updateRate,
                                       chrono::ChFrame<double> offsetPose,
                                       ChVector<double> gps_reference,
                                       std::shared_ptr<ChGPSNoiseModel> noise_model)
    : ChSensor(parent, updateRate, offsetPose) {
    m_filters.push_front(chrono_types::make_shared<ChFilterGPSUpdate>(gps_reference, noise_model));
}
CH_SENSOR_API ChGPSSensor::~ChGPSSensor() {}

}  // namespace sensor
}  // namespace chrono
