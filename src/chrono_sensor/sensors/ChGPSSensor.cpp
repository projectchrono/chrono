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

#include "chrono_sensor/sensors/ChGPSSensor.h"

namespace chrono {
namespace sensor {

ChGPSSensor::ChGPSSensor(std::shared_ptr<ChBody> parent,
                         float updateRate,
                         ChFrame<double> offsetPose,
                         ChVector3d gps_reference,
                         std::shared_ptr<ChNoiseModel> noise_model)
    : ChDynamicSensor(parent, updateRate, offsetPose), m_gps_reference(gps_reference) {
    m_filters.push_front(chrono_types::make_shared<ChFilterGPSUpdate>(gps_reference, noise_model));
}

ChGPSSensor::~ChGPSSensor() {}

void ChGPSSensor::PushKeyFrame() {
    ChVector3d pos_data = m_parent->TransformPointLocalToParent(m_offsetPose.GetPos());
    m_keyframes.push_back(std::make_tuple((float)m_parent->GetSystem()->GetChTime(), pos_data));
}

void ChGPSSensor::ClearKeyFrames() {
    m_keyframes.clear();
}

}  // namespace sensor
}  // namespace chrono
