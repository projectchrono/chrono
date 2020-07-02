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
// Container class for a camera sensor
//
// =============================================================================

#include "chrono_sensor/ChOptixSensor.h"
#include "chrono_sensor/filters/ChFilterOptixRender.h"

namespace chrono {
namespace sensor {

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChOptixSensor::ChOptixSensor(std::shared_ptr<chrono::ChBody> parent,
                                           double updateRate,
                                           chrono::ChFrame<double> offsetPose,
                                           unsigned int w,
                                           unsigned int h)
    : m_width(w), m_height(h), ChSensor(parent, updateRate, offsetPose) {
    // Camera sensor get rendered by Optix, so they must has as their first filter an optix renderer.
    m_filters.push_front(chrono_types::make_shared<ChFilterOptixRender>());
}

// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChOptixSensor::~ChOptixSensor() {}

}  // namespace sensor
}  // namespace chrono
