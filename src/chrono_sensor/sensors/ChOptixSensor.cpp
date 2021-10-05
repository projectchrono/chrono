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
// Container class for a sensor which uses optix for rendering
//
// =============================================================================

#include "chrono_sensor/sensors/ChOptixSensor.h"

namespace chrono {
namespace sensor {

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChOptixSensor::ChOptixSensor(std::shared_ptr<chrono::ChBody> parent,
                                           float updateRate,
                                           chrono::ChFrame<double> offsetPose,
                                           unsigned int w,
                                           unsigned int h)
    : m_width(w), m_height(h), ChSensor(parent, updateRate, offsetPose) {
    // Camera sensor get rendered by Optix, so they must has as their first filter an optix renderer.
    cudaStreamCreate(&m_cuda_stream);  // all gpu operations will happen on this stream

    // delayed creation of the optix render filter -> ChOptixEngine must do this to properly initialize the optix
    // parameters
}

// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChOptixSensor::~ChOptixSensor() {
    cudaStreamDestroy(m_cuda_stream);
}

}  // namespace sensor
}  // namespace chrono
