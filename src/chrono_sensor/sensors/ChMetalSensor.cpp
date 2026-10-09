// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Kyle Sha
// =============================================================================

#include "chrono_sensor/sensors/ChMetalSensor.h"

namespace chrono {
namespace sensor {

ChMetalSensor::ChMetalSensor(std::shared_ptr<ChBody> parent, float updateRate, ChFrame<double> offsetPose, unsigned int w, unsigned int h, MetalPipelineType pipeline_type)
    : ChSensor(parent, updateRate, offsetPose), m_width(w), m_height(h), m_pipeline_type(pipeline_type) {}

ChMetalSensor::~ChMetalSensor() {}

}  // namespace sensor
}  // namespace chrono
