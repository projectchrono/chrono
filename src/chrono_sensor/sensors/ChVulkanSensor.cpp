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
// Authors: Florian Reinle
// =============================================================================
// Experimental Vulkan RT sensor base class.
// =============================================================================

#include "chrono_sensor/sensors/ChVulkanSensor.h"

namespace chrono {
namespace sensor {

ChVulkanSensor::ChVulkanSensor(std::shared_ptr<ChBody> parent,
                               float updateRate,
                               ChFrame<double> offsetPose,
                               unsigned int w,
                               unsigned int h,
                               VulkanPipelineType pipeline_type)
    : ChSensor(parent, updateRate, offsetPose), m_width(w), m_height(h), m_pipeline_type(pipeline_type) {}

ChVulkanSensor::~ChVulkanSensor() {}

}  // namespace sensor
}  // namespace chrono
