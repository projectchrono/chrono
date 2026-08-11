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
// Experimental Vulkan RT camera sensor.
// =============================================================================

#include "chrono_sensor/sensors/ChVulkanCameraSensor.h"

namespace chrono {
namespace sensor {

ChVulkanCameraSensor::ChVulkanCameraSensor(std::shared_ptr<chrono::ChBody> parent,
                                           float updateRate,
                                           chrono::ChFrame<double> offsetPose,
                                           unsigned int w,
                                           unsigned int h,
                                           float hFOV,
                                           unsigned int supersample_factor,
                                           float gamma)
    : ChVulkanSensor(parent, updateRate, offsetPose, w, h, VulkanPipelineType::CAMERA),
      m_hFOV(hFOV),
      m_supersample_factor(supersample_factor),
      m_gamma(gamma) {
    SetCollectionWindow(0.f);
    SetLag(1.f / updateRate);
}

ChVulkanCameraSensor::~ChVulkanCameraSensor() {}

}  // namespace sensor
}  // namespace chrono
