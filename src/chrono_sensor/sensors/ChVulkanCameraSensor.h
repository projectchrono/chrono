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

#ifndef CH_VULKAN_CAMERA_SENSOR_H
#define CH_VULKAN_CAMERA_SENSOR_H

#include "chrono_sensor/sensors/ChVulkanSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// Camera sensor front-end for the experimental Vulkan RT backend.
/// This mirrors the public shape of ChCameraSensor enough to start migrating
/// applications without including OptiX/CUDA headers.
class CH_SENSOR_API ChVulkanCameraSensor : public ChVulkanSensor {
  public:
    ChVulkanCameraSensor(std::shared_ptr<chrono::ChBody> parent,
                         float updateRate,
                         chrono::ChFrame<double> offsetPose,
                         unsigned int w,
                         unsigned int h,
                         float hFOV,
                         unsigned int supersample_factor = 1,
                         float gamma = 2.2f);

    ~ChVulkanCameraSensor() override;

    float GetHFOV() const { return m_hFOV; }
    void SetHFOV(float hFOV) { m_hFOV = hFOV; }

    float GetGamma() const { return m_gamma; }
    void SetGamma(float gamma) { m_gamma = gamma; }

    unsigned int GetSampleFactor() const { return m_supersample_factor; }
    void SetSampleFactor(unsigned int sample_factor) { m_supersample_factor = sample_factor; }

  private:
    float m_hFOV = 0.f;
    unsigned int m_supersample_factor = 1;
    float m_gamma = 2.2f;
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
