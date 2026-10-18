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

#ifndef CH_VULKAN_SENSOR_H
#define CH_VULKAN_SENSOR_H

#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/vulkan/ChVulkanRTDefinitions.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// Base class for sensors rendered by the experimental Vulkan RT backend.
/// This is intentionally kept independent from ChOptixSensor/CUDA so that the
/// new backend can be integrated without pulling in OptiX headers.
class CH_SENSOR_API ChVulkanSensor : public ChSensor {
  public:
    ChVulkanSensor(std::shared_ptr<ChBody> parent,
                   float updateRate,
                   ChFrame<double> offsetPose,
                   unsigned int w,
                   unsigned int h,
                   VulkanPipelineType pipeline_type = VulkanPipelineType::CAMERA);

    ~ChVulkanSensor() override;

    VulkanPipelineType GetPipelineType() const { return m_pipeline_type; }
    void SetPipelineType(VulkanPipelineType pipeline_type) { m_pipeline_type = pipeline_type; }

    unsigned int GetWidth() const { return m_width; }
    unsigned int GetHeight() const { return m_height; }

  private:
    unsigned int m_width = 0;
    unsigned int m_height = 0;
    VulkanPipelineType m_pipeline_type = VulkanPipelineType::CAMERA;
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
