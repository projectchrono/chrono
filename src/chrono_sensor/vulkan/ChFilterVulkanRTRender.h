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
// Vulkan RT render filter.
// =============================================================================

#ifndef CH_FILTER_VULKAN_RT_RENDER_H
#define CH_FILTER_VULKAN_RT_RENDER_H

#include <memory>

#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/sensors/ChVulkanSensor.h"
#include "chrono_sensor/vulkan/ChVulkanRTDevice.h"
#include "chrono_sensor/vulkan/ChVulkanRTScene.h"

namespace chrono {
namespace sensor {

struct ChVulkanRTRenderCache;
struct ChVulkanRTGpuRenderer;

/// @addtogroup sensor_vulkan
/// @{

/// Render filter front-end for Vulkan RT sensors.
///
/// The filter produces the same first-order buffer types as the OptiX renderer
/// for camera, depth, normal, and segmentation sensors. It is wired so the
/// Vulkan RT BLAS/TLAS + vkCmdTraceRaysKHR path can replace the host fallback
/// without changing user-level filter graphs.
class CH_SENSOR_API ChFilterVulkanRTRender : public ChFilter {
  public:
    ChFilterVulkanRTRender(std::shared_ptr<ChVulkanRTDevice> device, std::shared_ptr<ChVulkanRTScene> scene);
    ~ChFilterVulkanRTRender() override;

    void Apply() override;
    void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) override;

    void SetRayRecursions(int rec) { m_ray_recursions = rec > 0 ? rec : 1; }

    void SetScene(std::shared_ptr<ChVulkanRTScene> scene) {
        if (m_scene.get() != scene.get())
            m_gpu_renderer.reset();
        m_scene = std::move(scene);
    }

  private:
    std::shared_ptr<ChVulkanRTDevice> m_device;
    std::shared_ptr<ChVulkanRTScene> m_scene;
    std::weak_ptr<ChVulkanSensor> m_sensor;
    std::unique_ptr<ChVulkanRTRenderCache> m_render_cache;
    std::shared_ptr<ChVulkanRTGpuRenderer> m_gpu_renderer;

    std::shared_ptr<SensorHostRGBA8Buffer> m_buffer_rgba8;
    std::shared_ptr<SensorHostRGBDHalf4Buffer> m_buffer_rgbd;
    std::shared_ptr<SensorHostDepthBuffer> m_buffer_depth;
    std::shared_ptr<SensorHostNormalBuffer> m_buffer_normal;
    std::shared_ptr<SensorHostSemanticBuffer> m_buffer_semantic;
    std::shared_ptr<SensorHostDIBuffer> m_buffer_di;
    std::shared_ptr<SensorHostRadarBuffer> m_buffer_radar;

    float m_time_stamp = 0.f;
    int m_ray_recursions = 1;

    friend class ChVulkanRTEngine;
};

/// @} sensor_vulkan

}  // namespace sensor
}  // namespace chrono

#endif
