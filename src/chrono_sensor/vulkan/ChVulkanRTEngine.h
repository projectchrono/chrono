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
// Experimental Vulkan RT engine.
// =============================================================================

#ifndef CH_VULKAN_RT_ENGINE_H
#define CH_VULKAN_RT_ENGINE_H

#include <memory>
#include <vector>

#include "chrono/physics/ChSystem.h"
#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/sensors/ChVulkanSensor.h"
#include "chrono_sensor/vulkan/ChFilterVulkanRTRender.h"
#include "chrono_sensor/vulkan/ChVulkanRTDevice.h"
#include "chrono_sensor/vulkan/ChVulkanRTScene.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_vulkan
/// @{

/// Engine for sensors backed by Vulkan ray tracing.
/// This mirrors ChOptixEngine's manager-facing contract but avoids CUDA/OptiX.
class CH_SENSOR_API ChVulkanRTEngine {
  public:
    ChVulkanRTEngine(ChSystem* sys, int device_id, int max_scene_reflections = 9, bool verbose = false, bool debug = false);
    ~ChVulkanRTEngine();

    void AssignSensor(std::shared_ptr<ChVulkanSensor> sensor);
    void UpdateSensors(std::shared_ptr<ChVulkanRTScene> scene);
    void ConstructScene();

    int GetDevice() const { return static_cast<int>(m_device_id); }
    int GetNumSensor() const { return static_cast<int>(m_assigned_sensors.size()); }
    std::vector<std::shared_ptr<ChVulkanSensor>> GetSensor() const { return m_assigned_sensors; }

    std::shared_ptr<ChVulkanRTDevice> GetVulkanDevice() const { return m_device; }

  private:
    ChSystem* m_system = nullptr;
    uint32_t m_device_id = 0;
    int m_recursions = 1;
    bool m_verbose = false;
    bool m_debug = false;

    std::shared_ptr<ChVulkanRTDevice> m_device;
    std::shared_ptr<ChVulkanRTScene> m_scene;
    std::vector<std::shared_ptr<ChVulkanSensor>> m_assigned_sensors;
    std::vector<std::shared_ptr<ChFilterVulkanRTRender>> m_assigned_renderers;
};

/// @} sensor_vulkan

}  // namespace sensor
}  // namespace chrono

#endif
