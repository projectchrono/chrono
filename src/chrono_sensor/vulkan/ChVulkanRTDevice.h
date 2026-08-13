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
// Experimental Vulkan ray tracing device wrapper.
// =============================================================================

#ifndef CH_VULKAN_RT_DEVICE_H
#define CH_VULKAN_RT_DEVICE_H

#include <memory>
#include <vector>

#include <vulkan/vulkan.h>

#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/vulkan/ChVulkanRTDefinitions.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_vulkan
/// @{

/// Minimal RAII owner for Vulkan objects required by the Sensor RT backend.
/// This class deliberately owns only instance/device/queue state. Acceleration
/// structures, descriptor sets, SBTs, and command buffers are layered above it.
class CH_SENSOR_API ChVulkanRTDevice {
  public:
    explicit ChVulkanRTDevice(const ChVulkanRTDeviceConfig& config = {});
    ~ChVulkanRTDevice();

    ChVulkanRTDevice(const ChVulkanRTDevice&) = delete;
    ChVulkanRTDevice& operator=(const ChVulkanRTDevice&) = delete;

    VkInstance GetInstance() const { return m_instance; }
    VkPhysicalDevice GetPhysicalDevice() const { return m_physical_device; }
    VkDevice GetDevice() const { return m_device; }
    VkQueue GetQueue() const { return m_queue; }
    uint32_t GetQueueFamilyIndex() const { return m_queue_family_index; }

    const VkPhysicalDeviceProperties& GetPhysicalDeviceProperties() const { return m_properties; }
    const ChVulkanRTCapabilities& GetCapabilities() const { return m_capabilities; }

    uint32_t FindMemoryType(uint32_t type_bits, VkMemoryPropertyFlags required_flags) const;

    PFN_vkGetBufferDeviceAddressKHR vkGetBufferDeviceAddressKHR = nullptr;
    PFN_vkCreateAccelerationStructureKHR vkCreateAccelerationStructureKHR = nullptr;
    PFN_vkDestroyAccelerationStructureKHR vkDestroyAccelerationStructureKHR = nullptr;
    PFN_vkGetAccelerationStructureBuildSizesKHR vkGetAccelerationStructureBuildSizesKHR = nullptr;
    PFN_vkGetAccelerationStructureDeviceAddressKHR vkGetAccelerationStructureDeviceAddressKHR = nullptr;
    PFN_vkCmdBuildAccelerationStructuresKHR vkCmdBuildAccelerationStructuresKHR = nullptr;
    PFN_vkCreateRayTracingPipelinesKHR vkCreateRayTracingPipelinesKHR = nullptr;
    PFN_vkGetRayTracingShaderGroupHandlesKHR vkGetRayTracingShaderGroupHandlesKHR = nullptr;
    PFN_vkCmdTraceRaysKHR vkCmdTraceRaysKHR = nullptr;

  private:
    void CreateInstance();
    void PickPhysicalDevice();
    void CreateLogicalDevice();
    void LoadDeviceFunctions();
    bool ProbePhysicalDevice(VkPhysicalDevice physical_device, uint32_t& queue_family_index, ChVulkanRTCapabilities& caps) const;

    ChVulkanRTDeviceConfig m_config;
    VkInstance m_instance = VK_NULL_HANDLE;
    VkPhysicalDevice m_physical_device = VK_NULL_HANDLE;
    VkDevice m_device = VK_NULL_HANDLE;
    VkQueue m_queue = VK_NULL_HANDLE;
    uint32_t m_queue_family_index = 0;
    VkPhysicalDeviceProperties m_properties = {};
    VkPhysicalDeviceMemoryProperties m_memory_properties = {};
    ChVulkanRTCapabilities m_capabilities;
};

/// @} sensor_vulkan

}  // namespace sensor
}  // namespace chrono

#endif
