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
// Experimental Vulkan buffer RAII helper.
// =============================================================================

#ifndef CH_VULKAN_RT_BUFFER_H
#define CH_VULKAN_RT_BUFFER_H

#include <cstddef>
#include <memory>

#include <vulkan/vulkan.h>

#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/vulkan/ChVulkanRTDevice.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_vulkan
/// @{

class CH_SENSOR_API ChVulkanRTBuffer {
  public:
    ChVulkanRTBuffer() = default;
    ChVulkanRTBuffer(std::shared_ptr<ChVulkanRTDevice> device,
                     VkDeviceSize size,
                     VkBufferUsageFlags usage,
                     VkMemoryPropertyFlags memory_flags);
    ~ChVulkanRTBuffer();

    ChVulkanRTBuffer(const ChVulkanRTBuffer&) = delete;
    ChVulkanRTBuffer& operator=(const ChVulkanRTBuffer&) = delete;

    ChVulkanRTBuffer(ChVulkanRTBuffer&& other) noexcept;
    ChVulkanRTBuffer& operator=(ChVulkanRTBuffer&& other) noexcept;

    void Reset();

    VkBuffer GetBuffer() const { return m_buffer; }
    VkDeviceMemory GetMemory() const { return m_memory; }
    VkDeviceSize GetSize() const { return m_size; }
    VkDeviceAddress GetDeviceAddress() const;

    void* Map();
    void Unmap();

  private:
    std::shared_ptr<ChVulkanRTDevice> m_device_owner;
    VkBuffer m_buffer = VK_NULL_HANDLE;
    VkDeviceMemory m_memory = VK_NULL_HANDLE;
    VkDeviceSize m_size = 0;
    void* m_mapped = nullptr;
};

/// @} sensor_vulkan

}  // namespace sensor
}  // namespace chrono

#endif
