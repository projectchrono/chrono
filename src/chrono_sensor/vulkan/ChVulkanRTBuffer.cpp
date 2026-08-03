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

#include "chrono_sensor/vulkan/ChVulkanRTBuffer.h"

#include <utility>

#include "chrono_sensor/vulkan/ChVulkanRTUtils.h"

namespace chrono {
namespace sensor {

ChVulkanRTBuffer::ChVulkanRTBuffer(std::shared_ptr<ChVulkanRTDevice> device,
                                   VkDeviceSize size,
                                   VkBufferUsageFlags usage,
                                   VkMemoryPropertyFlags memory_flags)
    : m_device_owner(std::move(device)), m_size(size) {
    VkBufferCreateInfo buffer_info = {};
    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_info.size = size;
    buffer_info.usage = usage;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    CH_VULKAN_CHECK(vkCreateBuffer(m_device_owner->GetDevice(), &buffer_info, nullptr, &m_buffer));

    VkMemoryRequirements requirements = {};
    vkGetBufferMemoryRequirements(m_device_owner->GetDevice(), m_buffer, &requirements);

    VkMemoryAllocateFlagsInfo flags_info = {};
    flags_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO;
    flags_info.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT;

    VkMemoryAllocateInfo alloc_info = {};
    alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc_info.pNext = (usage & VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT) ? &flags_info : nullptr;
    alloc_info.allocationSize = requirements.size;
    alloc_info.memoryTypeIndex = m_device_owner->FindMemoryType(requirements.memoryTypeBits, memory_flags);

    CH_VULKAN_CHECK(vkAllocateMemory(m_device_owner->GetDevice(), &alloc_info, nullptr, &m_memory));
    CH_VULKAN_CHECK(vkBindBufferMemory(m_device_owner->GetDevice(), m_buffer, m_memory, 0));
}

ChVulkanRTBuffer::~ChVulkanRTBuffer() {
    Reset();
}

ChVulkanRTBuffer::ChVulkanRTBuffer(ChVulkanRTBuffer&& other) noexcept {
    *this = std::move(other);
}

ChVulkanRTBuffer& ChVulkanRTBuffer::operator=(ChVulkanRTBuffer&& other) noexcept {
    if (this == &other)
        return *this;

    Reset();
    m_device_owner = std::move(other.m_device_owner);
    m_buffer = other.m_buffer;
    m_memory = other.m_memory;
    m_size = other.m_size;
    m_mapped = other.m_mapped;

    other.m_buffer = VK_NULL_HANDLE;
    other.m_memory = VK_NULL_HANDLE;
    other.m_size = 0;
    other.m_mapped = nullptr;
    return *this;
}

void ChVulkanRTBuffer::Reset() {
    if (!m_device_owner)
        return;

    if (m_mapped)
        Unmap();

    if (m_buffer)
        vkDestroyBuffer(m_device_owner->GetDevice(), m_buffer, nullptr);
    if (m_memory)
        vkFreeMemory(m_device_owner->GetDevice(), m_memory, nullptr);

    m_buffer = VK_NULL_HANDLE;
    m_memory = VK_NULL_HANDLE;
    m_size = 0;
}

VkDeviceAddress ChVulkanRTBuffer::GetDeviceAddress() const {
    VkBufferDeviceAddressInfo info = {};
    info.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
    info.buffer = m_buffer;
    return m_device_owner->vkGetBufferDeviceAddressKHR(m_device_owner->GetDevice(), &info);
}

void* ChVulkanRTBuffer::Map() {
    if (!m_mapped)
        CH_VULKAN_CHECK(vkMapMemory(m_device_owner->GetDevice(), m_memory, 0, m_size, 0, &m_mapped));
    return m_mapped;
}

void ChVulkanRTBuffer::Unmap() {
    if (!m_mapped)
        return;
    vkUnmapMemory(m_device_owner->GetDevice(), m_memory);
    m_mapped = nullptr;
}

}  // namespace sensor
}  // namespace chrono
