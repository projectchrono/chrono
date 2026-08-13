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

#include "chrono_sensor/vulkan/ChVulkanRTDevice.h"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "chrono_sensor/vulkan/ChVulkanRTUtils.h"

namespace chrono {
namespace sensor {
namespace {

std::vector<VkExtensionProperties> EnumerateDeviceExtensions(VkPhysicalDevice physical_device) {
    uint32_t count = 0;
    CH_VULKAN_CHECK(vkEnumerateDeviceExtensionProperties(physical_device, nullptr, &count, nullptr));
    std::vector<VkExtensionProperties> extensions(count);
    if (count)
        CH_VULKAN_CHECK(vkEnumerateDeviceExtensionProperties(physical_device, nullptr, &count, extensions.data()));
    return extensions;
}

}  // namespace

ChVulkanRTDevice::ChVulkanRTDevice(const ChVulkanRTDeviceConfig& config) : m_config(config) {
    CreateInstance();
    PickPhysicalDevice();
    CreateLogicalDevice();
    LoadDeviceFunctions();
}

ChVulkanRTDevice::~ChVulkanRTDevice() {
    if (m_device)
        vkDestroyDevice(m_device, nullptr);
    if (m_instance)
        vkDestroyInstance(m_instance, nullptr);
}

void ChVulkanRTDevice::CreateInstance() {
    VkApplicationInfo app = {};
    app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app.pApplicationName = "Chrono::Sensor Vulkan RT";
    app.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    app.pEngineName = "Chrono::Sensor";
    app.engineVersion = VK_MAKE_VERSION(1, 0, 0);
    app.apiVersion = VK_API_VERSION_1_2;

    std::vector<const char*> layers;
    if (m_config.enable_validation)
        layers.push_back("VK_LAYER_KHRONOS_validation");

    std::vector<const char*> extensions = m_config.extra_instance_extensions;

    VkInstanceCreateInfo create_info = {};
    create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    create_info.pApplicationInfo = &app;
    create_info.enabledLayerCount = static_cast<uint32_t>(layers.size());
    create_info.ppEnabledLayerNames = layers.empty() ? nullptr : layers.data();
    create_info.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
    create_info.ppEnabledExtensionNames = extensions.empty() ? nullptr : extensions.data();

    CH_VULKAN_CHECK(vkCreateInstance(&create_info, nullptr, &m_instance));
}

bool ChVulkanRTDevice::ProbePhysicalDevice(VkPhysicalDevice physical_device,
                                           uint32_t& queue_family_index,
                                           ChVulkanRTCapabilities& caps) const {
    const auto extensions = EnumerateDeviceExtensions(physical_device);
    for (const char* required : ChVulkanRequiredRTDeviceExtensions()) {
        if (!ChVulkanHasExtension(extensions, required))
            return false;
    }
    for (const char* required : m_config.extra_device_extensions) {
        if (!ChVulkanHasExtension(extensions, required))
            return false;
    }

    uint32_t queue_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(physical_device, &queue_count, nullptr);
    std::vector<VkQueueFamilyProperties> queue_families(queue_count);
    vkGetPhysicalDeviceQueueFamilyProperties(physical_device, &queue_count, queue_families.data());

    bool found_queue = false;
    for (uint32_t i = 0; i < queue_count; ++i) {
        if (queue_families[i].queueCount > 0 && (queue_families[i].queueFlags & (VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT))) {
            queue_family_index = i;
            found_queue = true;
            break;
        }
    }
    if (!found_queue)
        return false;

    VkPhysicalDeviceBufferDeviceAddressFeatures bda = {};
    bda.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES;

    VkPhysicalDeviceRayTracingPipelineFeaturesKHR rtp = {};
    rtp.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_FEATURES_KHR;
    rtp.pNext = &bda;

    VkPhysicalDeviceAccelerationStructureFeaturesKHR as = {};
    as.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
    as.pNext = &rtp;

    VkPhysicalDeviceFeatures2 features2 = {};
    features2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    features2.pNext = &as;
    vkGetPhysicalDeviceFeatures2(physical_device, &features2);

    if (!as.accelerationStructure || !rtp.rayTracingPipeline || !bda.bufferDeviceAddress)
        return false;

    VkPhysicalDeviceRayTracingPipelinePropertiesKHR rt_props = {};
    rt_props.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR;

    VkPhysicalDeviceProperties2 properties2 = {};
    properties2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
    properties2.pNext = &rt_props;
    vkGetPhysicalDeviceProperties2(physical_device, &properties2);

    caps.acceleration_structure = as.accelerationStructure == VK_TRUE;
    caps.ray_tracing_pipeline = rtp.rayTracingPipeline == VK_TRUE;
    caps.buffer_device_address = bda.bufferDeviceAddress == VK_TRUE;
    caps.deferred_host_operations = true;
    caps.shader_group_handle_size = rt_props.shaderGroupHandleSize;
    caps.shader_group_base_alignment = rt_props.shaderGroupBaseAlignment;
    caps.shader_group_handle_alignment = rt_props.shaderGroupHandleAlignment;
    caps.max_ray_recursion_depth = rt_props.maxRayRecursionDepth;

    return true;
}

void ChVulkanRTDevice::PickPhysicalDevice() {
    uint32_t count = 0;
    CH_VULKAN_CHECK(vkEnumeratePhysicalDevices(m_instance, &count, nullptr));
    if (!count)
        throw std::runtime_error("No Vulkan physical devices were found");

    std::vector<VkPhysicalDevice> physical_devices(count);
    CH_VULKAN_CHECK(vkEnumeratePhysicalDevices(m_instance, &count, physical_devices.data()));

    uint32_t matching_index = 0;
    for (auto physical_device : physical_devices) {
        uint32_t queue_family = 0;
        ChVulkanRTCapabilities caps;
        if (!ProbePhysicalDevice(physical_device, queue_family, caps))
            continue;

        if (matching_index++ != m_config.device_index)
            continue;

        m_physical_device = physical_device;
        m_queue_family_index = queue_family;
        m_capabilities = caps;
        vkGetPhysicalDeviceProperties(m_physical_device, &m_properties);
        vkGetPhysicalDeviceMemoryProperties(m_physical_device, &m_memory_properties);

        if (m_config.verbose) {
            std::cout << "Chrono::Sensor Vulkan RT device: " << m_properties.deviceName << std::endl;
        }
        return;
    }

    throw std::runtime_error("No Vulkan physical device supports VK_KHR_ray_tracing_pipeline + acceleration_structure");
}

void ChVulkanRTDevice::CreateLogicalDevice() {
    float priority = 1.0f;
    VkDeviceQueueCreateInfo queue_info = {};
    queue_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queue_info.queueFamilyIndex = m_queue_family_index;
    queue_info.queueCount = 1;
    queue_info.pQueuePriorities = &priority;

    VkPhysicalDeviceBufferDeviceAddressFeatures bda = {};
    bda.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES;
    bda.bufferDeviceAddress = VK_TRUE;

    VkPhysicalDeviceRayTracingPipelineFeaturesKHR rtp = {};
    rtp.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_FEATURES_KHR;
    rtp.rayTracingPipeline = VK_TRUE;
    rtp.pNext = &bda;

    VkPhysicalDeviceAccelerationStructureFeaturesKHR as = {};
    as.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
    as.accelerationStructure = VK_TRUE;
    as.pNext = &rtp;

    std::vector<const char*> extensions = ChVulkanRequiredRTDeviceExtensions();
    extensions.insert(extensions.end(), m_config.extra_device_extensions.begin(), m_config.extra_device_extensions.end());

    VkDeviceCreateInfo create_info = {};
    create_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    create_info.pNext = &as;
    create_info.queueCreateInfoCount = 1;
    create_info.pQueueCreateInfos = &queue_info;
    create_info.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
    create_info.ppEnabledExtensionNames = extensions.data();

    CH_VULKAN_CHECK(vkCreateDevice(m_physical_device, &create_info, nullptr, &m_device));
    vkGetDeviceQueue(m_device, m_queue_family_index, 0, &m_queue);
}

void ChVulkanRTDevice::LoadDeviceFunctions() {
#define LOAD_VK_DEVICE_FUNCTION(name)                                                                          \
    name = reinterpret_cast<PFN_##name>(vkGetDeviceProcAddr(m_device, #name));                                \
    if (!name)                                                                                                 \
        throw std::runtime_error(std::string("Failed to load Vulkan device function ") + #name)

    LOAD_VK_DEVICE_FUNCTION(vkGetBufferDeviceAddressKHR);
    LOAD_VK_DEVICE_FUNCTION(vkCreateAccelerationStructureKHR);
    LOAD_VK_DEVICE_FUNCTION(vkDestroyAccelerationStructureKHR);
    LOAD_VK_DEVICE_FUNCTION(vkGetAccelerationStructureBuildSizesKHR);
    LOAD_VK_DEVICE_FUNCTION(vkGetAccelerationStructureDeviceAddressKHR);
    LOAD_VK_DEVICE_FUNCTION(vkCmdBuildAccelerationStructuresKHR);
    LOAD_VK_DEVICE_FUNCTION(vkCreateRayTracingPipelinesKHR);
    LOAD_VK_DEVICE_FUNCTION(vkGetRayTracingShaderGroupHandlesKHR);
    LOAD_VK_DEVICE_FUNCTION(vkCmdTraceRaysKHR);

#undef LOAD_VK_DEVICE_FUNCTION
}

uint32_t ChVulkanRTDevice::FindMemoryType(uint32_t type_bits, VkMemoryPropertyFlags required_flags) const {
    for (uint32_t i = 0; i < m_memory_properties.memoryTypeCount; ++i) {
        const bool type_supported = (type_bits & (1u << i)) != 0;
        const bool flags_supported = (m_memory_properties.memoryTypes[i].propertyFlags & required_flags) == required_flags;
        if (type_supported && flags_supported)
            return i;
    }

    std::ostringstream out;
    out << "No Vulkan memory type satisfies flags 0x" << std::hex << required_flags;
    throw std::runtime_error(out.str());
}

}  // namespace sensor
}  // namespace chrono
