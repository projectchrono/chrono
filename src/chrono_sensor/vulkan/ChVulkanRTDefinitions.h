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
// Experimental Vulkan ray tracing backend definitions for Chrono::Sensor.
// =============================================================================

#ifndef CH_VULKAN_RT_DEFINITIONS_H
#define CH_VULKAN_RT_DEFINITIONS_H

#include <cstdint>
#include <string>
#include <vector>

#include "chrono_sensor/ChApiSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_vulkan
/// @{

/// Vulkan-side equivalent of the OptiX PipelineType enum.
/// Keep this list in lockstep with the public sensor classes while the backend
/// transition is in progress.
enum class VulkanPipelineType {
    CAMERA,
    PHYS_CAMERA,
    SEGMENTATION,
    DEPTH_CAMERA,
    NORMAL_CAMERA,
    LIDAR_SINGLE,
    LIDAR_MULTI,
    RADAR,
};

/// Feature summary captured during device creation.
struct CH_SENSOR_API ChVulkanRTCapabilities {
    bool acceleration_structure = false;
    bool ray_tracing_pipeline = false;
    bool buffer_device_address = false;
    bool deferred_host_operations = false;
    bool descriptor_indexing = false;

    uint32_t shader_group_handle_size = 0;
    uint32_t shader_group_base_alignment = 0;
    uint32_t shader_group_handle_alignment = 0;
    uint64_t max_ray_recursion_depth = 0;
};

/// Vulkan instance/device selection options.
struct CH_SENSOR_API ChVulkanRTDeviceConfig {
    uint32_t device_index = 0;
    bool enable_validation = false;
    bool verbose = false;
    std::vector<const char*> extra_instance_extensions;
    std::vector<const char*> extra_device_extensions;
};

/// @} sensor_vulkan

}  // namespace sensor
}  // namespace chrono

#endif
