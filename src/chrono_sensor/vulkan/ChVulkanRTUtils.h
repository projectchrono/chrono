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
// Experimental Vulkan ray tracing utility helpers.
// =============================================================================

#ifndef CH_VULKAN_RT_UTILS_H
#define CH_VULKAN_RT_UTILS_H

#include <stdexcept>
#include <string>
#include <vector>

#include <vulkan/vulkan.h>

#include "chrono_sensor/ChApiSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_vulkan
/// @{

CH_SENSOR_API const char* ChVulkanResultString(VkResult result);
CH_SENSOR_API void ChVulkanCheck(VkResult result, const char* expression, const char* file, int line);

CH_SENSOR_API bool ChVulkanHasExtension(const std::vector<VkExtensionProperties>& extensions, const char* name);
CH_SENSOR_API std::vector<const char*> ChVulkanRequiredRTDeviceExtensions();

#define CH_VULKAN_CHECK(expr) ::chrono::sensor::ChVulkanCheck((expr), #expr, __FILE__, __LINE__)

/// @} sensor_vulkan

}  // namespace sensor
}  // namespace chrono

#endif
