
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
// Authors: Radu Serban
// =============================================================================

#include <filesystem>
#include <system_error>

#ifdef _WIN32
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #include <windows.h>
#else
    #include <dlfcn.h>
#endif

#include "chrono/utils/ChConstants.h"

#include "chrono_sensor/utils/ChSensorUtils.h"

namespace chrono {
namespace sensor {

namespace {

// Directory of the module containing this code: the Chrono::Sensor shared library or, for a static build, the
// executable. Empty if it cannot be determined.
std::filesystem::path GetModuleDirectory() {
#ifdef _WIN32
    HMODULE module = nullptr;
    if (GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, reinterpret_cast<LPCWSTR>(&GetModuleDirectory), &module)) {
        std::wstring path(MAX_PATH, L'\0');
        DWORD length;
        while ((length = GetModuleFileNameW(module, path.data(), (DWORD)path.size())) == path.size())
            path.resize(2 * path.size());
        if (length > 0) {
            path.resize(length);
            return std::filesystem::path(path).parent_path();
        }
    }
#else
    Dl_info info;
    if (dladdr(reinterpret_cast<void*>(&GetModuleDirectory), &info) && info.dli_fname != nullptr) {
        std::error_code ec;
        return std::filesystem::absolute(info.dli_fname, ec).parent_path();
    }
#endif
    return {};
}

}  // namespace

std::string LocateSensorDirectory(const std::string& relative_path, const std::string& fallback_path, const std::string& required_entry) {
    if (!relative_path.empty()) {
        const auto module_dir = GetModuleDirectory();
        if (!module_dir.empty()) {
            const auto candidate = (module_dir / relative_path).lexically_normal();
            std::error_code ec;
            if (std::filesystem::is_directory(candidate, ec) && (required_entry.empty() || std::filesystem::exists(candidate / required_entry, ec)))
                return candidate.generic_string();
        }
    }
    return fallback_path;
}

#if defined(CHRONO_HAS_OPTIX) || defined(CHRONO_HAS_VULKAN_RT) || defined(CHRONO_HAS_METAL_RT)

std::string CameraLensModelTypeAsString(CameraLensModelType type) {
    switch (type) {
        case CameraLensModelType::PINHOLE:
            return "Pinhole";
        case CameraLensModelType::FOV_LENS:
            return "Spherical";
        case CameraLensModelType::RADIAL:
            return "Radial";
    }
    return "Unknown camera lens model";
}

std::string CameraNoiseModelTypeAsString(CameraNoiseModelType type) {
    switch (type) {
        case CameraNoiseModelType::NONE:
            return "None";
        case CameraNoiseModelType::CONST_NORMAL:
            return "Const_normal";
        case CameraNoiseModelType::PIXEL_DEPENDENT:
            return "Pixel_dependent";
    }
    return "Unknown noise model";
}

std::string LidarReturnModeAsString(LidarReturnMode mode) {
    switch (mode) {
        case LidarReturnMode::STRONGEST_RETURN:
            return "Strongest_return";
        case LidarReturnMode::MEAN_RETURN:
            return "Mean_return";
        case LidarReturnMode::FIRST_RETURN:
            return "First_return";
        case LidarReturnMode::LAST_RETURN:
            return "Last_return";
        case LidarReturnMode::DUAL_RETURN:
            return "Dual_return";
    }
    return "Unknown lidar return mode";
}

std::string LidarNoiseModelTypeAsString(LidarNoiseModelType type) {
    switch (type) {
        case LidarNoiseModelType::NONE:
            return "None";
        case LidarNoiseModelType::CONST_NORMAL:
            return "Const_normal";
    }
    return "Unknown noise model";
}

std::string LightTypeAsString(LightType type) {
    switch (type) {
        case LightType::POINT_LIGHT:
            return "Point";
        case LightType::SPOT_LIGHT:
            return "Spot";
        case LightType::DIRECTIONAL_LIGHT:
            return "Directional";
        case LightType::RECTANGLE_LIGHT:
            return "Rectangle";
        case LightType::DISK_LIGHT:
            return "Disk";
        case LightType::ENVIRONMENT_LIGHT:
            return "Environment";
        case LightType::AREA_LIGHT:
            return "Area";
    }
    return "Unknown light type";
}

#endif

#if defined(CHRONO_HAS_OPTIX) || defined(CHRONO_HAS_VULKAN_RT) || defined(CHRONO_HAS_METAL_RT)
#endif

}  // namespace sensor
}  // namespace chrono
