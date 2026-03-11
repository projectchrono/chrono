
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

#include "chrono/utils/ChConstants.h"

#include "chrono_sensor/utils/ChSensorUtils.h"

namespace chrono {
namespace sensor {

#ifdef CHRONO_HAS_OPTIX

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

}  // namespace sensor
}  // namespace chrono
