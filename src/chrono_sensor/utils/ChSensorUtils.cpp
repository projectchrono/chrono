
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

std::string CameraLensModelTypeAsString(CameraLensModelType type) {
    switch (type) {
        case PINHOLE:
            return "Pinhole";
        case FOV_LENS:
            return "Spherical";
        case RADIAL:
            return "Radial";
    }
    return "Unknown camera lens model";
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

}  // namespace sensor
}  // namespace chrono
