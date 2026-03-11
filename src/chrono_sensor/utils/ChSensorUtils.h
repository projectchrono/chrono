
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

#ifndef CH_SENSOR_UTILS_H
#define CH_SENSOR_UTILS_H

#include <string>

#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/ChConfigSensor.h"

#ifdef CHRONO_HAS_OPTIX
    #include "chrono_sensor/optix/ChOptixDefinitions.h"
    #include "chrono_sensor/sensors/ChCameraSensor.h"
    #include "chrono_sensor/sensors/ChLidarSensor.h"
    #include "chrono_sensor/filters/ChFilterCameraNoise.h"
    #include "chrono_sensor/filters/ChFilterLidarNoise.h"
#endif

namespace chrono {
namespace sensor {

#ifdef CHRONO_HAS_OPTIX

CH_SENSOR_API std::string CameraLensModelTypeAsString(CameraLensModelType type);
CH_SENSOR_API std::string CameraNoiseModelTypeAsString(CameraNoiseModelType type);

CH_SENSOR_API std::string LidarReturnModeAsString(LidarReturnMode mode);
CH_SENSOR_API std::string LidarNoiseModelTypeAsString(LidarNoiseModelType type);

CH_SENSOR_API std::string LightTypeAsString(LightType type);

#endif

}  // namespace sensor
}  // namespace chrono

#endif
