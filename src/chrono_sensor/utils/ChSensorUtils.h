
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
#include "chrono_sensor/optix/ChOptixDefinitions.h"

namespace chrono {
namespace sensor {
  
CH_SENSOR_API std::string CameraLensModelTypeAsString(CameraLensModelType type);

CH_SENSOR_API std::string LightTypeAsString(LightType type);

}  // namespace sensor
}  // namespace chrono

#endif
