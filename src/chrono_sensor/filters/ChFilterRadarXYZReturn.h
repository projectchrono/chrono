// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Han Wang, Asher Elmquist
// =============================================================================
// This filter converts radar polar coordinates to Cartesian coordinates and
// removes beams with no returns
// =============================================================================
#ifndef CHFILTERRADARXYZRETURN_H
#define CHFILTERRADARXYZRETURN_H

#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"

#ifdef CHRONO_HAS_OPTIX
#include <cuda.h>
#endif

namespace chrono {
namespace sensor {

class ChSensor;

class CH_SENSOR_API ChFilterRadarXYZReturn : public ChFilter {
  public:
    ChFilterRadarXYZReturn(std::string name = "ChFilterRadarXYZReturn");
    virtual void Apply();
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    float m_hFOV = 0.f;
    float m_vFOV = 0.f;
    std::shared_ptr<ChRadarSensor> m_radar;
    std::shared_ptr<SensorDeviceRadarBuffer> m_buffer_in;
    std::shared_ptr<SensorDeviceRadarXYZBuffer> m_buffer_out;
#ifdef CHRONO_HAS_OPTIX
    CUstream m_cuda_stream;
#endif
};

}  // namespace sensor
}  // namespace chrono

#endif