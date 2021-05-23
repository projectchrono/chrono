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
// Authors: Han Wang
// =============================================================================
//
// =============================================================================

#define PROFILE false

#ifndef CHFILTERRADARPROCESS_H
#define CHFILTERRADARPROCESS_H

#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/ChRadarSensor.h"
#include <cuda.h>

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

class CH_SENSOR_API ChFilterRadarProcess : public ChFilter {
  public:
    ChFilterRadarProcess(std::string name = "ChFilterRadarProcess");

    virtual void Apply();

    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<ChRadarSensor> m_radar;
    std::shared_ptr<SensorDeviceRadarBuffer> m_buffer_in;
    std::shared_ptr<SensorHostProcessedRadarBuffer> m_buffer_out;
    CUstream m_cuda_stream;
    float m_hFOV;
    float m_max_vert_angle;
    float m_min_vert_angle;
#if PROFILE
    unsigned int m_scan_number = 0;
#endif
};

}  // namespace sensor
}  // namespace chrono

#endif