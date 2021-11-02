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
// This filter converts radar polar coordinates to cartesian coordinates and 
// removes beams with no returns
// =============================================================================
#ifndef CHFILTERRADARXYZRETURN_H
#define CHFILTERRADARXYZRETURN_H

#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include <cuda.h>


namespace chrono{
namespace sensor {

class ChSensor;

class CH_SENSOR_API ChFilterRadarXYZReturn : public ChFilter{
    public:
        ChFilterRadarXYZReturn(std::string name = "ChFilterRadarXYZReturn");
    
    virtual void Apply();

    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

    private:
        std::shared_ptr<ChRadarSensor> m_radar;
        std::shared_ptr<SensorDeviceRadarBuffer> m_buffer_in;
        std::shared_ptr<SensorDeviceRadarXYZBuffer> m_buffer_out;
        CUstream m_cuda_stream; 
        float m_hFOV;
        float m_vFOV;
        float m_min_vert_angle;
};


} // namespace sensor
} // namespace chrono

#endif