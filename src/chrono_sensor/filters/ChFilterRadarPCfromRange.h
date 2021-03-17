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

#ifndef CHFILTERRADARPCFROMRANGE_H
#define CHFILTERRADARPCFROMRANGE_H

#include "chrono_sensor/filters/ChFilter.h"

namespace chrono{
namespace sensor{

// forward declaration
class ChSensor;

class CH_SENSOR_API ChFilterRadarPCfromRange : public ChFilter{
    public:
        ChFilterRadarPCfromRange(std::string name = {});

        virtual void Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

        virtual void Initialize(std::shared_ptr<ChSensor> pSensor){}

    private:
        std::shared_ptr<SensorDeviceXYZIBuffer> m_buffer;

};

} // namespace sensor
} // namespace chrono

#endif