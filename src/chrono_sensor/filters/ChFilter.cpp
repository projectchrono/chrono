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
// Authors: Eric Brandt
// =============================================================================
//
// =============================================================================

#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/sensors/ChSensor.h"

namespace chrono {
namespace sensor {

void ChFilter::InvalidFilterGraphNullBuffer(std::shared_ptr<ChSensor> pSensor) {
    throw std::runtime_error("Invalid filter graph on sensor [" + pSensor->GetName() + "]. Filter [" + Name() +
                             "] cannot be supplied a null input buffer.");
}

/// Error function for invalid filter graph: type mismatch in graph
void ChFilter::InvalidFilterGraphBufferTypeMismatch(std::shared_ptr<ChSensor> pSensor) {
    throw std::runtime_error("Invalid filter graph on sensor [" + pSensor->GetName() + "]. Filter [" + Name() +
                             "] has incompatible input buffer type.");
}

/// Error function for invalid filter graph: type mismatch in graph
void ChFilter::InvalidFilterGraphSensorTypeMismatch(std::shared_ptr<ChSensor> pSensor) {
    throw std::runtime_error("Invalid filter graph on sensor [" + pSensor->GetName() + "]. Filter [" + Name() +
                             "] was applied to incompatible sensor type.");
}

}  // namespace sensor
}  // namespace chrono
