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
// Authors: Eric Brandt, Asher Elmquist
// =============================================================================
//
// =============================================================================

#ifndef CHFILTEROPTIXRENDER_H
#define CHFILTEROPTIXRENDER_H

#include <memory>
#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/optixcpp/ChOptixUtils.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

/// @addtogroup sensor_filters
/// @{

/// A filter that generates data for a ChOptixSensor
class CH_SENSOR_API ChFilterOptixRender : public ChFilter {
  public:
    /// Class constructor
    ChFilterOptixRender();

    /// Apply function. Generates data for ChOptixSensors
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor);

  private:
    /// Allocates a buffer for the ChOptixSensor
    optix::Buffer AllocateBuffer(std::shared_ptr<ChSensor> pSensor);

    /// Finds visual filters
    std::shared_ptr<ChFilterVisualize> FindOnlyVisFilter(std::shared_ptr<ChSensor> pSensor);

    std::shared_ptr<SensorOptixBuffer> m_buffer;  ///< for holding the output buffer
    optix::Program m_program;                     ///< the render program used in optix
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
