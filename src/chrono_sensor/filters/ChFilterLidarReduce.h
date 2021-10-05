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

#ifndef CHFILTERLIDARREDUCE_H
#define CHFILTERLIDARREDUCE_H

#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

/// @addtogroup sensor_filters
/// @{

//? a filter that reduces lidar from raw to processed when multiple rays per beam are used.
class CH_SENSOR_API ChFilterLidarReduce : public ChFilter {
  public:
    /// Class constructor
    /// @param ret The return mode of the lidar sensor (STRONGEST, FIRST, LAST, etc)
    /// @param reduce_radius The radius of samples per beam
    /// @param name The string name of the filter
    ChFilterLidarReduce(LidarReturnMode ret, int reduce_radius, std::string name = "ChFilterLidarReduce");

    /// Apply function. Reduces lidar data from raw to processed.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<SensorDeviceDIBuffer> m_buffer_in;   ///< for holding the input buffer
    std::shared_ptr<SensorDeviceDIBuffer> m_buffer_out;  ///< for holding the output buffer
    LidarReturnMode m_ret;                               ///< for holding the return mode
    int m_reduce_radius;                                 ///< for holding the sample radius
    CUstream m_cuda_stream;                              ///< reference to the cuda stream
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
