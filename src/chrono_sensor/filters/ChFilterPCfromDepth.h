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

#ifndef CHFILTERPCFROMDEPTH_H
#define CHFILTERPCFROMDEPTH_H

#include "chrono_sensor/filters/ChFilter.h"
#include <cuda.h>

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

/// @addtogroup sensor_filters
/// @{

/// A filter that, when applied to a sensor, generates point cloud data from depth values
class CH_SENSOR_API ChFilterPCfromDepth : public ChFilter {
  public:
    /// Class constructor
    /// @param name String name of the filter
    ChFilterPCfromDepth(std::string name = {});

    /// Apply function. Converts data from depth/intensity to point cloud data
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    float m_hFOV;                                          ///< field of view of the parent lidar
    float m_min_vert_angle;                                ///< mimimum vertical angle of parent lidar
    float m_max_vert_angle;                                ///< maximum vertical angle of parent lidar
    CUstream m_cuda_stream;                                ///< reference to the cuda stream
    std::shared_ptr<SensorDeviceDIBuffer> m_buffer_in;     ///< holder of the input buffer
    std::shared_ptr<SensorDeviceXYZIBuffer> m_buffer_out;  ///< holder of the output buffer
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
