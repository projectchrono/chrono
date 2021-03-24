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

#ifndef CHFILTERLIDARNOISE_H
#define CHFILTERLIDARNOISE_H

#include "chrono_sensor/filters/ChFilter.h"

#include <cuda.h>
#include <curand.h>
#include <curand_kernel.h>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_filters
/// @{

/// A filter that adds noise based on depth and intensity given data in point cloud format
class CH_SENSOR_API ChFilterLidarNoiseXYZI : public ChFilter {
  public:
    /// Class constructor
    /// @param stdev_range Standard deviation of the normal distribution applied to the distance measurement
    /// @param stdev_v_angle Standard deviation of the normal distribution applied to the vertical angle
    /// @param stdev_h_angle Standard deviation of the normal distribution applied to the horizontal angle
    /// @param stdev_intensity Standard deviation of the normal distribution applied to the intensity measurement
    /// @param name String name of the filter
    ChFilterLidarNoiseXYZI(float stdev_range,
                           float stdev_v_angle,
                           float stdev_h_angle,
                           float stdev_intensity,
                           std::string name = "ChFilterLidarNoiseXYZI");

    /// Apply function. Applies noise to lidar data.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    float m_stdev_range;      ///< Standard deviation of the normal distribution applied to the distance measurement
    float m_stdev_v_angle;    ///< Standard deviation of the normal distribution applied to the vertical angle
    float m_stdev_h_angle;    ///< Standard deviation of the normal distribution applied to the horizontal angle
    float m_stdev_intensity;  ///< Standard deviation of the normal distribution applied to the intensity measurement
    std::shared_ptr<curandState_t> m_rng;                   ///< cuda random number generator
    std::shared_ptr<SensorDeviceXYZIBuffer> m_bufferInOut;  ///< buffer for applying noise to point cloud
    CUstream m_cuda_stream;                                 ///< reference to the cuda stream
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
