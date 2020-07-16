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
                           std::string name = {});

    /// Apply function. Applies noise to lidar data.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor);

  private:
    float m_stdev_range;      ///< Standard deviation of the normal distribution applied to the distance measurement
    float m_stdev_v_angle;    ///< Standard deviation of the normal distribution applied to the vertical angle
    float m_stdev_h_angle;    ///< Standard deviation of the normal distribution applied to the horizontal angle
    float m_stdev_intensity;  ///< Standard deviation of the normal distribution applied to the intensity measurement
    std::shared_ptr<curandState_t> m_rng;  ///< cuda random number generator
    bool m_noise_init = true;              ///< initialize noise only once
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
