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

#ifndef CHFILTERCAMERANOISE_H
#define CHFILTERCAMERANOISE_H

#include "chrono_sensor/filters/ChFilter.h"

#include <cuda.h>
#include <curand.h>
#include <curand_kernel.h>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_filters
/// @{

/// A filter that adds Gaussian noise across an image with constant mean and standard deviation
class CH_SENSOR_API ChFilterCameraNoiseConstNormal : public ChFilter {
  public:
    /// Class constructor
    /// @param mean The mean value of the Guassian distribution
    /// @param stdev The standard deviation of the Gaussian distribution
    /// @param name The string name of the filter.
    ChFilterCameraNoiseConstNormal(float mean, float stdev, std::string name = "ChFilterCameraNoiseConstNormal");

    /// Apply function. Adds uniform Gaussian noise to an image.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter. This data is what will be made available for the
    /// user.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    float m_mean;                                           ///< mean value of the Guassian distribution
    float m_stdev;                                          ///< standard deviation of the Gaussian distribution
    std::shared_ptr<curandState_t> m_rng;                   ///< cuda random number generator
    bool m_noise_init = true;                               ///< initialize noise only once
    std::shared_ptr<SensorDeviceRGBA8Buffer> m_rgba8InOut;  ///< input/output buffer for rgba8
    std::shared_ptr<SensorDeviceR8Buffer> m_r8InOut;        ///< input/output buffer for r8
    CUstream m_cuda_stream;                                 ///< reference to the cuda stream
};

/// A filter that adds pixel dependent gaussian noise across an image. Method summarized in paper: ()
class CH_SENSOR_API ChFilterCameraNoisePixDep : public ChFilter {
  public:
    /// Class constructor
    /// @param variance_slope The standard deviation of the multiplicative noise
    /// @param variance_intercept The standard deviation of the additive noise
    /// @param name The string name of the filter.
    ChFilterCameraNoisePixDep(float variance_slope,
                              float variance_intercept,
                              std::string name = "ChFilterCameraNoisePixDep");

    /// Apply function. Adds uniform Gaussian noise to an image.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter. This data is what will be made available for the
    /// user.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    float m_variance_slope;                                 ///< The variance of the multiplicative noise
    float m_variance_intercept;                             ///< The variance of the additive noise
    std::shared_ptr<curandState_t> m_rng;                   ///< cuda random number generator
    bool m_noise_init = true;                               ///< initialize noise only once
    std::shared_ptr<SensorDeviceRGBA8Buffer> m_rgba8InOut;  ///< input/output buffer for rgba8
    std::shared_ptr<SensorDeviceR8Buffer> m_r8InOut;        ///< input/output buffer for r8
    CUstream m_cuda_stream;                                 ///< reference to the cuda stream
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
