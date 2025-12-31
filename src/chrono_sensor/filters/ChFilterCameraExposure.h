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
// Authors: Bo-Hsun Chen
// =============================================================================
//
// =============================================================================

#ifndef CHFILTERCAMERAEXPOSURE_H
#define CHFILTERCAMERAEXPOSURE_H

#include "chrono_sensor/filters/ChFilter.h"
#include <cuda.h>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_filters
/// @{

/// A filter that adjust the brightness of the image according to exposure time and sensitivity coefficients
class CH_SENSOR_API ChFilterCameraExposureCorrect : public ChFilter {
  public:
    /// Class constructor
    /// @param a0 coefficient of log2(exposure_time[ms]) in a-term
    /// @param a1 coefficient of src_exposure_mean in a-term
    /// @param a2 constant in a-term
    /// @param b0 coefficient of log2(exposure_time[ms]) in b-term
    /// @param b1 coefficient of src_exposure_mean in b-term
    /// @param b2 constant in b-term
    /// @param expsr_time exposure time [sec]
    /// @param name The string name of the filter
    ChFilterCameraExposureCorrect(float a0, float a1, float a2, float b0, float b1, float b2, float expsr_time,
                                  std::string name = "ChFilterCameraExposureCorrect");

    /// Apply function. Adds uniform Gaussian noise to an image.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter. This data is what will be made available for the
    /// user.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    float m_a0;                                             // coefficient of log2(exposure_time) in a-term
    float m_a1;                                             // coefficient of src_exposure_mean in a-term
    float m_a2;                                             // constant in a-term
    float m_b0;                                             // coefficient of log2(exposure_time) in b-term
    float m_b1;                                             // coefficient of src_exposure_mean in b-term
    float m_b2;                                             // constant in b-term
    float m_a;                                              // a-term
    float m_b;                                              // b-term
    float m_expsr_time;                                     // exposure time [sec]
    std::shared_ptr<SensorDeviceRGBA8Buffer> m_rgba8InOut;  ///< input/output buffer for rgba8
    std::shared_ptr<SensorDeviceR8Buffer> m_r8InOut;        ///< input/output buffer for r8
    CUstream m_cuda_stream;                                 ///< reference to the cuda stream
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
