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

#ifndef CHFILTERGRAYSCALE_H
#define CHFILTERGRAYSCALE_H

#include "chrono_sensor/filters/ChFilter.h"

#include <cuda.h>

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

/// @addtogroup sensor_filters
/// @{

/// A filter that, when applied to a sensor, changes the RGB buffer to grayscale
class CH_SENSOR_API ChFilterGrayscale : public ChFilter {
  public:
    /// Class constructor
    /// @param name String name of the filter
    ChFilterGrayscale(std::string name = {});

    /// Apply function. Converts RGB to Greyscale data.
    virtual void Apply();

    /// Initializes all data needed by the filter apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter. This data is what will be made available for the
    /// user.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<SensorDeviceRGBA8Buffer> m_buffer_in;  ///< input buffer
    std::shared_ptr<SensorDeviceR8Buffer> m_buffer_out;    ///< Buffer for holding the greyscale data
    CUstream m_cuda_stream;                                ///< reference to the cuda stream
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
