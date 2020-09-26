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

#ifndef CHFILTERLIDARPOWERCLIP_H
#define CHFILTERLIDARPOWERCLIP_H

#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/ChLidarSensor.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

/// @addtogroup sensor_filters
/// @{

//? a filter that removes points with intensity below a threshold
class CH_SENSOR_API ChFilterLidarIntensityClip : public ChFilter {
  public:
    /// Class constructor
    /// @param intensity_thresh Intensity threshold under which points will be removed
    /// @param default_value default value for distance when intensity is below threshold
    /// @param name The string name of the filter
    ChFilterLidarIntensityClip(float intensity_thresh, float default_value, std::string name = {});

    /// Apply function. Reduces lidar data from raw to processed.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor) {}

  private:
    std::shared_ptr<SensorDeviceDIBuffer>
        m_buffer;              ///< for holding the output buffer -> computations here can be performed in place though
    float m_intensity_thresh;  ///< intensity threshold for clipping data. Can be determined by max distance of lidar
                               ///< for object with 90% return -> see ChLidarSensor.cpp
    float m_default_dist;      ///< default distance value used when intensity fall below threshold
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
