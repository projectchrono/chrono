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
// Authors: Han Wang, Asher Elmquist
// =============================================================================
//
// =============================================================================

#define PROFILE false

#ifndef CHFILTERRADARPROCESS_H
    #define CHFILTERRADARPROCESS_H

    #include "chrono_sensor/filters/ChFilter.h"
    #include "chrono_sensor/sensors/ChRadarSensor.h"
    #include <cuda.h>

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

/// @addtogroup sensor_filters
/// @{

/// A filter that, when applied to a sensor, converts the depth values to pointcloud,
/// clusters, and calculates velocity and centroid
class CH_SENSOR_API ChFilterRadarProcess : public ChFilter {
  public:
    /// Class constructor
    /// @param name String name of the filter
    ChFilterRadarProcess(std::string name = "ChFilterRadarProcess");

    /// Apply function. Converts depth to PC, clusters, and calculate centroid and avg velocity of each cluster
    virtual void Apply();

    /// Initializes data needed by the filter
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<ChRadarSensor> m_radar;                        /// radar this filter is attached
    std::shared_ptr<SensorDeviceRadarBuffer> m_buffer_in;          /// holder of the input buffer
    std::shared_ptr<SensorHostRadarXYZBuffer> m_buffer_out;  /// holder of the output buffer
    CUstream m_cuda_stream;                                        /// reference to the cuda stream
    float m_hFOV;                                                  /// horizontal field of view of the radar
    float m_vFOV;                                        /// mimimum vertical angle of the radar
    #if PROFILE
    unsigned int m_scan_number = 0;
    #endif
};

}  // namespace sensor
}  // namespace chrono

#endif