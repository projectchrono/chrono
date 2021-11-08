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
// Authors: Asher Elmquist
// =============================================================================
//
//
// =============================================================================

#ifndef CHFILTERSAVEPTCLOUD_H
#define CHFILTERSAVEPTCLOUD_H

#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

/// @addtogroup sensor_filters
/// @{

/// A filter that, when applied to a sensor, saves point cloud data. Format will be CSV one point per line, with data as
/// X,Y,Z,I
class CH_SENSOR_API ChFilterSavePtCloud : public ChFilter {
  public:
    /// Class constructor
    /// @param data_path Path to where data should be saved
    /// @param name the name of the filter
    ChFilterSavePtCloud(std::string data_path = "", std::string name = "ChFilterSavePtCloud");

    /// Class destructor
    virtual ~ChFilterSavePtCloud();

    /// Apply function. Saves point cloud data in CSV file format.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::string m_path;                                   ///< path to saved data
    unsigned int m_frame_number = 0;                      ///< frame counter for saving sequential frames
    std::shared_ptr<SensorDeviceXYZIBuffer> m_buffer_in;  ///< input buffer for point cloud
    std::shared_ptr<SensorHostXYZIBuffer> m_host_buffer;  ///< input buffer for point cloud
    CUstream m_cuda_stream;
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
