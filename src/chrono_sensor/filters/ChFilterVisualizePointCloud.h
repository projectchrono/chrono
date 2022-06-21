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

#ifndef CHFILTERVISUALIZEPOINTCLOUD_H
#define CHFILTERVISUALIZEPOINTCLOUD_H

#include "chrono_sensor/filters/ChFilterVisualize.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

/// @addtogroup sensor_filters
/// @{

/// A filter that, when applied to a sensor, creates a GUI window to visualize the sensor (using GLFW). This visualizes
/// data as a point cloud. Will only work on data that can be interpreted as point cloud data.
class CH_SENSOR_API ChFilterVisualizePointCloud : public ChFilterVisualize {
  public:
    /// Class constructor
    /// @param w Width of the window to create
    /// @param h Height of the window to create
    /// @param zoom Value to multiply by the default box in which points are viewed
    /// @param name String name of the filter
    ChFilterVisualizePointCloud(int w, int h, float zoom, std::string name = "ChFilterVisualizePointCloud");

    /// Class destructor
    virtual ~ChFilterVisualizePointCloud();

    /// Apply function. Visualizes data as an image.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    float m_zoom;  ///< value for setting the zoom factor of the visualization box
    std::shared_ptr<SensorDeviceXYZIBuffer> m_buffer_in;  ///< input buffer
    std::shared_ptr<SensorHostXYZIBuffer> m_host_buffer;  ///< host side buffer for visualization
    CUstream m_cuda_stream;                               ///< reference to the cuda stream
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
