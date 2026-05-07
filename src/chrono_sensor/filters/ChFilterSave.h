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

#ifndef CHFILTERSAVE_H
#define CHFILTERSAVE_H

#include "chrono_sensor/filters/ChFilter.h"

#include <cuda.h>

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

/// @addtogroup sensor_filters
/// @{

/// A filter that, when applied to a sensor, saves the data as an image.
class CH_SENSOR_API ChFilterSave : public ChFilter {
  public:
    /// Class constructor
    /// @param data_path The path to save the data
    /// @param name the name of the filter
    ChFilterSave(std::string data_path = "", std::string name = "ChFilterSave");

    /// Class destructor
    virtual ~ChFilterSave() {}

    /// Apply function. Saves image data.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

    /// @brief Change the path of where the Save filter saves data
    /// @param data_path The new path string to save sensor data output files
    void ChangeDataPath(std::string data_path);

  private:
    std::string m_path;               ///< path to where data should be saved
    unsigned int m_frame_number = 0;  ///< frame counter to prevent overwriting data

    std::shared_ptr<SensorDeviceRGBA8Buffer> m_rgba8_in;  ///< input buffer for rgba8 image
    std::shared_ptr<SensorHostRGBA8Buffer> m_host_rgba8;  ///< host buffer for rgba8 image

    std::shared_ptr<SensorDeviceRGBA16Buffer> m_rgba16_in;  ///< input buffer for rgba16 image
    std::shared_ptr<SensorHostRGBA16Buffer> m_host_rgba16;  ///< host buffer for rgba16 image

    std::shared_ptr<SensorDeviceR8Buffer> m_r8_in;  ///< input buffer for r8 image
    std::shared_ptr<SensorHostR8Buffer> m_host_r8;  ///< host buffer for r8 image

    std::shared_ptr<SensorHostSemanticBuffer> m_semantic_in;    ///< input buffer for semantic image
    std::shared_ptr<SensorHostSemanticBuffer> m_host_semantic;  ///< host buffer for semantic image

    std::shared_ptr<SensorDeviceDepthBuffer> m_depth_in;    ///< input buffer for depth map (float)
    std::shared_ptr<SensorDeviceDepthBuffer> m_host_depth;  ///< host buffer for depth map (float)

    CUstream m_cuda_stream;  ///< reference to the cuda stream
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
