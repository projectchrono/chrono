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
// Container class for a camera sensor. This specifies a default ray tracing
// for cameras.
//
// =============================================================================

#ifndef CHOPTIXSENSOR_H
#define CHOPTIXSENSOR_H

#include "chrono_sensor/sensors/ChSensor.h"

#include "chrono_sensor/optix/ChOptixPipeline.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// Optix sensor class - the base class for all sensors that interface with OptiX to generate and render their data
class CH_SENSOR_API ChOptixSensor : public ChSensor {
  public:
    /// Constructor for the base camera class that defaults to a pinhole lens model
    /// @param parent Body to which the sensor is attached.
    /// @param updateRate Rate at which the sensor should update.
    /// @param offsetPose Relative position and orientation of the sensor with respect to its parent object.
    /// @param w Width of the data the sensor should generate
    /// @param h Height of the data the sensor should generate
    // @param lag Lag time between end of data collection and when data becomes available to the user.
    // @param collection_window Collection time over which the sensor should collect data from the simulation.
    ChOptixSensor(std::shared_ptr<chrono::ChBody> parent,
                  float updateRate,
                  chrono::ChFrame<double> offsetPose,
                  unsigned int w,
                  unsigned int h);

    /// camera class destructor
    virtual ~ChOptixSensor();

    PipelineType GetPipelineType() { return m_pipeline_type; }

    unsigned int GetWidth() { return m_width; }
    unsigned int GetHeight() { return m_height; }
    CUstream GetCudaStream() { return m_cuda_stream; }

  protected:
    PipelineType m_pipeline_type;  ///< the type of pipeline for rendering

  private:
    unsigned int m_width;    ///< to hold reference to the width for rendering
    unsigned int m_height;   ///< to hold reference to the height for rendering
    CUstream m_cuda_stream;  ///< cuda stream for this buffer when applicable
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
