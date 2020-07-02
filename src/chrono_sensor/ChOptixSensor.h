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

#include "chrono_sensor/ChSensor.h"

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
    /// @param lag Lag time between end of data collection and when data becomes available to the user.
    /// @param collection_window Collection time over which the sensor should collect data from the simulation.
    ChOptixSensor(std::shared_ptr<chrono::ChBody> parent,
                  double updateRate,
                  chrono::ChFrame<double> offsetPose,
                  unsigned int w,
                  unsigned int h);

    /// camera class destructor
    virtual ~ChOptixSensor();

    /// virtual function for getting the string used for generating the ray-laucnh program
    /// @return The combination of file name and function name that specifies the render program
    ProgramString& RenderProgramString() { return m_program_string; }

    /// virtual function of getting the format for the output buffer
    /// @return The buffer format type that the sensor uses.
    RTformat& RenderBufferFormat() { return m_buffer_format; }

    /// virtual function for getting any additional render parameters need by the sensor
    /// @return the list of parameters that need to be passed to the ray launch program of the sensor.
    std::vector<std::tuple<std::string, RTobjecttype, void*>>& RayLaunchParameters() { return m_ray_launch_params; }

  protected:
    ProgramString
        m_program_string;      ///< the string tuple that specifies the file and name of the ray generation program
    RTformat m_buffer_format;  ///< the format of the output buffer
    std::vector<std::tuple<std::string, RTobjecttype, void*>>
        m_ray_launch_params;  ///< holder of any additional ray generation parameters we need to pass to OptiX

  private:
    /// Variables below are for friends only (i.e. they had better know what they are doing to use these)
    /// Reason being that these must be used inside the render engine loop that is on a separate thread.
    /// Since these objects cannot be easily locked and unlocked since they are effectively shared_ptrs.
    optix::Context m_context;      ///< to hold a reference to the OptiX context that is rendering this sensor
    optix::Program m_ray_gen;      ///< to hold a reference to the ray generation program used to generate the data
    optix::Transform m_transform;  ///< ray gen transform

    unsigned int m_width;         ///< to hold reference to the width for rendering
    unsigned int m_height;        ///< to hold reference to the height for rendering
    unsigned int m_launch_index;  ///< for holding the launch index that tells optix which sensor this is
    float m_time_stamp;           ///< time stamp for when the data (render) was launched

    friend class ChFilterOptixRender;  ///< ChFilterOptixRender is allowed to set and use the private members
    friend class ChOptixEngine;        ///< ChOptixEngine is allowed to set and use the private members
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
