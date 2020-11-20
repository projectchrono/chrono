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
// Container class for a lidar sensor
//
// =============================================================================

#ifndef CHLIDARSENSOR_H
#define CHLIDARSENSOR_H

#include "chrono_sensor/ChOptixSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// Lidar model type. Currently supports a ray cast model that does not include material properties.
enum LidarModelType {
    RAYCAST  ///< Beams are raycast in the environment with no bounces
};           // TODO: implement PATH_TRACE, material properties

/// Lidar return mode when multiple objects are seen. Currently supported: strongest return (default), and mean return
/// which averages all returned intensity and distance measurement data
enum LidarReturnMode {
    STRONGEST_RETURN,  ///< range at peak intensity
    MEAN_RETURN,       ///< average beam range
    FIRST_RETURN,      ///< shorted beam range
    LAST_RETURN        ///< longest beam range
};

/// Lidar class. This corresponds to a scanning lidar.
class CH_SENSOR_API ChLidarSensor : public ChOptixSensor {
  public:
    /// Constructor for the base lidar class, defaulting to using a single ray per beam
    /// @param parent Body to which the sensor is attached.
    /// @param updateRate Rate at which the sensor should update.
    /// @param offsetPose Relative position and orientation of the sensor with respect to its parent object.
    /// @param w Width in number of samples of a lidar scan
    /// @param h Height in number of sample of a lidar scan (typical the same as laser channels)
    /// @param hfov Horizontal field of view of the lidar
    /// @param max_vertical_angle Maximum vertical angle of the lidar
    /// @param min_vertical_angle Minimum vertical angle of the lidar
    /// @param lag Lag time between end of data collection and when data becomes available to the user.
    /// @param collection_time Collection time over which the sensor should collect data from the simulation.
    /// @param sample_radius The radius in samples for multisampling beams (total samples per beam is 2*radius-1)
    /// @param divergence_angle The divergence angle of the lidar's laser beam
    /// @param return_mode The return mode for lidar data when multiple objects are visible
    /// @param lidar_model The model to be used for generating lidar data
    /// @param clip_near Near clipping distance so that lidar sensor can be easily placed inside a visualization object
    /// (sensor housing)

    ChLidarSensor(std::shared_ptr<chrono::ChBody> parent,
                  float updateRate,
                  chrono::ChFrame<double> offsetPose,
                  unsigned int w,
                  unsigned int h,
                  float hfov,
                  float max_vertical_angle,
                  float min_vertical_angle,
                  float max_distance,
                  unsigned int sample_radius = 1,
                  float divergence_angle = .003,
                  LidarReturnMode return_mode = STRONGEST_RETURN,
                  LidarModelType lidar_model = RAYCAST,
                  float clip_near = 1e-3);

    /// Class destructor
    ~ChLidarSensor();

    /// Gives the horizontal field of view of the lidar (angle between right-most and left-most ray for a "frame").
    /// Horizontal field of view should be 360-(angular resulution) in degrees for a full 360 degree scanning lidar.
    /// @return The horizontal field of view of the lidar sensor
    float GetHFOV() const { return m_hFOV; }  /// returns the lidar's horizontal field of view

    /// Returns the highest vertical angle of any ray in the lidar
    /// @return The angle of the highest ray
    float GetMaxVertAngle() const { return m_max_vert_angle; }

    /// Returns the lowest vertical angle of any ray in the lidar
    /// @return The angle of the lowest ray
    float GetMinVertAngle() const { return m_min_vert_angle; }

    /// Returns the model type being used by the lidar for generating data.
    /// @return The type of model that is being used for generating lidar data
    LidarModelType GetModelType() const { return m_model_type; }

  private:
    float m_hFOV;                   ///< the horizontal field of view of the sensor
    float m_max_vert_angle;         ///< maximum vertical angle of the rays
    float m_min_vert_angle;         ///< minimum vertical angle of the rays
    float m_max_distance;           ///< maximum distance for lidar based on 90% reflectance
    LidarModelType m_model_type;    ///< lens model used by the camera
    unsigned int m_sample_radius;   ///< radius of the beam samples
    float m_divergence_angle;       ///< divergence angle of the beam
    LidarReturnMode m_return_mode;  ///< return mode of the lidar
    float m_clip_near;              ///< near clipping distance so that lidar sensor housings can be transparent to self
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
