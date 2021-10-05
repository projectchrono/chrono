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

#include "chrono_sensor/sensors/ChOptixSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// Lidar return mode when multiple objects are seen. Currently supported: strongest return (default), and mean return
/// which averages all returned intensity and distance measurement data
enum class LidarReturnMode {
    STRONGEST_RETURN,  ///< range at peak intensity
    MEAN_RETURN,       ///< average beam range
    FIRST_RETURN,      ///< shortest beam range
    LAST_RETURN,       ///< longest beam range
    DUAL_RETURN        ///< first and strongest returns
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
    /// @param beam_shape Shape of lidar beam, only rectangules and ellipse are supported currently.
    /// @param sample_radius The radius in samples for multisampling beams (total samples per beam is 2*radius-1)
    /// @param vert_divergence_angle The vertical divergence angle of the lidar's laser beam
    /// @param hori_divergence_angle The horizontal divergence angle of the lidar's laser beam
    /// @param return_mode The return mode for lidar data when multiple objects are visible
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
                  LidarBeamShape beam_shape = LidarBeamShape::RECTANGULAR,
                  unsigned int sample_radius = 1,
                  float vert_divergence_angle = .003f,
                  float hori_divergence_angle = .003f,
                  LidarReturnMode return_mode = LidarReturnMode::MEAN_RETURN,
                  float clip_near = 1e-3f);

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

    /// Returns the maximum range of the lidar
    /// @return The maximum distance for the lidar
    float GetMaxDistance() const { return m_max_distance; }

    /// Returns the near clipping distance
    /// @return the near clipping distance
    float GetClipNear() const { return m_clip_near; }

    /// Returns the beam cross section shape
    /// @return the beam cross section shape
    LidarBeamShape GetBeamShape() const { return m_beam_shape; }

    /// Returns the beam sample radius
    /// @return the beam sample radius
    float GetSampleRadius() const { return m_sample_radius; }

    /// Returns the horizontal beam divergence angle
    /// @return the horizontal beam divergence angle
    float GetHorizDivAngle() const { return m_hori_divergence_angle; }

    /// Returns the vertical beam divergence angle
    /// @return the vertical beam divergence angle
    float GetVertDivAngle() const { return m_vert_divergence_angle; }

    bool DualReturnFlag() const {
      switch (m_return_mode){
      case LidarReturnMode::DUAL_RETURN:
        return true;
      default:
        return false;
      }
    }
    


  private:
    float m_hFOV;                   ///< the horizontal field of view of the sensor
    float m_max_vert_angle;         ///< maximum vertical angle of the rays
    float m_min_vert_angle;         ///< minimum vertical angle of the rays
    float m_max_distance;           ///< maximum distance for lidar based on 90% reflectance
    unsigned int m_sample_radius;   ///< radius of the beam samples
    LidarBeamShape m_beam_shape;    ///< lidar beam shape
    float m_vert_divergence_angle;  ///< vertical divergence angle of the beam
    float m_hori_divergence_angle;  ///< horizontal divergence angle of the beam
    LidarReturnMode m_return_mode;  ///< return mode of the lidar
    float m_clip_near;              ///< near clipping distance so that lidar sensor housings can be transparent to self
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
