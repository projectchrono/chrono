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
// Container class for a radar sensor
//
// =============================================================================

#ifndef CHRADARSENSOR_H
#define CHRADARSENSOR_H

#include "chrono_sensor/ChOptixSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

enum class RadarReturnMode {
    RadarReturn,
    RadarTrack
};

/// Radar Class/ This corresponds to a fmcw radar
class CH_SENSOR_API ChRadarSensor : public ChOptixSensor {
    /// Constructor for the base radar class
    /// @param parent Body to which the sensor is attached
    /// @param w Width in number of samples of a radar scan
    /// @param h Height in number of samples of a radar scan
    /// @param clip_near Near clipping distance so that lidar sensor can be easily placed inside a visualization object
    /// (sensor housing)
    /// @param hfov Horizontal field of view of the lidar
    /// @param max_vertical_angle Maximum vertical angle of the lidar
    /// @param min_vertical_angle Minimum vertical angle of the lidar

  public:
    ChRadarSensor(std::shared_ptr<chrono::ChBody> parent,
                  const float updateRate,
                  chrono::ChFrame<double> offsetPose,
                  const unsigned int w,
                  const unsigned int h,
                  const float hfov,
                  const float max_vertical_angle,
                  const float min_vertical_angle,
                  const float max_distance,
                  RadarReturnMode return_mode,
                  const float clip_near = 1e-3f
    );

    /// Class destructor
    ~ChRadarSensor();

    /// Gives the horizontal field of view of the radar (angle between right-most and left-most ray for a "frame").
    /// Horizontal field of view should be 360-(angular resulution) in degrees for a full 360 degree scanning radar.
    /// @return The horizontal field of view of the radar sensor
    float GetHFOV() const { return m_hFOV; }

    /// Returns the highest vertical angle of any ray in the radar
    /// @return The angle of the highest ray
    float GetMaxVertAngle() const { return m_max_vert_angle; }

    /// Returns the lowest vertical angle of any ray in the radar
    /// @return The angle of the lowest ray
    float GetMinVertAngle() const { return m_min_vert_angle; }

    /// Returns the maximum range of the radar
    /// @return The maximum distance for the radar
    float GetMaxDistance() const { return m_max_distance; }

    /// Returns the near clipping distance
    /// @return the near clipping distance
    float GetClipNear() const { return m_clip_near; }

    RadarReturnMode GetReturnMode() const {return m_return_mode;}

    /// Returns the translational velocity of the object the radar is attached to
    /// @return Returns the translational velocity of the object the radar is attached to
    ChVector<double> GetTranslationalVelocity() { return m_parent->GetPos_dt(); }

    /// Returns the angular velocity of the object the radar is attached to
    /// @return Returns the angular velocity of the object the radar is attached to
    ChVector<double> GetAngularVelocity() { return m_parent->GetWvel_loc(); }

  private:
    float m_hFOV;            ///< the horizontal field of view of the radar
    float m_max_vert_angle;  ///< max vertical angle of the rays
    float m_min_vert_angle;  ///< min vertical angle of the rays
    float m_max_distance;    ///< max distance for radar
    float m_clip_near;       ///< near clipping distance so that radar sensor housings can be transparent to self
    RadarReturnMode m_return_mode;
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif