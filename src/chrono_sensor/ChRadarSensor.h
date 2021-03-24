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
                  float updateRate,
                  chrono::ChFrame<double> offsetPose,
                  unsigned int w,
                  unsigned int h,
                  float hfov,
                  float max_vertical_angle,
                  float min_vertical_angle,
                  float max_distance,
                  float clip_near = 1e-3f);

    /// Class destructor
    ~ChRadarSensor();

    float GetHFOV() const { return m_hFOV; }

    float GetMaxVertAngle() const { return m_max_vert_angle; }

    float GetMinVertAngle() const { return m_min_vert_angle; }

    float GetMaxDistance() const { return m_max_distance; }

    float GetClipNear() const { return m_clip_near; }

  private:
    unsigned int m_vertical_samples;
    unsigned int m_horizontal_samples;
    float m_hFOV;
    float m_max_vert_angle;
    float m_min_vert_angle;
    float m_max_distance;
    float m_clip_near;
};

}  // namespace sensor
}  // namespace chrono

#endif