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
// Authors: Han Wang
// =============================================================================
//
// Container class for a lidar sensor
//
// ============================================================================

#include "chrono_sensor/ChRadarSensor.h"


namespace chrono{
namespace sensor{


CH_SENSOR_API ChRadarSensor::ChRadarSensor(
    std::shared_ptr<chrono::ChBody> parent,
    float updateRate, 
    chrono::ChFrame<double> offsetPose,
    unsigned int w, 
    unsigned int h,
    float hfov,
    float max_vertical_angle,
    float min_vertical_angle,
    float max_distance,
    float clip_near
)
: m_vertical_samples(h),
  m_horizontal_samples(w),
  m_hFOV(hfov),
  m_max_vert_angle(max_vertical_angle),
  m_min_vert_angle(min_vertical_angle),
  m_max_distance(max_distance),
  m_clip_near(clip_near),
  ChOptixSensor(parent, updateRate, offsetPose, w,h){
  m_program_string = {"radar", "spherical"};

  m_ray_launch_params.push_back(
      std::make_tuple<std::string, RTobjecttype, void*>("hFOV", RT_OBJECTTYPE_FLOAT, &m_hFOV));
  m_ray_launch_params.push_back(
      std::make_tuple<std::string, RTobjecttype, void*>("max_vert_angle", RT_OBJECTTYPE_FLOAT, &m_max_vert_angle));
  m_ray_launch_params.push_back(
      std::make_tuple<std::string, RTobjecttype, void*>("min_vert_angle", RT_OBJECTTYPE_FLOAT, &m_min_vert_angle));
  m_ray_launch_params.push_back(
      std::make_tuple<std::string, RTobjecttype, void*>("max_distance", RT_OBJECTTYPE_FLOAT, &m_max_distance));
  m_ray_launch_params.push_back(
      std::make_tuple<std::string, RTobjecttype, void*>("clip_near", RT_OBJECTTYPE_FLOAT, &m_clip_near));

  m_buffer_format = RT_FORMAT_FLOAT2;

  SetCollectionWindow(0);
  SetLag(1 / updateRate);
  }

  // -----------------------------
  // Destructor
  // -----------------------------
  CH_SENSOR_API ChRadarSensor::~ChRadarSensor(){}
}
}