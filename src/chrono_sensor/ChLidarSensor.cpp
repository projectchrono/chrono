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

#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/filters/ChFilterLidarReduce.h"
#include "chrono_sensor/filters/ChFilterLidarIntensityClip.h"
#include "chrono_sensor/filters/ChFilterOptixRender.h"

namespace chrono {
namespace sensor {

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChLidarSensor::ChLidarSensor(
    std::shared_ptr<chrono::ChBody> parent,
    float updateRate,
    chrono::ChFrame<double> offsetPose,
    unsigned int w,               // image width
    unsigned int h,               // image height
    float hFOV,                   // horizontal field of view
    float max_vertical_angle,     // highest vertical angle
    float min_vertical_angle,     // lowest ray angle
    float max_distance,           // maximum distance for lidar
    unsigned int sample_radius,   // radius of the beam samples
    float divergence_angle,       // divergence angle of the beam
    LidarReturnMode return_mode,  // return mode of the lidar
    LidarModelType lidar_model,   // lidar model for generating data
    float clip_near  // minimum return distance, for making nearby objects transparent when placed inside housing
    )
    : m_sample_radius(sample_radius),
      m_divergence_angle(divergence_angle),
      m_return_mode(return_mode),
      m_hFOV(hFOV),
      m_max_vert_angle(max_vertical_angle),
      m_min_vert_angle(min_vertical_angle),
      m_max_distance(max_distance),
      m_model_type(lidar_model),
      m_clip_near(clip_near),
      ChOptixSensor(parent, updateRate, offsetPose, w * (2 * sample_radius - 1), h * (2 * sample_radius - 1)) {
    // set the program to match the model requested
    switch (lidar_model) {
        default:  // same as RAYCAST
            // select if we should use multisample ray casting method
            if (sample_radius > 1) {
                m_program_string = {"lidar", "multi_sample"};
                m_ray_launch_params.push_back(std::make_tuple<std::string, RTobjecttype, void*>(
                    "divergence_angle", RT_OBJECTTYPE_FLOAT, &m_divergence_angle));
                m_ray_launch_params.push_back(std::make_tuple<std::string, RTobjecttype, void*>(
                    "ray_samples", RT_OBJECTTYPE_INT, &m_sample_radius));
                m_filters.push_back(
                    chrono_types::make_shared<ChFilterLidarReduce>(return_mode, sample_radius, "lidar reduction"));

            } else {
                m_program_string = {"lidar", "spherical"};
            }

            m_buffer_format = RT_FORMAT_FLOAT2;
            break;
    }

    // list of parameters to pass to the ray generation program
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

    SetCollectionWindow(0);
    SetLag(1 / updateRate);
}
// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChLidarSensor::~ChLidarSensor() {}

}  // namespace sensor
}  // namespace chrono
