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

#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/filters/ChFilterLidarReduce.h"
#include "chrono_sensor/optix/ChFilterOptixRender.h"

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
    LidarBeamShape beam_shape,    // beam shape, only rectangular and eliptical are supported
    unsigned int sample_radius,   // radius of the beam samples
    float vert_divergence_angle,  // vertical divergence angle of the beam
    float hori_divergence_angle,  // horizontal divergence angle of the beam
    LidarReturnMode return_mode,  // return mode of the lidar
    float clip_near  // minimum return distance, for making nearby objects transparent when placed inside housing
    )
    : m_sample_radius(sample_radius),
      m_vert_divergence_angle(vert_divergence_angle),
      m_hori_divergence_angle(hori_divergence_angle),
      m_return_mode(return_mode),
      m_hFOV(hFOV),
      m_max_vert_angle(max_vertical_angle),
      m_min_vert_angle(min_vertical_angle),
      m_max_distance(max_distance),
      m_clip_near(clip_near),
      m_beam_shape(beam_shape),
      ChOptixSensor(parent, updateRate, offsetPose, w * (2 * sample_radius - 1), h * (2 * sample_radius - 1)) {
    if (sample_radius > 1) {
        m_pipeline_type = PipelineType::LIDAR_MULTI;
        PushFilter(chrono_types::make_shared<ChFilterLidarReduce>(return_mode, sample_radius, "lidar reduction"));
    } else {  // default to single ray sampled lidar
        m_pipeline_type = PipelineType::LIDAR_SINGLE;
    }

    SetCollectionWindow(0);
    SetLag(1 / updateRate);
}
// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChLidarSensor::~ChLidarSensor() {}

}  // namespace sensor
}  // namespace chrono
