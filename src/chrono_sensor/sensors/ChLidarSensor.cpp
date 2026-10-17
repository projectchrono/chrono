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

#ifdef CHRONO_HAS_OPTIX
    #include "chrono_sensor/optix/ChFilterOptixRender.h"
#endif

namespace chrono {
namespace sensor {

ChLidarSensor::ChLidarSensor(
    std::shared_ptr<ChBody> parent,
    float updateRate,
    ChFrame<double> offsetPose,
    unsigned int w,               // image width
    unsigned int h,               // image height
    float hFOV,                   // horizontal field of view
    float max_vertical_angle,     // highest vertical angle
    float min_vertical_angle,     // lowest ray angle
    float max_distance,           // maximum distance for lidar
    LidarBeamShape beam_shape,    // beam shape, only rectangular and elliptical are supported
    unsigned int sample_radius,   // radius of the beam samples
    float vert_divergence_angle,  // vertical divergence angle of the beam
    float hori_divergence_angle,  // horizontal divergence angle of the beam
    LidarReturnMode return_mode,  // return mode of the lidar
    float clip_near  // minimum return distance, for making nearby objects transparent when placed inside housing
    )
#if defined(CHRONO_HAS_OPTIX)
    : ChOptixSensor(parent, updateRate, offsetPose, w * (2 * sample_radius - 1), h * (2 * sample_radius - 1)),
#elif defined(CHRONO_HAS_VULKAN_RT)
    : ChVulkanSensor(parent,
                     updateRate,
                     offsetPose,
                     w * (2 * sample_radius - 1),
                     h * (2 * sample_radius - 1),
                     sample_radius > 1 ? VulkanPipelineType::LIDAR_MULTI : VulkanPipelineType::LIDAR_SINGLE),
#elif defined(CHRONO_HAS_METAL_RT)
    // Metal keeps the native w x h grid; beam sub-sampling (sample_radius) and
    // return-mode reduction are done in the shader, so no dimension inflation
    // or separate reduce filter is needed.
    : ChMetalSensor(parent, updateRate, offsetPose, w, h, MetalPipelineType::LIDAR_SINGLE),
#else
    : ChSensor(parent, updateRate, offsetPose),
#endif
      m_hFOV(hFOV),
      m_max_vert_angle(max_vertical_angle),
      m_min_vert_angle(min_vertical_angle),
      m_max_distance(max_distance),
      m_sample_radius(sample_radius),
      m_beam_shape(beam_shape),
      m_vert_divergence_angle(vert_divergence_angle),
      m_hori_divergence_angle(hori_divergence_angle),
      m_return_mode(return_mode),
      m_clip_near(clip_near) {
    if (sample_radius > 1) {
#ifdef CHRONO_HAS_OPTIX
        m_pipeline_type = PipelineType::LIDAR_MULTI;
#endif
#if !defined(CHRONO_HAS_METAL_RT)
        PushFilter(chrono_types::make_shared<ChFilterLidarReduce>(return_mode, sample_radius, "lidar reduction"));
#endif
    } else {
#ifdef CHRONO_HAS_OPTIX
        m_pipeline_type = PipelineType::LIDAR_SINGLE;
#endif
    }

    SetCollectionWindow(0.f);
    SetLag(1.f / updateRate);
}

ChLidarSensor::~ChLidarSensor() {}

}  // namespace sensor
}  // namespace chrono
