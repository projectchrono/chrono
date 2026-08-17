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
// ============================================================================

#include "chrono_sensor/sensors/ChRadarSensor.h"

namespace chrono {
namespace sensor {

ChRadarSensor::ChRadarSensor(std::shared_ptr<ChBody> parent,
                             float updateRate,
                             ChFrame<double> offsetPose,
                             unsigned int w,  // image width (# of rays)
                             unsigned int h,  // image height (# of rays)
                             float hfov,
                             float vfov,
                             float max_distance,
                             float clip_near)
#if defined(CHRONO_HAS_OPTIX)
    : ChOptixSensor(parent, updateRate, offsetPose, w, h),
#elif defined(CHRONO_HAS_VULKAN_RT)
    : ChVulkanSensor(parent, updateRate, offsetPose, w, h, VulkanPipelineType::RADAR),
#else
    : ChSensor(parent, updateRate, offsetPose),
#endif
      m_hFOV(hfov),
      m_vFOV(vfov),
      m_max_distance(max_distance),
      m_clip_near(clip_near) {
#ifdef CHRONO_HAS_OPTIX
    m_pipeline_type = PipelineType::RADAR;
#endif

    SetCollectionWindow(0.f);
    SetLag(1.f / updateRate);
}

ChRadarSensor::~ChRadarSensor() {}

}  // namespace sensor
}  // namespace chrono