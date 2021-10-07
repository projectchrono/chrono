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

CH_SENSOR_API ChRadarSensor::ChRadarSensor(std::shared_ptr<chrono::ChBody> parent,  // object to attach to
                                           float updateRate,
                                           chrono::ChFrame<double> offsetPose,
                                           unsigned int w,  // image width (# of rays)
                                           unsigned int h,  // image height (# of rays)
                                           float hfov,
                                           float vfov,
                                           float max_distance,
                                           float clip_near)
    : m_hFOV(hfov),
      m_vFOV(vfov),
      m_max_distance(max_distance),
      m_clip_near(clip_near),
      ChOptixSensor(parent, updateRate, offsetPose, w, h) {
      m_pipeline_type = PipelineType::RADAR;

    SetCollectionWindow(0);
    SetLag(1 / updateRate);
}

// -----------------------------
// Destructor
// -----------------------------
CH_SENSOR_API ChRadarSensor::~ChRadarSensor() {}
}  // namespace sensor
}  // namespace chrono