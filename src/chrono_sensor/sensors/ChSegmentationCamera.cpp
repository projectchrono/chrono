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
// Container class for a camera sensor
//
// =============================================================================

#include "chrono_sensor/sensors/ChSegmentationCamera.h"

namespace chrono {
namespace sensor {

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChSegmentationCamera::ChSegmentationCamera(std::shared_ptr<chrono::ChBody> parent,
                                                         float updateRate,
                                                         chrono::ChFrame<double> offsetPose,
                                                         unsigned int w,                  // image width
                                                         unsigned int h,                  // image height
                                                         float hFOV,                      // horizontal field of view
                                                         CameraLensModelType lens_model)  // lens model to use
    : m_hFOV(hFOV),
      m_lens_model_type(lens_model),
      m_lens_parameters({0.f, 0.f, 0.f}),
      ChOptixSensor(parent, updateRate, offsetPose, w, h) {
    // set the pipeline for this
    m_pipeline_type = PipelineType::SEGMENTATION;

    SetCollectionWindow(0.f);
    SetLag(1.f / updateRate);
}

// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChSegmentationCamera::~ChSegmentationCamera() {}

void ChSegmentationCamera::SetRadialLensParameters(ChVector<float> params) {
    float b1 = -params.x();
    float b2 = 3.f * params.x() * params.x() - params.y();
    float b3 = 8.f * params.x() * params.y() - 12.f * params.x() * params.x() * params.x() - params.z();
    m_lens_parameters = {b1, b2, b3};
}

}  // namespace sensor
}  // namespace chrono
