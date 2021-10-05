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
    : m_hFOV(hFOV), m_lens_model_type(lens_model), ChOptixSensor(parent, updateRate, offsetPose, w, h) {
    // set the program to match the model requested
    switch (lens_model) {
        case CameraLensModelType::FOV_LENS:
            m_pipeline_type = PipelineType::SEGMENTATION_FOV_LENS;
            break;
        default:  // default to CameraLensModelType::PINHOLE
            m_pipeline_type = PipelineType::SEGMENTATION_PINHOLE;
            break;
    }

    SetCollectionWindow(0.f);
    SetLag(1.f / updateRate);
}

// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChSegmentationCamera::~ChSegmentationCamera() {}

}  // namespace sensor
}  // namespace chrono
