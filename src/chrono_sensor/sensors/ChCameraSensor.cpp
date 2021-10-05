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

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/optix/ChFilterOptixRender.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"

namespace chrono {
namespace sensor {

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChCameraSensor::ChCameraSensor(std::shared_ptr<chrono::ChBody> parent,
                                             float updateRate,
                                             chrono::ChFrame<double> offsetPose,
                                             unsigned int w,                   // image width
                                             unsigned int h,                   // image height
                                             float hFOV,                       // horizontal field of view
                                             unsigned int supersample_factor,  // super sampling factor
                                             CameraLensModelType lens_model,   // lens model to use
                                             bool use_gi,                      // 1 to use Global Illumination
                                             float gamma                       // 1 for linear color space, 2.2 for sRGB
                                             )
    : m_hFOV(hFOV),
      m_supersample_factor(supersample_factor),
      m_lens_model_type(lens_model),
      m_use_gi(use_gi),
      m_gamma(gamma),
      ChOptixSensor(parent, updateRate, offsetPose, w * supersample_factor, h * supersample_factor) {
    // set the program to match the model requested
    switch (lens_model) {
        case CameraLensModelType::FOV_LENS:
            m_pipeline_type = PipelineType::CAMERA_FOV_LENS;
            break;
        default:  // default to CameraLensModelType::PINHOLE
            m_pipeline_type = PipelineType::CAMERA_PINHOLE;
            break;
    }

    m_filters.push_back(chrono_types::make_shared<ChFilterImageHalf4ToRGBA8>());

    if (m_supersample_factor > 1) {
        m_filters.push_back(
            chrono_types::make_shared<ChFilterImgAlias>(m_supersample_factor, "Image antialias filter"));
    }

    SetCollectionWindow(0.f);
    SetLag(1.f / updateRate);
}

// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChCameraSensor::~ChCameraSensor() {}

}  // namespace sensor
}  // namespace chrono
