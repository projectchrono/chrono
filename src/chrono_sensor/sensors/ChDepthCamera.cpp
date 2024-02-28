// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nevindu M. Batagoda, Asher Elmquist
// =============================================================================
//
// Container class for a camera sensor
//
// =============================================================================

#include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"

namespace chrono {
namespace sensor {

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChDepthCamera::ChDepthCamera(std::shared_ptr<chrono::ChBody> parent,
                                                         float updateRate,
                                                         chrono::ChFrame<double> offsetPose,
                                                         unsigned int w,                  // image width
                                                         unsigned int h,                  // image height
                                                         float hFOV,                      // horizontal field of view
                                                         CameraLensModelType lens_model)  // lens model to use
    : m_hFOV(hFOV),
      m_lens_model_type(lens_model),
      m_lens_parameters({}),
      ChOptixSensor(parent, updateRate, offsetPose, w, h) {
    // set the pipeline for this
    m_pipeline_type = PipelineType::DEPTH_CAMERA;


    m_filters.push_back(chrono_types::make_shared<ChFilterDepthToRGBA8>());

    SetCollectionWindow(0.f);
    SetLag(1.f / updateRate);
}

// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChDepthCamera::~ChDepthCamera() {}

void ChDepthCamera::SetRadialLensParameters(ChVector<float> params) {
    // Drap, P., & Lefèvre, J. (2016). 
    // An Exact Formula for Calculating Inverse Radial Lens Distortions. 
    // Sensors (Basel, Switzerland), 16(6), 807. https://doi.org/10.3390/s16060807
    // float a1_2 = params.x();
    // float b1 = -params.x();
    // float b2 = 3.f * params.x() * params.x() - params.y();
    // float b3 = 8.f * params.x() * params.y() - 12.f * a1_2 * params.x() - params.z();
    // float b4 = 55 * a1_2 * a1_2 + 10*params.x()*params.z() - 55*a1_2*params.z() + 5*params.y()*params.y();
    // m_lens_parameters = {b1, b2, b3, b4};
    m_lens_parameters = ChCameraSensor::CalcInvRadialModel(params);
}

}  // namespace sensor
}  // namespace chrono
