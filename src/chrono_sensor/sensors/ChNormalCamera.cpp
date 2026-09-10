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
// Authors: Bo-Hsun Chen, Nevindu M. Batagoda
// =============================================================================
//
// Container class for the normal camera sensor
//
// =============================================================================

#include "chrono_sensor/sensors/ChNormalCamera.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

namespace chrono {
namespace sensor {

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChNormalCamera::ChNormalCamera(std::shared_ptr<chrono::ChBody> parent,
                                            float updateRate,
                                            chrono::ChFrame<double> offsetPose,
                                            unsigned int w,                  // image width
                                            unsigned int h,                  // image height
                                            float hFOV,                      // horizontal field of view
                                            CameraLensModelType lens_model)  // lens model to use
#if defined(CHRONO_HAS_OPTIX)
    : ChOptixSensor(parent, updateRate, offsetPose, w, h),
#elif defined(CHRONO_HAS_VULKAN_RT)
    : ChVulkanSensor(parent, updateRate, offsetPose, w, h, VulkanPipelineType::NORMAL_CAMERA),
#elif defined(CHRONO_HAS_METAL_RT)
    : ChMetalSensor(parent, updateRate, offsetPose, w, h, MetalPipelineType::NORMAL_CAMERA),
#else
    : ChSensor(parent, updateRate, offsetPose),
#endif
      m_hFOV(hFOV), m_lens_model_type(lens_model), m_lens_parameters({}) {
#ifdef CHRONO_HAS_OPTIX
    m_pipeline_type = PipelineType::NORMAL_CAMERA;
#endif
    m_filters.push_back(chrono_types::make_shared<ChFilterNormalAccess>());

    m_filters.push_back(chrono_types::make_shared<ChFilterNormalToRGBA8>());

    SetCollectionWindow(0.f);
    SetLag(1.f / updateRate);
}

// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChNormalCamera::~ChNormalCamera() {}

void ChNormalCamera::SetRadialLensParameters(ChVector3f params) {
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
