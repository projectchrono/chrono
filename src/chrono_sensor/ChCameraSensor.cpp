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

#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterOptixRender.h"
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
                                             int use_gi                        // 1 to use Global Illumination
                                             )
    : m_hFOV(hFOV),
      m_supersample_factor(supersample_factor),
      m_lens_model_type(lens_model),
      ChOptixSensor(parent, updateRate, offsetPose, w * supersample_factor, h * supersample_factor, use_gi) {
    // set the program to match the model requested
    switch (lens_model) {
        case SPHERICAL:
            m_program_string = {"camera", "fov_lens_camera"};
            m_buffer_format = RT_FORMAT_FLOAT4;
            // m_buffer_format = RT_FORMAT_UNSIGNED_BYTE4;
            break;

        default:  // same as PINHOLEP
        {
            if (use_gi) {
                m_program_string = {"camera", "pinhole_gi_camera"};
                m_buffer_format = RT_FORMAT_FLOAT4;
            } else {
                m_program_string = {"camera", "pinhole_camera"};
                m_buffer_format = RT_FORMAT_FLOAT4;
            }
            // m_buffer_format = RT_FORMAT_UNSIGNED_BYTE4;
            break;
        }
    }

    // convert from float4 to rgba8 format
    m_filters.push_back(chrono_types::make_shared<ChFilterImageFloat4ToRGBA8>());

    if (m_supersample_factor > 1) {
        m_filters.push_back(chrono_types::make_shared<ChFilterImgAlias>(m_supersample_factor));
        // m_filters.push_back(chrono_types::make_shared<ChFilterImageResize>(w, h));
    }

    // list of parameters to pass to the ray generation program
    m_ray_launch_params.push_back(
        std::make_tuple<std::string, RTobjecttype, void*>("hFOV", RT_OBJECTTYPE_FLOAT, &m_hFOV));

    SetCollectionWindow(0);
    SetLag(1 / updateRate);
}

// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChCameraSensor::~ChCameraSensor() {}

}  // namespace sensor
}  // namespace chrono
