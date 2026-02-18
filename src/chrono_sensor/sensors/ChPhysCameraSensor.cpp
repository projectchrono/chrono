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
// Authors: Bo-Hsun Chen
// =============================================================================
//
// Container class for a physics-based high-fidelity camera sensor with 
// several camera model and control parameters
//
// =============================================================================

#include "chrono_sensor/sensors/ChPhysCameraSensor.h"
#include "chrono_sensor/optix/ChFilterOptixRender.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include <math.h> 

namespace chrono {
namespace sensor {

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChPhysCameraSensor::ChPhysCameraSensor(
    std::shared_ptr<chrono::ChBody> parent,
    float updateRate,
    chrono::ChFrame<double> offsetPose,
    unsigned int w,						// image width
    unsigned int h,						// image height
    CameraLensModelType lens_model,		// lens model to use
    unsigned int supersample_factor,	// super sampling factor
    bool use_diffuse_reflect,        	// true to consider diffuse reflection in rendering, false to only consider specular reflection
    bool use_denoiser,					// whether to use OptiX denoiser
    bool use_defocus_blur,              // whether to activate defocus blur
    bool use_vignetting,                // whether to activate vignetting
    bool use_aggregator,				// whether to activate illumination irradiance aggregation
    bool use_noise,                     // whether to add noises
    bool use_expsr_to_dv,               // whether to convert exposure to digital values
    Integrator integrator,              // integrator algorithm to use for rendering
    float gamma,						// 1.0 for linear color space, 2.2 for sRGB
    bool use_fog,						// whether to use fog on this camera
    bool use_motion_blur        		// whether to use motion blur effect
) :
    m_supersample_factor(supersample_factor),
    m_lens_model_type(lens_model),
    m_use_gi(use_diffuse_reflect),
    m_use_denoiser(use_denoiser),
    m_use_defocus_blur(use_defocus_blur),
    m_use_vignetting(use_vignetting),
    m_use_aggregator(use_aggregator),
    m_use_noise(use_noise),
    m_use_expsr_to_dv(use_expsr_to_dv),
    m_integrator(integrator),
    m_gamma(gamma),
    m_use_fog(use_fog),
    m_use_motion_blur(use_motion_blur),
    m_width(w),
    m_height(h),
	m_aperture_num(1.6f),          
    m_expsr_time(0.512f),
    m_ISO(100.0f),
    m_focal_length(0.012f),
    m_focus_dist(3.0f),
	m_max_scene_light_amount(250.f),
	m_sensor_width(0.013f),
    m_pixel_size(5.86e-6),
    m_rgb_QE_vec({1.0f, 1.0f, 1.0f}), // m_rgb_QE_vec({1.f, 1.f, 1.f})
    m_distortion_params({0.f, 0.f, 0.f}),
    m_lens_parameters({}),
	m_gain_params({}),
	m_noise_params({}),
    ChOptixSensor(parent, updateRate, offsetPose, w, h)
{
    //
    m_pipeline_type = PipelineType::PHYS_CAMERA;

    m_gain_params.defocus_gain = 4.0f;
    m_gain_params.defocus_bias = 0.f;
    m_gain_params.vignetting_gain = 2.0f;
    // m_gain_params.aggregator_gain = 1.0f / 0.16f / 0.16f / 0.512f / 5.86e-6 / 5.86e-6 / 1000.f;
    // m_gain_params.aggregator_gain = 2.221748e9;
    // m_gain_params.aggregator_gain = 1e8 * 6.5536; // w/o aggregator
    m_gain_params.aggregator_gain = 1e7; // w/ aggregator, w/o CRF
    m_gain_params.expsr2dv_gamma = 1.0f;
    m_gain_params.expsr2dv_crf_type = 2; // 0: gamma_correct, 1:sigmoid, 2: linear
    m_gain_params.expsr2dv_gains = {1.0f, 1.0f, 1.0f};
    m_gain_params.expsr2dv_biases = {0.f, 0.f, 0.f};  
    m_expsr2dv_gain_vec.Set(m_gain_params.expsr2dv_gains.x, m_gain_params.expsr2dv_gains.y, m_gain_params.expsr2dv_gains.z);
    m_expsr2dv_bias_vec.Set(m_gain_params.expsr2dv_biases.x, m_gain_params.expsr2dv_biases.y, m_gain_params.expsr2dv_biases.z);

    m_noise_params.dark_currents = {1.0f, 1.0f, 1.0f}; // [electrons/sec]
    m_noise_params.noise_gains = {0.1f, 0.1f, 0.1f}; // [1/1]
    m_noise_params.STD_reads = {0.f, 0.f, 0.f}; // [electrons]
    m_noise_dark_current_vec.Set(m_noise_params.dark_currents.x, m_noise_params.dark_currents.y, m_noise_params.dark_currents.z);
    m_noise_gain_vec.Set(m_noise_params.noise_gains.x, m_noise_params.noise_gains.y, m_noise_params.noise_gains.z);
    m_noise_STD_read_vec.Set(m_noise_params.STD_reads.x, m_noise_params.STD_reads.y, m_noise_params.STD_reads.z); 
    m_noise_params.FPN_rng_seed = 1234;

	// calculate inferred parameters in camera.cu
    m_hFOV = 2.f * atanf(0.5f * m_sensor_width / m_focal_length); // [rad]

    // ------------------------------------------------------ //
    // push back filters to build up the physics-based camera //
    // ------------------------------------------------------ //
    if (m_use_defocus_blur == true) {
        m_defocus_blur_ptr = chrono_types::make_shared<ChFilterPhysCameraDefocusBlur>(
            m_focal_length, m_focus_dist, m_aperture_num, m_pixel_size, m_gain_params.defocus_gain, m_gain_params.defocus_bias
        );
        // m_defocus_blur_ptr->SetFilterParameters(m_focal_length, m_focus_dist, m_aperture_num, m_pixel_size, 1.0);
	    m_filters.push_back(m_defocus_blur_ptr);
    }
    else {
        m_filters.push_back(chrono_types::make_shared<ChFilterRGBDHalf4ToImageHalf4>("RGBD(Half4) to Image(Half4) filter"));
        // m_filters.push_back(chrono_types::make_shared<ChFilterRGBDHalf4ToR8>("RGBD(Half4) to R8 filter"));
    }
    if (m_use_vignetting == true) {
        m_vignetting_ptr = chrono_types::make_shared<ChFilterPhysCameraVignetting>(
            m_focal_length, m_sensor_width, m_gain_params.vignetting_gain
        );
	    m_filters.push_back(m_vignetting_ptr);
    }
    if (m_use_aggregator == true) {
        m_aggregator_ptr = chrono_types::make_shared<ChFilterPhysCameraAggregator>(
            m_aperture_num, m_expsr_time, m_pixel_size, m_max_scene_light_amount, m_rgb_QE_vec,
            m_gain_params.aggregator_gain
        );
	    m_filters.push_back(m_aggregator_ptr);
    }
    if (m_use_noise == true) {
        m_noise_ptr = chrono_types::make_shared<ChFilterPhysCameraNoise>(
            m_expsr_time, m_noise_dark_current_vec, m_noise_gain_vec, m_noise_STD_read_vec, m_noise_params.FPN_rng_seed
        );
	    m_filters.push_back(m_noise_ptr);
    }
    if (m_use_expsr_to_dv == true) {
        m_expsr_to_dv_ptr = chrono_types::make_shared<ChFilterPhysCameraExpsrToDV>(
            m_ISO, m_expsr2dv_gain_vec, m_expsr2dv_bias_vec, m_gain_params.expsr2dv_gamma,
            m_gain_params.expsr2dv_crf_type 
        );
	    m_filters.push_back(m_expsr_to_dv_ptr);
    }

    // if (m_use_expsr_to_dv == false) {
    m_filters.push_back(chrono_types::make_shared<ChFilterImageHalf4ToRGBA16>("Image(Half4) to RGBA16 filter"));
    // }
    // m_filters.push_back(chrono_types::make_shared<ChFilterImageHalf4ToRGBA8>("Image(Half4) to RGBA8 filter"));

    SetCollectionWindow((m_use_motion_blur ? m_expsr_time : 0.f));
    SetLag(1.f / updateRate);

    // m_defocus_blur_ptr->SetFilterParameters(..., m_aperture_num, m_focal_length, m_focus_dist, m_pixel_size);
    // m_vignetting_ptr->SetFilterParameters(..., ...);
    // m_aggregator_ptr->SetFilterParameters(..., m_expsr_time, m_aperture_num);
    // m_noise_ptr->SetFilterParameters(m_noise_params);
    // m_expsr_to_dv_ptr->SetFilterParameters(..., m_ISO);
    // SetCollectionWindow((use_motion_blur ? m_expsr_time : 0.f));
}

// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
CH_SENSOR_API ChPhysCameraSensor::~ChPhysCameraSensor() {}


void ChPhysCameraSensor::SetRadialLensParameters(ChVector3f params) {
    m_distortion_params = params;
    m_lens_parameters = CalcInvRadialModel(params);
}


ChMatrix33<float> ChPhysCameraSensor::GetCameraIntrinsicMatrix() {
    ChMatrix33<float> I;
    float focal_length = (m_width / 2) * tanf(m_hFOV / 2);
    I(0, 0) = focal_length;
    I(0, 1) = 0.f;
    I(0, 2) = m_width / 2;
    I(1, 0) = 0.f;
    I(1, 1) = focal_length;
    I(1, 2) = m_height / 2;
    I(2, 0) = 0.f;
    I(2, 1) = 0.f;
    I(2, 2) = 1.f;

    return I;
};


LensParams ChPhysCameraSensor::CalcInvRadialModel(ChVector3f params) {
    // coefficients directory without algorithm from 
    // Drap, P., & Lefèvre, J. (2016). 
    // An Exact Formula for Calculating Inverse Radial Lens Distortions. 
    // Sensors (Basel, Switzerland), 16(6), 807. https://doi.org/10.3390/s16060807

    LensParams p = {};
    
    double k1 = params.x();
    double k21 = k1*k1;
    double k31 = k21*k1;
    double k41 = k21*k21;
    double k51 = k31*k21;
    double k61 = k31*k31;
    double k71 = k41*k31;
    double k81 = k41*k41;
    double k91 = k51*k41;

    double k2 = params.y();
    double k22 = k2*k2;
    double k32 = k2*k22;
    double k42 = k22*k22;

    double k3 = params.z();
    double k23 = k3*k3;
    double k33 = k23*k3;

    //k4 = 0

    p.a0 = -k1;
    p.a1 = (float)(3*k21 - k2);
    p.a2 = (float)(-12*k31 + 8*k1*k2 - k3);
    p.a3 = (float)(55*k41 - 55*k21*k2 + 5*k22 + 10*k1*k3);
    p.a4 = (float)(-273*k51 + 364*k31*k2- 78*k1*k22 - 78*k21*k3 + 12*k2*k3);
    p.a5 = (float)(1428*k61 - 2380*k41*k2 + 840*k21*k22 - 35*k32 + 560*k31*k3 - 210*k1*k2*k3 + 7*k23);
    p.a6 = (float)(-7752*k71 + 15504*k51*k2 - 7752*k31*k22 + 816*k1*k32 - 3876*k41*k3 + 2448*k21*k2*k3 - 136*k22*k3 - 136*k1*k23);
    p.a7 = (float)(43263*k81 - 100947*k61*k2 + 65835*k41*k22 - 11970*k21*k32 + 285*k42 + 26334*k51*k3 - 23940*k31*k2*k3
        + 3420*k1*k22*k3 + 1710*k21*k23 - 171*k2*k23);
    p.a8 = (float)(-246675*k91 + 657800*k71*k2 - 531300*k51*k22 + 141680*k31*k32 - 8855*k1*k42 - 177100*k61*k3
        + 212520*k41*k2*k3 - 53130*k21*k22*k3 + 1540*k32*k3 - 17710*k31*k23 + 4620*k1*k2*k23 - 70*k33);

    return p;
}


void ChPhysCameraSensor::SetCtrlParameters(
    float aperture_num, float expsr_time, float ISO, float focal_length, float focus_dist
) {
    // update control parameters
    m_aperture_num = aperture_num;
    m_expsr_time = expsr_time;
    m_ISO = ISO;
    m_focal_length = focal_length;
    m_focus_dist = focus_dist;
    
    // update inferred parameters
    m_hFOV = 2.f * atanf(0.5f * m_sensor_width / m_focal_length);
    SetCollectionWindow((m_use_motion_blur ? m_expsr_time : 0.f));

    // update control parameters in filters
    if (m_use_defocus_blur == true) {
        m_defocus_blur_ptr->SetFilterCtrlParameters(m_focal_length, m_focus_dist, m_aperture_num);
    }
    if (m_use_vignetting == true) {
        m_vignetting_ptr->SetFilterCtrlParameters(m_focal_length);
    }
	if (m_use_aggregator == true) {
        m_aggregator_ptr->SetFilterCtrlParameters(m_aperture_num, m_expsr_time);
    }
    if (m_use_noise == true) {
        m_noise_ptr->SetFilterCtrlParameters(m_expsr_time);
    }
    if (m_use_expsr_to_dv == true) {
        m_expsr_to_dv_ptr->SetFilterCtrlParameters(m_ISO);
    }
}


void ChPhysCameraSensor::SetModelParameters(
    float sensor_width, float pixel_size, float max_scene_light_amount, ChVector3f rgb_QE_vec,
    PhysCameraGainParams gain_params, PhysCameraNoiseParams noise_params
) {
    // update model parameters
    m_sensor_width = sensor_width;
    m_pixel_size = pixel_size;
    m_max_scene_light_amount = max_scene_light_amount;
    m_rgb_QE_vec.Set(rgb_QE_vec);
    m_gain_params = gain_params;
    m_noise_params = noise_params;
    
    m_expsr2dv_gain_vec.Set(m_gain_params.expsr2dv_gains.x, m_gain_params.expsr2dv_gains.y, m_gain_params.expsr2dv_gains.z);
    m_expsr2dv_bias_vec.Set(m_gain_params.expsr2dv_biases.x, m_gain_params.expsr2dv_biases.y, m_gain_params.expsr2dv_biases.z);
    
    m_noise_dark_current_vec.Set(m_noise_params.dark_currents.x, m_noise_params.dark_currents.y, m_noise_params.dark_currents.z);
    m_noise_gain_vec.Set(m_noise_params.noise_gains.x, m_noise_params.noise_gains.y, m_noise_params.noise_gains.z);
    m_noise_STD_read_vec.Set(m_noise_params.STD_reads.x, m_noise_params.STD_reads.y, m_noise_params.STD_reads.z); 

    // update inferred parameters
    m_hFOV = 2.f * atanf(0.5f * m_sensor_width / m_focal_length);

    // update model parameters in filters
    if (m_use_defocus_blur == true) {
        m_defocus_blur_ptr->SetFilterModelParameters(m_pixel_size, m_gain_params.defocus_gain, m_gain_params.defocus_bias);
    }
    if (m_use_vignetting == true) {
        m_vignetting_ptr->SetFilterModelParameters(m_sensor_width, m_gain_params.vignetting_gain);
    }
	if (m_use_aggregator == true) {
        m_aggregator_ptr->SetFilterModelParameters(
            m_pixel_size, m_max_scene_light_amount, m_rgb_QE_vec, m_gain_params.aggregator_gain
        );
    }
    if (m_use_noise == true) {
        m_noise_ptr->SetFilterModelParameters(m_noise_dark_current_vec, m_noise_gain_vec, m_noise_STD_read_vec);
    }
    if (m_use_expsr_to_dv == true) {
        m_expsr_to_dv_ptr->SetFilterModelParameters(
            m_expsr2dv_gain_vec, m_expsr2dv_bias_vec, m_gain_params.expsr2dv_gamma, m_gain_params.expsr2dv_crf_type
        );
    }
}


void ChPhysCameraSensor::UpdateFilterParameters() {
    // if (m_use_defocus_blur == true) {
    //     m_defocus_blur_ptr->SetFilterCtrlParameters(m_focal_length, m_focus_dist, m_aperture_num);
    //     m_defocus_blur_ptr->SetFilterModelParameters(m_pixel_size, m_gain_params.defocus_gain);
    // }
    // if (m_use_vignetting == true) {
    //     m_vignetting_ptr->SetFilterCtrlParameters(m_focal_length);
    //     m_vignetting_ptr->SetFilterModelParameters(m_sensor_width, m_gain_params.vignetting_gain);
    // }
	// if (m_use_aggregator == true) {
    //     m_aggregator_ptr->SetFilterCtrlParameters(m_aperture_num, m_expsr_time);
    //     m_aggregator_ptr->SetFilterModelParameters(m_gain_params.aggregator_gain);
    // }

    // 
    // m_aggregator_ptr->SetFilterParameters(..., m_expsr_time, m_aperture_num);
    // m_noise_ptr->SetFilterParameters(m_noise_params);
    // m_expsr_to_dv_ptr->SetFilterParameters(..., m_ISO);
    // SetCollectionWindow((use_motion_blur ? m_expsr_time : 0.f));
}
}  // namespace sensor
}  // namespace chrono
