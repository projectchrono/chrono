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

#ifndef CHPHYSCAMERASENSOR_H
#define CHPHYSCAMERASENSOR_H

#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/filters/ChFilterPhysCameraDefocusBlur.h"
#include "chrono_sensor/filters/ChFilterPhysCameraVignetting.h"
#include "chrono_sensor/filters/ChFilterPhysCameraAggregator.h"
#include "chrono_sensor/filters/ChFilterPhysCameraNoise.h"
#include "chrono_sensor/filters/ChFilterPhysCameraExpsrToDV.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// Camera class
class CH_SENSOR_API ChPhysCameraSensor : public ChOptixSensor {
	public:
		/// @brief Constructor for the phys camera class that defaults to a pinhole lens model
		/// @param parent A shared pointer to a body on which the sensor should be attached.
		/// @param updateRate The desired update rate of the sensor in Hz.
		/// @param offsetPose The desired relative position and orientation of the sensor on the body.
		/// @param w The width of the image the camera should generate.
		/// @param h The height of the image the camera should generate.
		/// @param lens_model A enum specifying the desired lens model.
		/// @param sensor_width equivalent width of the image sensor to calculate hHOV, [mm]
		/// @param pixel_size length of a pixel to calculate depth of field, [mm]
		/// @param supersample_factor The number of rays that should be sampled per pixel for antialiasing.
		/// @param use_diffuse_reflect whether to consider diffuse reflection in rendering. If false, only consider specular reflection.
		/// @param use_denoiser whether to use OptiX denoiser for diffuse reflection or area light.
		/// @param use_defocus_blur whether to use defocus blur effect
		/// @param use_vignetting whether to use vignetting effect
		/// @param use_aggregator whether to aggregate illumination irradiance over exposure time and pixel area
		/// @param use_expsr_to_dv whether to convert exposure domain to digital value domain
		/// @param integrator The type of integrator algorithm to use for rendering.
		/// @param gamma correction of the image, 1 for linear color space, 2.2 for sRGB
		/// @param use_fog whether to use fog on this camera
		/// @param use_motion_blur whether to use motion blur effect
		ChPhysCameraSensor(
			std::shared_ptr<chrono::ChBody> parent,                         // object to which the sensor is attached
			float updateRate,                                               // rate at which the sensor updates
			chrono::ChFrame<double> offsetPose,                             // position of sensor relative to parent object
			unsigned int w,                                                 // image width, [px]
			unsigned int h,                                                 // image height, [px]
			CameraLensModelType lens_model = CameraLensModelType::PINHOLE,  // lens distortion model for rendering
			unsigned int supersample_factor = 1,                            // number of samples per pixel for antialiasing
			bool use_diffuse_reflect = false,								// whether to consider diffuse reflection in rendering
			bool use_denoiser = false,										// whether to use a denoiser for considering diffuse reflection or area light
			bool use_defocus_blur = false,                                  // whether to activate defocus blur
			bool use_vignetting = false,                                    // whether to activate vignetting
			bool use_aggregator = false,									// whether to aggregate irradiance
			bool use_noise = false,											// whether to add noises
			bool use_expsr_to_dv = false,									// whether to convert exposure to digital values
			Integrator integrator = Integrator::LEGACY,						// integrator algorithm to use for rendering
			float gamma = 1.0f,												// gamma correction value
			bool use_fog = false,											// whether to use fog
			bool use_motion_blur = false									// whether to open motion blur effect
		);  

		/// phys-camera class destructor
		~ChPhysCameraSensor();

		/// Sets the parameters for a radial lens distortion model
		/// Parameters should be given for the forward model
		/// The backward distortion model will then be used and calculated from the forward parameters given
		/// @param params the set of 3 radial parameters (k1, k2, k3)
		void SetRadialLensParameters(ChVector3f params);

		/// Set camera control parameters
		/// @param aperture_num F-number (or aperture number) = focal_length / aperture_diameter, [1/1]
		/// @param expsr_time exposure time, [sec]
		/// @param ISO ISO exposure gain, [1/1]
		/// @param focal_length focal length, [m]
		/// @param focus_dist focus distance, [m]
		void SetCtrlParameters(float aperture_num, float expsr_time, float ISO, float focal_length, float focus_dist);

		/// Set camera intrinsic model parameters
		/// @param sensor_width equivalent width of the image sensor, [m]
		/// @param pixel_size length of a pixel, [m]
		/// @param max_scene_light_amount maximum light amount in the scene, consider distance-diminishing effect [lux = lumen/m^2]
		/// @param rgb_QE_vec vector of RGB quantum efficiencies, [1/1]
		/// @param gain_params all gain-related parameters in camera model, [1/1]
		/// @param noise_params noise model parameters in camera model
		void SetModelParameters(
			float sensor_width, float pixel_size, float max_scene_light_amount, ChVector3f rgb_QE_vec,
			PhysCameraGainParams gain_params, PhysCameraNoiseParams noise_params
		);

		/// returns the camera's horizontal field of view. Vertical field of view is determined by the image aspect
		/// ratio and the lens model
		/// @return The horizontal field of view of the camera lens, [rad]
		float GetHFOV() const { return m_hFOV; }

		/// returns the lens model type used for rendering
		/// @return An enum specifying which lens model is being used. (0: PINHOLE, 1: FOV, 2: Radial)
		CameraLensModelType GetLensModelType() const { return m_lens_model_type; }

		/// returns the lens model parameters
		/// @return LensParams struct of lens parameters. Will default to zeros for any terms not used. These are coverted for the inverse model
		LensParams GetLensParameters() const { return m_lens_parameters; }

		/// returns if the cemera considers diffuse reflection
		/// @return True if it does consider
		bool GetUseGI() { return m_use_gi; }

		/// returns if the cemera should use a denoiser
		/// @return True if it does use a denoiser
		bool GetUseDenoiser() { return m_use_denoiser; }

		/// Get the integrator algorithm type used for rendering
		/// @return the integrator algorithm type used for rendering
		Integrator GetIntegrator() { return m_integrator; }

		/// returns the gamma correction value of this camera.
		/// 1 means no correction and the image is in linear color space. Useful for other ML applications
		/// 2.2 means the image is in sRGB color space. Useful for display
		/// @return Gamma value of the image
		float GetGamma() { return m_gamma; }

		/// Gets the number of samples per pixels in each direction used for super sampling
		/// @return the number of samples per pixel
		unsigned int GetSampleFactor() { return m_supersample_factor; }

		/// returns if the cemera should use fog as dictated by the scene
		/// @return True if it does request
		bool GetUseFog() { return m_use_fog; }

		/// Get 3x3 Intrinsic Matrix
		ChMatrix33<float> GetCameraIntrinsicMatrix();
		
		/// Get the camera distortion coefficients
		ChVector3f GetCameraDistortionCoefficients() { return m_distortion_params; }

		/// Get the aperture number
		/// @return the aperture number, [1/1]
		float GetApertureNum() { return m_aperture_num; }

		/// Get the exposure time
		/// @return the exposure time, [sec]
		float GetExpsrTime() { return m_expsr_time; }

		/// Get the ISO gain
		/// @return the ISO, [1/1]
		float GetISO() { return m_ISO; }

		/// Get the focal length
		/// @return the focal length, [m]
		float GetFocalLength() { return m_focal_length; }

		/// Get the focus distance
		/// @return the focus distance, [m]
		float GetFocusDistance() { return m_focus_dist; }

		/// Get the sensor width
		/// @return the sensor width, [m]
		float GetSensorWidth() { return m_sensor_width; }

		/// Get the pixel size
		/// @return the pixel size, [m]
		float GetPixelSize() { return m_pixel_size; }

		/// Get the maximum light amount in the scene
		/// @return the maximum light amount in the scene, [W]
		float GetMaxSceneLightAmount() { return m_max_scene_light_amount; }

		/// Get all gain-related parameters in camera model
		/// @return all gain-related parameters in camera model, [1/1]
		PhysCameraGainParams GetGainParams() { return m_gain_params; }

		/// Get noise model parameters in camera model
		/// @return noise model parameters in camera model, [1/1]
		PhysCameraNoiseParams GetNoiseParams() { return m_noise_params; }

		/// update filter parameters in ChOptixEngine.cpp
		void UpdateFilterParameters();

	private:
		/// calculate the parameters for the inverse polynomial model
		static LensParams CalcInvRadialModel(ChVector3f params);

		// camera basic parameters
		float m_hFOV;								///< the horizontal field of view (FOV) of the sensor, [rad]
		unsigned int m_supersample_factor;			///< super sampling factor for antialiasing
		float m_gamma;								///< holds the camera's gamma value
		CameraLensModelType m_lens_model_type;		///< lens model used by the camera
		LensParams m_lens_parameters;				///< lens parameters when applicable
		float m_width;								///< width of the image formed, [px]
		float m_height;								///< height of the image formed, [px]
		ChVector3f m_distortion_params;		        ///< radial distortion parameters (k1, k2, k3)
		Integrator m_integrator;                    ///< the type of integrator algorithm used for rendering

		// switches to activate artifacts
		bool m_use_gi;								///< holds whether to consider diffuse reflection in rendering. If false, only consider specular reflection.
		bool m_use_denoiser;						///< holds whether to use a denoiser for considering diffuse reflection or area light
		bool m_use_fog;								///< holds whether the camera follows the scene fog model
		bool m_use_motion_blur;						///< holds whether to open motion blur effect
		bool m_use_defocus_blur;					///< holds whether to activate defocus blur
		bool m_use_vignetting;						///< holds whether to activate vignetting
		bool m_use_aggregator;						///< holds whether to activate irradiance aggregation
		bool m_use_noise;							///< holds whether to add noises
		bool m_use_expsr_to_dv;						///< holds whether to convert exposure to digital value

		//// camera control parameters
		float m_aperture_num;						///< F-number (or aperture number) = focal_length / aperture_diameter, [1/1]
		float m_expsr_time;                     	///< exposure time, [sec]
		float m_ISO;                            	///< ISO exposure gain, [1/1]
		float m_focal_length;                   	///< focal length, [m]
		float m_focus_dist;                     	///< focus distance, [m]

		//// camera intrinsic model parameters
		float m_sensor_width;                   	///< equivalent width of the image sensor, [m]
		float m_pixel_size;                     	///< length of a pixel, [m]
		float m_max_scene_light_amount;				///< maximum brightness amount of a light source in the scene, [W]
		ChVector3f m_rgb_QE_vec;				///< vector of RGB quantum efficiencies, [1/1]
		ChVector3f m_expsr2dv_gain_vec;		///< vector of proportional gains in expsr_to_dv filter, [1/1]
		ChVector3f m_expsr2dv_bias_vec;		///< vector of biases in expsr_to_dv filter, [dv]
		ChVector3f m_noise_dark_current_vec;	///< vector of dark currents in noise filter, [electron/sec]
		ChVector3f m_noise_gain_vec;			///< vector of temporal noise gains in noise filter, [1/1]
		ChVector3f m_noise_STD_read_vec;		///< vector of STDs of FPN and readout noises in noise filter, [electron]
		PhysCameraGainParams m_gain_params;		  	///< all gain-related parameters in camera model, [1/1]
		PhysCameraNoiseParams m_noise_params;		///< noise model parameters in camera model

		//// pointers to filters
		std::shared_ptr<ChFilterPhysCameraDefocusBlur> m_defocus_blur_ptr;	///< pointer to the defocus blur filter
		std::shared_ptr<ChFilterPhysCameraVignetting> m_vignetting_ptr;		///< pointer to the vignetting filter
		std::shared_ptr<ChFilterPhysCameraAggregator> m_aggregator_ptr;		///< pointer to the aggregator filter
		std::shared_ptr<ChFilterPhysCameraNoise> m_noise_ptr;				///< pointer to the noise model filter
		std::shared_ptr<ChFilterPhysCameraExpsrToDV> m_expsr_to_dv_ptr;		///< pointer to the exposure-to-digital-value filter
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
