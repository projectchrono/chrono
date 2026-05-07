// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// Filter to do defocus blur based on camera control parameters and depth map
// 
// =============================================================================

#ifndef CHFILTERPHYSCAMERADEFOCUSBLUR_H
#define CHFILTERPHYSCAMERADEFOCUSBLUR_H

#include "chrono_sensor/filters/ChFilter.h"
#include <cuda.h>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_filters
/// @{

/// A filter that adjust the brightness of the image according to exposure time and sensitivity coefficients
class CH_SENSOR_API ChFilterPhysCameraDefocusBlur : public ChFilter {
	public:
		/// Class constructor
		/// @param focal_length (f) focal length
		/// @param focus_dist (U) focus distance
		/// @param aperture_num (N) F-number (or aperture number) = focal_length / aperture_diameter
		/// @param pixel_size (C) length of a pixel
		/// @param defocus_gain proportional gain
		/// @param defocus_bias [px], defocus-blur diameter bias
		/// @param name The string name of the filter
		ChFilterPhysCameraDefocusBlur(
			float focal_length, float focus_dist, float aperture_num, float pixel_size, float defocus_gain,
			float defocus_bias, std::string name = "Defocus Blur Filter in Phys Camera"
		);

		/// Apply function
		virtual void Apply();

		/// Set control parameters in the filter function
		/// @param focal_length (f) focal length
		/// @param focus_dist (U) focus distance
		/// @param aperture_num (N) F-number (or aperture number) = focal_length / aperture_diameter
		void SetFilterCtrlParameters(float focal_length, float focus_dist, float aperture_num);

		/// Set model parameters in the filter function
		/// @param pixel_size (C) length of a pixel
		/// @param defocus_gain (defocus_gain) proportional gain
		/// @param defocus_bias [px], (defocus_bias) bias
		void SetFilterModelParameters(float pixel_size, float defocus_gain, float defocus_bias);

		/// Initializes all data needed by the filter access apply function.
		/// @param pSensor A pointer to the sensor on which the filter is attached.
		/// @param bufferInOut A buffer that is passed into the filter. This data is what will be made available for the user.
		virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    float m_focal_length;										// focal length
	float m_focus_dist;											// focus distance
    float m_pixel_size;											// length of a pixel
    float m_aperture_num;										// F-number (or aperture number) = focal_length / aperture_diameter
    float m_defocus_gain;										// proportional gain of defocus-blur diameter
	float m_defocus_bias;										// [px], bias of defocus-blur diameter
	std::shared_ptr<SensorDeviceRGBDHalf4Buffer> m_buffer_in;	///< input buffer for rgbd(half4)
    std::shared_ptr<SensorDeviceHalf4Buffer> m_buffer_out;  	///< output buffer for rgba(half4)
    CUstream m_cuda_stream;                                 	///< reference to the cuda stream
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
