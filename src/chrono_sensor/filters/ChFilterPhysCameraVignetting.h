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
// Filter to do vignetting based on camera control parameters
// E_out =  E_in * (1 - G_vignet + G_vignet * (cos(theta))^4)
// 
// =============================================================================

#ifndef CHFILTERPHYSCAMERAVIGNETTING_H
#define CHFILTERPHYSCAMERAVIGNETTING_H

#include "chrono_sensor/filters/ChFilter.h"
#include <cuda.h>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_filters
/// @{

/// A filter that adjust the brightness of the image according to exposure time and sensitivity coefficients
class CH_SENSOR_API ChFilterPhysCameraVignetting : public ChFilter {
	public:
		/// Class constructor
		/// @param focal_length (f) focal length
		/// @param sensor_width (L) sensor width
		/// @param vignetting_gain (G_vignet) proportional gain of irradiance falloff
		/// @param name The string name of the filter
		ChFilterPhysCameraVignetting(float focal_length, float sensor_width, float vignetting_gain,
			std::string name = "Vignetting Filter in Phys Camera"
		);

		/// Apply function
		virtual void Apply();

		/// Set control parameters in the filter function
		/// @param focal_length (f) focal length
		void SetFilterCtrlParameters(float focal_length);

		/// Set model parameters in the filter function
		/// @param sensor_width (L) sensor width
		/// @param vignetting_gain (P_vignet) proportional gain of irradiance falloff
		void SetFilterModelParameters(float sensor_width, float vignetting_gain);


		/// Initializes all data needed by the filter access apply function.
		/// @param pSensor A pointer to the sensor on which the filter is attached.
		/// @param bufferInOut A buffer that is passed into the filter. This data is what will be made available for the user.
		virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

	private:
		float m_focal_length;									// (f) focal length
		float m_sensor_width;									// (L) sensor width
		float m_vignetting_gain;								// proportional gain of irradiance falloff
		std::shared_ptr<SensorDeviceHalf4Buffer> m_in_out;		///< input/output buffer for RGBA(Half4)
		CUstream m_cuda_stream;									///< reference to the cuda stream
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
