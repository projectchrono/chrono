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
// Filter to convert exposure to digital values with considering ISO, through
// camera response function (CRF) (gamma_correct curve or sigmoid function),
//
// 			sigmoid: 		I = sigmoid(a * lg(E) + b),
//			gamma_correct: 	I = a * (lg(E))^\gamma + b,
//
// where I is image intensity (0 to 65535), E is exposure, a is expsr2dv_gain,
// b is expsr2dv_bias and \gamma is expsr2dv_gain
// 
// =============================================================================

#ifndef CHFILTERPHYSCAMERAEXPSRTODV_H
#define CHFILTERPHYSCAMERAEXPSRTODV_H

#include "chrono_sensor/filters/ChFilter.h"
#include <cuda.h>
#include "chrono/core/ChVector3.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_filters
/// @{

/// A filter that convert exposure to digital values with considering ISO, through camera response function (CRF)
class CH_SENSOR_API ChFilterPhysCameraExpsrToDV : public ChFilter {
	public:
		/// Class constructor
		/// @param ISO (ISO) analog amplification factor
		/// @param expsr2dv_gain_vec ChVector of proportional gains
		/// @param expsr2dv_bias_vec ChVector of bias or intercept
		/// @param expsr2dv_gamma (gamma) gamma of the gamma correction function if used
		/// @param crf_type type of camera response function (CRF). 0: "gamma_correct", 1: "sigmoid"
		/// @param name The string name of the filter
		ChFilterPhysCameraExpsrToDV(float ISO, ChVector3f expsr2dv_gain_vec, ChVector3f expsr2dv_bias_vec,
			float expsr2dv_gamma, int crf_type = 0, std::string name = "Vignetting Filter in Phys Camera"
		);

		/// Apply function
		virtual void Apply();

		/// Set control parameters in the filter function
		/// @param ISO (ISO) analog amplification factor
		void SetFilterCtrlParameters(float ISO);

		/// Set model parameters in the filter function
		/// @param expsr2dv_gain_vec ChVector of proportional gains
		/// @param expsr2dv_bias_vec ChVector of bias or intercept
		/// @param expsr2dv_gamma (gamma) gamma of the gamma correction function if used
		/// @param crf_type type of camera response function (CRF). 0: "gamma_correct", 1: "sigmoid"
		void SetFilterModelParameters(ChVector3f expsr2dv_gain_vec, ChVector3f expsr2dv_bias_vec,
			float expsr2dv_gamma, int crf_type
		);

		/// Initializes all data needed by the filter access apply function.
		/// @param pSensor A pointer to the sensor on which the filter is attached.
		/// @param bufferInOut A buffer that is passed into the filter. This data is what will be made available for the user.
		virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

	private:
		float m_ISO;											// (ISO) analog amplification factor
		float m_expsr2dv_gains[3];								// proportional gains
		float m_expsr2dv_biases[3];								// biases or intercepts
		float m_expsr2dv_gamma;									// (gamma) gamma of the gamma correction function if used
		int m_crf_type;											// type of camera response function (CRF). 0: "gamma_correct", 1: "sigmoid"
		std::shared_ptr<SensorDeviceHalf4Buffer> m_buffer_in;	// input buffer for RGBA(Half4)
		std::shared_ptr<SensorDeviceHalf4Buffer> m_buffer_out;	// output bufffer for RGBA16
		CUstream m_cuda_stream;									// reference to the cuda stream
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
