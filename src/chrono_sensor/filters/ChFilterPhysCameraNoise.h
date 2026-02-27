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
// Filter to add noises in the phys camera sensor. The noise model considers
// photon shot noise (governed by Poisson distribution), dark current and hot
// pixels (also governed by Poisson distribution), and read noise and fixed
// pattern noise (FPN) (governed by Gaussian distribution). The noise model is
// depicted as follows,
// 
// I = L + N_read
// 
// L ~ Poisson(\lambda + D * t), where \lambda is average number of illumination
// eletrons, D is temporal dark current and hot pixels in [electrons/sec], and t is exposure time in [sec].
// N_read ~ Normal(0, \sigma_{read}), where \sigma_{read} is STD of read and
// fixed pattern noises
//
// =============================================================================

#ifndef CHFILTERPHYSCAMERANOISE_H
#define CHFILTERPHYSCAMERANOISE_H

#include "chrono_sensor/filters/ChFilter.h"
#include <cuda.h>
#include <curand.h>
#include <curand_kernel.h>
#include "chrono/core/ChVector3.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_filters
/// @{

/// A filter that adjust the brightness of the image according to exposure time and sensitivity coefficients
class CH_SENSOR_API ChFilterPhysCameraNoise : public ChFilter {
	public:
		/// Class constructor
		/// @param expsr_time (t) exposure time
		/// @param dark_current_vec ChVector of temporal means of dark currents
		/// @param noise_gain_vec ChVector of temporal noise gains
		/// @param STD_read_vec ChVector of standard deviations of read and FPN noises, equivalent [electrons]
		/// @param FPN_seed seed of random number generator for getting read and FPN noises
		/// @param name The string name of the filter
		ChFilterPhysCameraNoise(float expsr_time, ChVector3f dark_current_vec, ChVector3f noise_gain_vec,
			ChVector3f STD_read_vec, unsigned int FPN_seed, std::string name = "Noise Model Filter in Phys Camera"
		);

		/// Apply function
		virtual void Apply();

		/// Set control parameters in the filter
		/// @param expsr_time (t) exposure time
		void SetFilterCtrlParameters(float expsr_time);

		/// Set model parameters in the filter
		/// @param dark_current_vec ChVector of temporal means of dark currents
		/// @param noise_gain_vec ChVector of temporal noise gains
		/// @param STD_read_vec ChVector of standard deviations of read and FPN noises, equivalent [electrons]
		void SetFilterModelParameters(
			ChVector3f dark_current_vec, ChVector3f noise_gain_vec, ChVector3f STD_read_vec
		);

		/// Initializes all data needed by the filter access apply function.
		/// @param pSensor A pointer to the sensor on which the filter is attached.
		/// @param bufferInOut A buffer that is passed into the filter. This data is what will be made available for the user.
		virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

	private:
		float m_expsr_time; 						// (t) exposure time
		float m_dark_currents[3];					// temporal mean of dark currents
		float m_noise_gains[3];						// temporal noise gains
		float m_STD_reads[3];						// standard deviations of read and FPN noises, equivalent [electrons]
		unsigned int m_FPN_seed;					// seed of random number generator for getting read and FPN noises
		std::shared_ptr<curandState_t> m_rng_shot;	// cuda random number generator for shot and dark current noises
		std::shared_ptr<curandState_t> m_rng_FPN;	// cuda random number generator for read and FPN noises
		std::shared_ptr<SensorDeviceHalf4Buffer> m_in_out;	// input/output buffer for RGBA(Half4)
		CUstream m_cuda_stream;								// reference to the cuda stream
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
