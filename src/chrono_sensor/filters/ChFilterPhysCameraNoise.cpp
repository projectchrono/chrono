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
// =============================================================================

#include "chrono_sensor/filters/ChFilterPhysCameraNoise.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/cuda/phys_cam_ops.cuh"
#include "chrono_sensor/cuda/curand_utils.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include <chrono>

namespace chrono {
namespace sensor {
ChFilterPhysCameraNoise::ChFilterPhysCameraNoise(
    float expsr_time, ChVector3f dark_current_vec, ChVector3f noise_gain_vec, ChVector3f STD_read_vec,
    unsigned int FPN_seed, std::string name
) :
    m_expsr_time(expsr_time), m_FPN_seed(FPN_seed), ChFilter(name)
{
    m_dark_currents[0] = dark_current_vec.x();
    m_dark_currents[1] = dark_current_vec.y();
    m_dark_currents[2] = dark_current_vec.z();
    
    m_noise_gains[0] = noise_gain_vec.x();
    m_noise_gains[1] = noise_gain_vec.y();
    m_noise_gains[2] = noise_gain_vec.z();

    m_STD_reads[0] = STD_read_vec.x();
    m_STD_reads[1] = STD_read_vec.y();
    m_STD_reads[2] = STD_read_vec.z();
};

CH_SENSOR_API void ChFilterPhysCameraNoise::Initialize(
    std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut
) {
    if (!bufferInOut) {
        InvalidFilterGraphNullBuffer(pSensor);
    }

    if (auto pRGBAHalf4 = std::dynamic_pointer_cast<SensorDeviceHalf4Buffer>(bufferInOut)) {
        m_in_out = pRGBAHalf4;
    }
    else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }

    m_rng_shot = std::shared_ptr<curandState_t>(
        cudaMallocHelper<curandState_t>(bufferInOut->Width * bufferInOut->Height), cudaFreeHelper<curandState_t>
    );
    init_cuda_rng(
        (unsigned int)(std::chrono::high_resolution_clock::now().time_since_epoch().count()), m_rng_shot.get(),
        bufferInOut->Width * bufferInOut->Height
    );
    m_rng_FPN = std::shared_ptr<curandState_t>(
        cudaMallocHelper<curandState_t>(bufferInOut->Width * bufferInOut->Height), cudaFreeHelper<curandState_t>
    );
    init_cuda_rng(m_FPN_seed, m_rng_FPN.get(), bufferInOut->Width * bufferInOut->Height);


    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    }
}

CH_SENSOR_API void ChFilterPhysCameraNoise::Apply() {    
    cuda_phys_cam_noise(
		m_in_out->Buffer.get(), m_in_out->Width, m_in_out->Height, m_expsr_time, m_dark_currents, m_noise_gains,
        m_STD_reads, m_rng_shot.get(), m_rng_FPN.get(), m_cuda_stream
	);
}

CH_SENSOR_API void ChFilterPhysCameraNoise::SetFilterCtrlParameters(float expsr_time) {
    m_expsr_time = expsr_time;
}

CH_SENSOR_API void ChFilterPhysCameraNoise::SetFilterModelParameters(
    ChVector3f dark_current_vec, ChVector3f noise_gain_vec, ChVector3f STD_read_vec
) {
	m_dark_currents[0] = dark_current_vec.x();
    m_dark_currents[1] = dark_current_vec.y();
    m_dark_currents[2] = dark_current_vec.z();
    
    m_noise_gains[0] = noise_gain_vec.x();
    m_noise_gains[1] = noise_gain_vec.y();
    m_noise_gains[2] = noise_gain_vec.z();

    m_STD_reads[0] = STD_read_vec.x();
    m_STD_reads[1] = STD_read_vec.y();
    m_STD_reads[2] = STD_read_vec.z();
}


}  // namespace sensor
}  // namespace chrono
